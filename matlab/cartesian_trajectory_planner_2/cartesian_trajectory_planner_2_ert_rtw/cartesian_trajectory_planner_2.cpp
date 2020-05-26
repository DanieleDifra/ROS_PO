//
// Academic License - for use in teaching, academic research, and meeting
// course requirements at degree granting institutions only.  Not for
// government, commercial, or other organizational use.
//
// File: cartesian_trajectory_planner_2.cpp
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
#include "cartesian_trajectory_planner_2.h"
#include "cartesian_trajectory_planner_2_private.h"

// Block signals (default storage)
B_cartesian_trajectory_planne_T cartesian_trajectory_planner__B;

// Block states (default storage)
DW_cartesian_trajectory_plann_T cartesian_trajectory_planner_DW;

// Real-time model
RT_MODEL_cartesian_trajectory_T cartesian_trajectory_planner_M_ =
  RT_MODEL_cartesian_trajectory_T();
RT_MODEL_cartesian_trajectory_T *const cartesian_trajectory_planner_M =
  &cartesian_trajectory_planner_M_;

// Forward declaration for local functions
static void cartesian_trajec_emxInit_char_T(emxArray_char_T_cartesian_tra_T
  **pEmxArray, int32_T numDimensions, B_MATLABSystem_cartesian_traj_T *localB);
static void cartesian_trajec_emxInit_real_T(emxArray_real_T_cartesian_tra_T
  **pEmxArray, int32_T numDimensions, B_MATLABSystem_cartesian_traj_T *localB);
static void emxInitStruct_c_rigidBodyJoint(c_rigidBodyJoint_cartesian_tr_T
  *pStruct, B_MATLABSystem_cartesian_traj_T *localB);
static void emxInitStruct_v_robotics_manip_(v_robotics_manip_internal_Rig_T
  *pStruct, B_MATLABSystem_cartesian_traj_T *localB);
static void emxInitStruct_y_robotics_manip_(y_robotics_manip_internal_Rig_T
  *pStruct, B_MATLABSystem_cartesian_traj_T *localB);
static void emxInitStruct_b_inverseKinemati(b_inverseKinematics_cartesian_T
  *pStruct, B_MATLABSystem_cartesian_traj_T *localB);
static void emxInitStruct_robotics_slmanip_(robotics_slmanip_internal_blo_T
  *pStruct, B_MATLABSystem_cartesian_traj_T *localB);
static void emxInitStruct_w_robotics_manip_(w_robotics_manip_internal_Rig_T
  *pStruct, B_MATLABSystem_cartesian_traj_T *localB);
static void emxInitStruct_x_robotics_manip_(x_robotics_manip_internal_Rig_T
  *pStruct, B_MATLABSystem_cartesian_traj_T *localB);
static void emxInitStruct_f_robotics_manip_(f_robotics_manip_internal_IKE_T
  *pStruct, B_MATLABSystem_cartesian_traj_T *localB);
static void emxInitStruct_h_robotics_core_i(h_robotics_core_internal_Damp_T
  *pStruct, B_MATLABSystem_cartesian_traj_T *localB);
static void cartesia_twister_state_vector_l(uint32_T mt[625]);
static void cartesian_tr_eml_rand_mt19937ar(uint32_T state[625]);
static void cartesian_traject_seed_to_state(uint32_T state[2]);
static void cartesian_tra_eml_rand_shr3cong(uint32_T state[2]);
static void cartes_emxEnsureCapacity_char_T(emxArray_char_T_cartesian_tra_T
  *emxArray, int32_T oldNumel, B_MATLABSystem_cartesian_traj_T *localB);
static void cartesian_trajec_emxFree_char_T(emxArray_char_T_cartesian_tra_T
  **pEmxArray);
static void cartes_emxEnsureCapacity_real_T(emxArray_real_T_cartesian_tra_T
  *emxArray, int32_T oldNumel, B_MATLABSystem_cartesian_traj_T *localB);
static v_robotics_manip_internal_Rig_T *cartesian_t_RigidBody_RigidBody
  (v_robotics_manip_internal_Rig_T *obj, B_MATLABSystem_cartesian_traj_T *localB);
static v_robotics_manip_internal_Rig_T *cartesian_RigidBody_RigidBody_b
  (v_robotics_manip_internal_Rig_T *obj, B_MATLABSystem_cartesian_traj_T *localB);
static v_robotics_manip_internal_Rig_T *cartesia_RigidBody_RigidBody_by
  (v_robotics_manip_internal_Rig_T *obj, B_MATLABSystem_cartesian_traj_T *localB);
static v_robotics_manip_internal_Rig_T *cartesi_RigidBody_RigidBody_bya
  (v_robotics_manip_internal_Rig_T *obj, B_MATLABSystem_cartesian_traj_T *localB);
static v_robotics_manip_internal_Rig_T *cartes_RigidBody_RigidBody_byap
  (v_robotics_manip_internal_Rig_T *obj, B_MATLABSystem_cartesian_traj_T *localB);
static v_robotics_manip_internal_Rig_T *carte_RigidBody_RigidBody_byapf
  (v_robotics_manip_internal_Rig_T *obj, B_MATLABSystem_cartesian_traj_T *localB);
static v_robotics_manip_internal_Rig_T *cart_RigidBody_RigidBody_byapfa
  (v_robotics_manip_internal_Rig_T *obj, B_MATLABSystem_cartesian_traj_T *localB);
static y_robotics_manip_internal_Rig_T *car_RigidBodyTree_RigidBodyTree
  (y_robotics_manip_internal_Rig_T *obj, v_robotics_manip_internal_Rig_T *iobj_0,
   v_robotics_manip_internal_Rig_T *iobj_1, v_robotics_manip_internal_Rig_T
   *iobj_2, v_robotics_manip_internal_Rig_T *iobj_3,
   v_robotics_manip_internal_Rig_T *iobj_4, v_robotics_manip_internal_Rig_T
   *iobj_5, v_robotics_manip_internal_Rig_T *iobj_6,
   v_robotics_manip_internal_Rig_T *iobj_7, B_MATLABSystem_cartesian_traj_T
   *localB);
static void cartesi_genrand_uint32_vector_d(uint32_T mt[625], uint32_T u[2],
  B_MATLABSystem_cartesian_traj_T *localB);
static boolean_T cartesian_trajec_is_valid_state(const uint32_T mt[625],
  B_MATLABSystem_cartesian_traj_T *localB);
static void cartesian__eml_rand_mt19937ar_a(const uint32_T state[625], uint32_T
  b_state[625], real_T *r, B_MATLABSystem_cartesian_traj_T *localB);
static void cartesian_trajectory_plann_rand(real_T r[5],
  B_MATLABSystem_cartesian_traj_T *localB, DW_MATLABSystem_cartesian_tra_T
  *localDW);
static w_robotics_manip_internal_Rig_T *car_RigidBody_RigidBody_byapfac
  (w_robotics_manip_internal_Rig_T *obj, c_rigidBodyJoint_cartesian_tr_T *iobj_0,
   B_MATLABSystem_cartesian_traj_T *localB);
static w_robotics_manip_internal_Rig_T *ca_RigidBody_RigidBody_byapfac3
  (w_robotics_manip_internal_Rig_T *obj, c_rigidBodyJoint_cartesian_tr_T *iobj_0,
   B_MATLABSystem_cartesian_traj_T *localB);
static w_robotics_manip_internal_Rig_T *c_RigidBody_RigidBody_byapfac3i
  (w_robotics_manip_internal_Rig_T *obj, c_rigidBodyJoint_cartesian_tr_T *iobj_0,
   B_MATLABSystem_cartesian_traj_T *localB);
static w_robotics_manip_internal_Rig_T *RigidBody_RigidBody_byapfac3iu
  (w_robotics_manip_internal_Rig_T *obj, c_rigidBodyJoint_cartesian_tr_T *iobj_0,
   B_MATLABSystem_cartesian_traj_T *localB);
static w_robotics_manip_internal_Rig_T *RigidBody_RigidBody_byapfac3iu2
  (w_robotics_manip_internal_Rig_T *obj, c_rigidBodyJoint_cartesian_tr_T *iobj_0,
   B_MATLABSystem_cartesian_traj_T *localB);
static void ca_RigidBodyTree_clearAllBodies(x_robotics_manip_internal_Rig_T *obj,
  w_robotics_manip_internal_Rig_T *iobj_0, w_robotics_manip_internal_Rig_T
  *iobj_1, w_robotics_manip_internal_Rig_T *iobj_2,
  w_robotics_manip_internal_Rig_T *iobj_3, w_robotics_manip_internal_Rig_T
  *iobj_4, w_robotics_manip_internal_Rig_T *iobj_5,
  w_robotics_manip_internal_Rig_T *iobj_6, c_rigidBodyJoint_cartesian_tr_T
  *iobj_7, c_rigidBodyJoint_cartesian_tr_T *iobj_8,
  c_rigidBodyJoint_cartesian_tr_T *iobj_9, c_rigidBodyJoint_cartesian_tr_T
  *iobj_10, c_rigidBodyJoint_cartesian_tr_T *iobj_11,
  c_rigidBodyJoint_cartesian_tr_T *iobj_12, c_rigidBodyJoint_cartesian_tr_T
  *iobj_13, c_rigidBodyJoint_cartesian_tr_T *iobj_14,
  w_robotics_manip_internal_Rig_T *iobj_15, B_MATLABSystem_cartesian_traj_T
  *localB, DW_MATLABSystem_cartesian_tra_T *localDW);
static x_robotics_manip_internal_Rig_T *c_RigidBodyTree_RigidBodyTree_i
  (x_robotics_manip_internal_Rig_T *obj, w_robotics_manip_internal_Rig_T *iobj_0,
   w_robotics_manip_internal_Rig_T *iobj_1, w_robotics_manip_internal_Rig_T
   *iobj_2, w_robotics_manip_internal_Rig_T *iobj_3,
   w_robotics_manip_internal_Rig_T *iobj_4, w_robotics_manip_internal_Rig_T
   *iobj_5, w_robotics_manip_internal_Rig_T *iobj_6,
   c_rigidBodyJoint_cartesian_tr_T *iobj_7, c_rigidBodyJoint_cartesian_tr_T
   *iobj_8, c_rigidBodyJoint_cartesian_tr_T *iobj_9,
   c_rigidBodyJoint_cartesian_tr_T *iobj_10, c_rigidBodyJoint_cartesian_tr_T
   *iobj_11, c_rigidBodyJoint_cartesian_tr_T *iobj_12,
   c_rigidBodyJoint_cartesian_tr_T *iobj_13, c_rigidBodyJoint_cartesian_tr_T
   *iobj_14, c_rigidBodyJoint_cartesian_tr_T *iobj_15,
   w_robotics_manip_internal_Rig_T *iobj_16, B_MATLABSystem_cartesian_traj_T
   *localB, DW_MATLABSystem_cartesian_tra_T *localDW);
static boolean_T cartesian_trajectory_pla_strcmp(const
  emxArray_char_T_cartesian_tra_T *a, const emxArray_char_T_cartesian_tra_T *b);
static c_rigidBodyJoint_cartesian_tr_T *c_rigidBodyJoint_rigidBodyJoint
  (c_rigidBodyJoint_cartesian_tr_T *obj, const emxArray_char_T_cartesian_tra_T
   *jname, const emxArray_char_T_cartesian_tra_T *jtype,
   B_MATLABSystem_cartesian_traj_T *localB);
static void cartesian_trajec_emxFree_real_T(emxArray_real_T_cartesian_tra_T
  **pEmxArray);
static w_robotics_manip_internal_Rig_T *cartesian_trajec_RigidBody_copy(const
  v_robotics_manip_internal_Rig_T *obj, c_rigidBodyJoint_cartesian_tr_T *iobj_0,
  c_rigidBodyJoint_cartesian_tr_T *iobj_1, w_robotics_manip_internal_Rig_T
  *iobj_2, B_MATLABSystem_cartesian_traj_T *localB);
static void cartesian_RigidBodyTree_addBody(x_robotics_manip_internal_Rig_T *obj,
  v_robotics_manip_internal_Rig_T *bodyin, const emxArray_char_T_cartesian_tra_T
  *parentName, c_rigidBodyJoint_cartesian_tr_T *iobj_0,
  c_rigidBodyJoint_cartesian_tr_T *iobj_1, w_robotics_manip_internal_Rig_T
  *iobj_2, B_MATLABSystem_cartesian_traj_T *localB);
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
  c_rigidBodyJoint_cartesian_tr_T *iobj_15, c_rigidBodyJoint_cartesian_tr_T
  *iobj_16, c_rigidBodyJoint_cartesian_tr_T *iobj_17,
  c_rigidBodyJoint_cartesian_tr_T *iobj_18, c_rigidBodyJoint_cartesian_tr_T
  *iobj_19, c_rigidBodyJoint_cartesian_tr_T *iobj_20,
  c_rigidBodyJoint_cartesian_tr_T *iobj_21, c_rigidBodyJoint_cartesian_tr_T
  *iobj_22, c_rigidBodyJoint_cartesian_tr_T *iobj_23,
  c_rigidBodyJoint_cartesian_tr_T *iobj_24, c_rigidBodyJoint_cartesian_tr_T
  *iobj_25, c_rigidBodyJoint_cartesian_tr_T *iobj_26,
  c_rigidBodyJoint_cartesian_tr_T *iobj_27, c_rigidBodyJoint_cartesian_tr_T
  *iobj_28, c_rigidBodyJoint_cartesian_tr_T *iobj_29,
  c_rigidBodyJoint_cartesian_tr_T *iobj_30, c_rigidBodyJoint_cartesian_tr_T
  *iobj_31, c_rigidBodyJoint_cartesian_tr_T *iobj_32,
  c_rigidBodyJoint_cartesian_tr_T *iobj_33, c_rigidBodyJoint_cartesian_tr_T
  *iobj_34, c_rigidBodyJoint_cartesian_tr_T *iobj_35,
  c_rigidBodyJoint_cartesian_tr_T *iobj_36, c_rigidBodyJoint_cartesian_tr_T
  *iobj_37, c_rigidBodyJoint_cartesian_tr_T *iobj_38,
  c_rigidBodyJoint_cartesian_tr_T *iobj_39, w_robotics_manip_internal_Rig_T
  *iobj_40, x_robotics_manip_internal_Rig_T *iobj_41,
  B_MATLABSystem_cartesian_traj_T *localB, DW_MATLABSystem_cartesian_tra_T
  *localDW);
static h_robotics_core_internal_Damp_T *DampedBFGSwGradientProjection_D
  (h_robotics_core_internal_Damp_T *obj);
static void car_inverseKinematics_setupImpl(b_inverseKinematics_cartesian_T *obj,
  f_robotics_manip_internal_IKE_T *iobj_0, B_MATLABSystem_cartesian_traj_T
  *localB);
static void c_inverseKinematics_setPoseGoal(b_inverseKinematics_cartesian_T *obj,
  const real_T tform[16], const real_T weights[6],
  B_MATLABSystem_cartesian_traj_T *localB);
static void RigidBodyTree_validateConfigu_j(x_robotics_manip_internal_Rig_T *obj,
  real_T Q[6], B_MATLABSystem_cartesian_traj_T *localB);
static void ca_rigidBodyJoint_get_JointAxis(const
  c_rigidBodyJoint_cartesian_tr_T *obj, real_T ax[3],
  B_MATLABSystem_cartesian_traj_T *localB);
static void cartesian_trajectory_planne_cat(real_T varargin_1, real_T varargin_2,
  real_T varargin_3, real_T varargin_4, real_T varargin_5, real_T varargin_6,
  real_T varargin_7, real_T varargin_8, real_T varargin_9, real_T y[9]);
static void rigidBodyJoint_transformBodyToP(const
  c_rigidBodyJoint_cartesian_tr_T *obj, real_T T[16],
  B_MATLABSystem_cartesian_traj_T *localB);
static void rigidBodyJoint_transformBodyT_n(const
  c_rigidBodyJoint_cartesian_tr_T *obj, const real_T q_data[], const int32_T
  *q_size, real_T T[16], B_MATLABSystem_cartesian_traj_T *localB);
static void RigidBodyTree_efficientFKAndJac(x_robotics_manip_internal_Rig_T *obj,
  const real_T qv[6], const emxArray_char_T_cartesian_tra_T *body1Name, real_T
  T_data[], int32_T T_size[2], emxArray_real_T_cartesian_tra_T *Jac,
  B_MATLABSystem_cartesian_traj_T *localB);
static creal_T cartesian_trajectory_plann_sqrt(const creal_T x);
static real_T cartesian_trajectory_plan_xnrm2(int32_T n, const real_T x[9],
  int32_T ix0);
static real_T cartesian_trajectory_pl_xnrm2_a(const real_T x[3], int32_T ix0);
static real_T cartesian_trajectory_plan_xdotc(int32_T n, const real_T x[9],
  int32_T ix0, const real_T y[9], int32_T iy0);
static void cartesian_trajectory_plan_xaxpy(int32_T n, real_T a, int32_T ix0,
  const real_T y[9], int32_T iy0, real_T b_y[9]);
static void cartesian_trajectory__xaxpy_cxo(int32_T n, real_T a, const real_T x
  [9], int32_T ix0, real_T y[3], int32_T iy0);
static void cartesian_trajectory_p_xaxpy_cx(int32_T n, real_T a, const real_T x
  [3], int32_T ix0, const real_T y[9], int32_T iy0, real_T b_y[9]);
static void cartesian_trajectory_plan_xrotg(real_T a, real_T b, real_T *b_a,
  real_T *b_b, real_T *c, real_T *s, B_MATLABSystem_cartesian_traj_T *localB);
static void cartesian_trajectory_plann_xrot(const real_T x[9], int32_T ix0,
  int32_T iy0, real_T c, real_T s, real_T b_x[9]);
static void cartesian_trajectory_plan_xswap(const real_T x[9], int32_T ix0,
  int32_T iy0, real_T b_x[9]);
static void cartesian_trajectory_planne_svd(const real_T A[9], real_T U[9],
  real_T s[3], real_T V[9], B_MATLABSystem_cartesian_traj_T *localB);
static void cartesian_trajectory_rotm2axang(const real_T R[9], real_T axang[4],
  B_MATLABSystem_cartesian_traj_T *localB);
static void cartesian_IKHelpers_computeCost(const real_T x[6],
  f_robotics_manip_internal_IKE_T *args, real_T *cost, real_T W[36],
  emxArray_real_T_cartesian_tra_T *Jac, f_robotics_manip_internal_IKE_T **b_args,
  B_MATLABSystem_cartesian_traj_T *localB);
static void cartesian_trajectory_planne_eye(real_T b_I[36]);
static void cartesian_tra_emxInit_boolean_T(emxArray_boolean_T_cartesian__T
  **pEmxArray, int32_T numDimensions);
static void cartesian_traje_emxInit_int32_T(emxArray_int32_T_cartesian_tr_T
  **pEmxArray, int32_T numDimensions);
static void car_emxEnsureCapacity_boolean_T(emxArray_boolean_T_cartesian__T
  *emxArray, int32_T oldNumel);
static void carte_emxEnsureCapacity_int32_T(emxArray_int32_T_cartesian_tr_T
  *emxArray, int32_T oldNumel);
static real_T SystemTimeProvider_getElapsedTi(const
  f_robotics_core_internal_Syst_T *obj);
static real_T cartesian_trajectory_pla_norm_j(const real_T x[6]);
static void cartesian_tra_emxFree_boolean_T(emxArray_boolean_T_cartesian__T
  **pEmxArray);
static boolean_T DampedBFGSwGradientProjection_a(const
  h_robotics_core_internal_Damp_T *obj, const real_T Hg[6], const
  emxArray_real_T_cartesian_tra_T *alpha);
static void cartesian_traje_emxFree_int32_T(emxArray_int32_T_cartesian_tr_T
  **pEmxArray);
static void cartesian_trajectory_pl_xzgetrf(int32_T m, int32_T n, const
  emxArray_real_T_cartesian_tra_T *A, int32_T lda,
  emxArray_real_T_cartesian_tra_T *b_A, emxArray_int32_T_cartesian_tr_T *ipiv,
  int32_T *info, B_MATLABSystem_cartesian_traj_T *localB);
static void cartesian_trajectory_plan_xtrsm(int32_T m, int32_T n, const
  emxArray_real_T_cartesian_tra_T *A, int32_T lda, const
  emxArray_real_T_cartesian_tra_T *B, int32_T ldb,
  emxArray_real_T_cartesian_tra_T *b_B, B_MATLABSystem_cartesian_traj_T *localB);
static real_T cartesian_trajectory_p_xnrm2_ad(int32_T n, const
  emxArray_real_T_cartesian_tra_T *x, int32_T ix0,
  B_MATLABSystem_cartesian_traj_T *localB);
static void cartesian_trajectory_pla_qrpf_b(const
  emxArray_real_T_cartesian_tra_T *A, int32_T m, int32_T n,
  emxArray_real_T_cartesian_tra_T *tau, const emxArray_int32_T_cartesian_tr_T
  *jpvt, emxArray_real_T_cartesian_tra_T *b_A, emxArray_int32_T_cartesian_tr_T
  *b_jpvt, B_MATLABSystem_cartesian_traj_T *localB);
static void cartesian_trajectory_p_mldivide(const
  emxArray_real_T_cartesian_tra_T *A, const emxArray_real_T_cartesian_tra_T *B,
  emxArray_real_T_cartesian_tra_T *Y, B_MATLABSystem_cartesian_traj_T *localB);
static void cartesian_trajectory_planne_inv(const
  emxArray_real_T_cartesian_tra_T *x, emxArray_real_T_cartesian_tra_T *y,
  B_MATLABSystem_cartesian_traj_T *localB);
static void cartesian_trajectory_plann_diag(const
  emxArray_real_T_cartesian_tra_T *v, emxArray_real_T_cartesian_tra_T *d,
  B_MATLABSystem_cartesian_traj_T *localB);
static boolean_T cartesian_trajectory_planne_any(const
  emxArray_boolean_T_cartesian__T *x);
static void cartesian_trajectory_pl_sqrt_nh(emxArray_real_T_cartesian_tra_T *x);
static boolean_T cartesian_tr_isPositiveDefinite(const real_T B[36],
  B_MATLABSystem_cartesian_traj_T *localB);
static boolean_T DampedBFGSwGradientProjection_k(const
  h_robotics_core_internal_Damp_T *obj, const real_T xNew[6],
  B_MATLABSystem_cartesian_traj_T *localB);
static void DampedBFGSwGradientProjection_s(h_robotics_core_internal_Damp_T *obj,
  real_T xSol[6], c_robotics_core_internal_NLPS_T *exitFlag, real_T *err, real_T
  *iter, B_MATLABSystem_cartesian_traj_T *localB);
static void cartesian_trajectory_p_isfinite(const
  emxArray_real_T_cartesian_tra_T *x, emxArray_boolean_T_cartesian__T *b);
static void cartesian_trajectory_pla_rand_m(real_T varargin_1,
  emxArray_real_T_cartesian_tra_T *r, B_MATLABSystem_cartesian_traj_T *localB,
  DW_MATLABSystem_cartesian_tra_T *localDW);
static real_T cartesian_trajectory_genrandu_n(uint32_T mt[625],
  B_MATLABSystem_cartesian_traj_T *localB);
static real_T cartesia_eml_rand_mt19937ar_aoi(uint32_T state[625],
  B_MATLABSystem_cartesian_traj_T *localB);
static void cartesian_trajectory_plan_randn(const real_T varargin_1[2],
  emxArray_real_T_cartesian_tra_T *r, B_MATLABSystem_cartesian_traj_T *localB,
  DW_MATLABSystem_cartesian_tra_T *localDW);
static void cartes_NLPSolverInterface_solve(h_robotics_core_internal_Damp_T *obj,
  const real_T seed[6], real_T xSol[6], real_T *solutionInfo_Iterations, real_T *
  solutionInfo_RRAttempts, real_T *solutionInfo_Error, real_T
  *solutionInfo_ExitFlag, char_T solutionInfo_Status_data[], int32_T
  solutionInfo_Status_size[2], B_MATLABSystem_cartesian_traj_T *localB,
  DW_MATLABSystem_cartesian_tra_T *localDW);
static void cart_inverseKinematics_stepImpl(b_inverseKinematics_cartesian_T *obj,
  const real_T tform[16], const real_T weights[6], const real_T initialGuess[6],
  real_T QSol[6], real_T *solutionInfo_Iterations, real_T
  *solutionInfo_PoseErrorNorm, real_T *solutionInfo_ExitFlag, char_T
  solutionInfo_Status_data[], int32_T solutionInfo_Status_size[2],
  B_MATLABSystem_cartesian_traj_T *localB, DW_MATLABSystem_cartesian_tra_T
  *localDW);
static void matlabCodegenHandle_matlabCod_o(robotics_slmanip_internal_blo_T *obj);
static void cartesian_tr_SystemCore_release(b_inverseKinematics_cartesian_T *obj);
static void cartesian_tra_SystemCore_delete(b_inverseKinematics_cartesian_T *obj);
static void matlabCodegenHandle_matlabCodeg(b_inverseKinematics_cartesian_T *obj);
static void emxFreeStruct_c_rigidBodyJoint(c_rigidBodyJoint_cartesian_tr_T
  *pStruct);
static void emxFreeStruct_v_robotics_manip_(v_robotics_manip_internal_Rig_T
  *pStruct);
static void emxFreeStruct_y_robotics_manip_(y_robotics_manip_internal_Rig_T
  *pStruct);
static void emxFreeStruct_b_inverseKinemati(b_inverseKinematics_cartesian_T
  *pStruct);
static void emxFreeStruct_robotics_slmanip_(robotics_slmanip_internal_blo_T
  *pStruct);
static void emxFreeStruct_w_robotics_manip_(w_robotics_manip_internal_Rig_T
  *pStruct);
static void emxFreeStruct_x_robotics_manip_(x_robotics_manip_internal_Rig_T
  *pStruct);
static void emxFreeStruct_f_robotics_manip_(f_robotics_manip_internal_IKE_T
  *pStruct);
static void emxFreeStruct_h_robotics_core_i(h_robotics_core_internal_Damp_T
  *pStruct);

// Forward declaration for local functions
static void matlabCodegenHandle_matlabC_jbz(ros_slros_internal_block_GetP_T *obj);
static void cartesian_tr_matlabCodegenHa_ku(ros_slros_internal_block_SetP_T *obj);
static void matlabCodegenHandle_matlabCo_jb(ros_slros_internal_block_Subs_T *obj);
static void matlabCodegenHandle_matlabCod_j(ros_slros_internal_block_Publ_T *obj);
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

static void cartesian_trajec_emxInit_char_T(emxArray_char_T_cartesian_tra_T
  **pEmxArray, int32_T numDimensions, B_MATLABSystem_cartesian_traj_T *localB)
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
  for (localB->i_b = 0; localB->i_b < numDimensions; localB->i_b++) {
    emxArray->size[localB->i_b] = 0;
  }
}

static void cartesian_trajec_emxInit_real_T(emxArray_real_T_cartesian_tra_T
  **pEmxArray, int32_T numDimensions, B_MATLABSystem_cartesian_traj_T *localB)
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
  for (localB->i_m = 0; localB->i_m < numDimensions; localB->i_m++) {
    emxArray->size[localB->i_m] = 0;
  }
}

static void emxInitStruct_c_rigidBodyJoint(c_rigidBodyJoint_cartesian_tr_T
  *pStruct, B_MATLABSystem_cartesian_traj_T *localB)
{
  cartesian_trajec_emxInit_char_T(&pStruct->Type, 2, localB);
  cartesian_trajec_emxInit_real_T(&pStruct->MotionSubspace, 2, localB);
  cartesian_trajec_emxInit_char_T(&pStruct->NameInternal, 2, localB);
  cartesian_trajec_emxInit_real_T(&pStruct->PositionLimitsInternal, 2, localB);
  cartesian_trajec_emxInit_real_T(&pStruct->HomePositionInternal, 1, localB);
}

static void emxInitStruct_v_robotics_manip_(v_robotics_manip_internal_Rig_T
  *pStruct, B_MATLABSystem_cartesian_traj_T *localB)
{
  cartesian_trajec_emxInit_char_T(&pStruct->NameInternal, 2, localB);
  emxInitStruct_c_rigidBodyJoint(&pStruct->JointInternal, localB);
}

static void emxInitStruct_y_robotics_manip_(y_robotics_manip_internal_Rig_T
  *pStruct, B_MATLABSystem_cartesian_traj_T *localB)
{
  emxInitStruct_v_robotics_manip_(&pStruct->Base, localB);
}

static void emxInitStruct_b_inverseKinemati(b_inverseKinematics_cartesian_T
  *pStruct, B_MATLABSystem_cartesian_traj_T *localB)
{
  cartesian_trajec_emxInit_real_T(&pStruct->Limits, 2, localB);
}

static void emxInitStruct_robotics_slmanip_(robotics_slmanip_internal_blo_T
  *pStruct, B_MATLABSystem_cartesian_traj_T *localB)
{
  emxInitStruct_y_robotics_manip_(&pStruct->TreeInternal, localB);
  emxInitStruct_b_inverseKinemati(&pStruct->IKInternal, localB);
}

static void emxInitStruct_w_robotics_manip_(w_robotics_manip_internal_Rig_T
  *pStruct, B_MATLABSystem_cartesian_traj_T *localB)
{
  cartesian_trajec_emxInit_char_T(&pStruct->NameInternal, 2, localB);
}

static void emxInitStruct_x_robotics_manip_(x_robotics_manip_internal_Rig_T
  *pStruct, B_MATLABSystem_cartesian_traj_T *localB)
{
  emxInitStruct_w_robotics_manip_(&pStruct->Base, localB);
}

static void emxInitStruct_f_robotics_manip_(f_robotics_manip_internal_IKE_T
  *pStruct, B_MATLABSystem_cartesian_traj_T *localB)
{
  cartesian_trajec_emxInit_char_T(&pStruct->BodyName, 2, localB);
  cartesian_trajec_emxInit_real_T(&pStruct->ErrTemp, 1, localB);
  cartesian_trajec_emxInit_real_T(&pStruct->GradTemp, 1, localB);
}

static void emxInitStruct_h_robotics_core_i(h_robotics_core_internal_Damp_T
  *pStruct, B_MATLABSystem_cartesian_traj_T *localB)
{
  cartesian_trajec_emxInit_real_T(&pStruct->ConstraintMatrix, 2, localB);
  cartesian_trajec_emxInit_real_T(&pStruct->ConstraintBound, 1, localB);
}

static void cartesia_twister_state_vector_l(uint32_T mt[625])
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
  cartesia_twister_state_vector_l(state);
}

static void cartesian_traject_seed_to_state(uint32_T state[2])
{
  state[0] = 362436069U;
  state[1] = 521288629U;
}

static void cartesian_tra_eml_rand_shr3cong(uint32_T state[2])
{
  cartesian_traject_seed_to_state(state);
}

static void cartes_emxEnsureCapacity_char_T(emxArray_char_T_cartesian_tra_T
  *emxArray, int32_T oldNumel, B_MATLABSystem_cartesian_traj_T *localB)
{
  void *newData;
  if (oldNumel < 0) {
    oldNumel = 0;
  }

  localB->newNumel = 1;
  for (localB->i_fr = 0; localB->i_fr < emxArray->numDimensions; localB->i_fr++)
  {
    localB->newNumel *= emxArray->size[localB->i_fr];
  }

  if (localB->newNumel > emxArray->allocatedSize) {
    localB->i_fr = emxArray->allocatedSize;
    if (localB->i_fr < 16) {
      localB->i_fr = 16;
    }

    while (localB->i_fr < localB->newNumel) {
      if (localB->i_fr > 1073741823) {
        localB->i_fr = MAX_int32_T;
      } else {
        localB->i_fr <<= 1;
      }
    }

    newData = calloc(static_cast<uint32_T>(localB->i_fr), sizeof(char_T));
    if (emxArray->data != NULL) {
      memcpy(newData, emxArray->data, sizeof(char_T) * oldNumel);
      if (emxArray->canFreeData) {
        free(emxArray->data);
      }
    }

    emxArray->data = (char_T *)newData;
    emxArray->allocatedSize = localB->i_fr;
    emxArray->canFreeData = true;
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

static void cartes_emxEnsureCapacity_real_T(emxArray_real_T_cartesian_tra_T
  *emxArray, int32_T oldNumel, B_MATLABSystem_cartesian_traj_T *localB)
{
  void *newData;
  if (oldNumel < 0) {
    oldNumel = 0;
  }

  localB->newNumel_e = 1;
  for (localB->i_nh = 0; localB->i_nh < emxArray->numDimensions; localB->i_nh++)
  {
    localB->newNumel_e *= emxArray->size[localB->i_nh];
  }

  if (localB->newNumel_e > emxArray->allocatedSize) {
    localB->i_nh = emxArray->allocatedSize;
    if (localB->i_nh < 16) {
      localB->i_nh = 16;
    }

    while (localB->i_nh < localB->newNumel_e) {
      if (localB->i_nh > 1073741823) {
        localB->i_nh = MAX_int32_T;
      } else {
        localB->i_nh <<= 1;
      }
    }

    newData = calloc(static_cast<uint32_T>(localB->i_nh), sizeof(real_T));
    if (emxArray->data != NULL) {
      memcpy(newData, emxArray->data, sizeof(real_T) * oldNumel);
      if (emxArray->canFreeData) {
        free(emxArray->data);
      }
    }

    emxArray->data = (real_T *)newData;
    emxArray->allocatedSize = localB->i_nh;
    emxArray->canFreeData = true;
  }
}

static v_robotics_manip_internal_Rig_T *cartesian_t_RigidBody_RigidBody
  (v_robotics_manip_internal_Rig_T *obj, B_MATLABSystem_cartesian_traj_T *localB)
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
  cartes_emxEnsureCapacity_char_T(obj->NameInternal, b_kstr, localB);
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
  cartes_emxEnsureCapacity_char_T(obj->JointInternal.NameInternal, b_kstr,
    localB);
  for (b_kstr = 0; b_kstr < 15; b_kstr++) {
    obj->JointInternal.NameInternal->data[b_kstr] = tmp_4[b_kstr];
  }

  b_kstr = obj->JointInternal.Type->size[0] * obj->JointInternal.Type->size[1];
  obj->JointInternal.Type->size[0] = 1;
  obj->JointInternal.Type->size[1] = 5;
  cartes_emxEnsureCapacity_char_T(obj->JointInternal.Type, b_kstr, localB);
  for (b_kstr = 0; b_kstr < 5; b_kstr++) {
    obj->JointInternal.Type->data[b_kstr] = tmp_5[b_kstr];
  }

  cartesian_trajec_emxInit_char_T(&switch_expression, 2, localB);
  b_kstr = switch_expression->size[0] * switch_expression->size[1];
  switch_expression->size[0] = 1;
  switch_expression->size[1] = obj->JointInternal.Type->size[1];
  cartes_emxEnsureCapacity_char_T(switch_expression, b_kstr, localB);
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
  cartes_emxEnsureCapacity_real_T(obj->JointInternal.MotionSubspace, b_kstr,
    localB);
  for (b_kstr = 0; b_kstr < 6; b_kstr++) {
    obj->JointInternal.MotionSubspace->data[b_kstr] = msubspace_data[b_kstr];
  }

  b_kstr = obj->JointInternal.PositionLimitsInternal->size[0] *
    obj->JointInternal.PositionLimitsInternal->size[1];
  obj->JointInternal.PositionLimitsInternal->size[0] = 1;
  obj->JointInternal.PositionLimitsInternal->size[1] = 2;
  cartes_emxEnsureCapacity_real_T(obj->JointInternal.PositionLimitsInternal,
    b_kstr, localB);
  for (b_kstr = 0; b_kstr < 2; b_kstr++) {
    obj->JointInternal.PositionLimitsInternal->data[b_kstr] = poslim_data[b_kstr];
  }

  b_kstr = obj->JointInternal.HomePositionInternal->size[0];
  obj->JointInternal.HomePositionInternal->size[0] = 1;
  cartes_emxEnsureCapacity_real_T(obj->JointInternal.HomePositionInternal,
    b_kstr, localB);
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
  cartes_emxEnsureCapacity_real_T(obj->JointInternal.MotionSubspace, b_kstr,
    localB);
  for (b_kstr = 0; b_kstr < 6; b_kstr++) {
    obj->JointInternal.MotionSubspace->data[b_kstr] = 0.0;
  }

  obj->JointInternal.InTree = true;
  b_kstr = obj->JointInternal.PositionLimitsInternal->size[0] *
    obj->JointInternal.PositionLimitsInternal->size[1];
  obj->JointInternal.PositionLimitsInternal->size[0] = 1;
  obj->JointInternal.PositionLimitsInternal->size[1] = 2;
  cartes_emxEnsureCapacity_real_T(obj->JointInternal.PositionLimitsInternal,
    b_kstr, localB);
  obj->JointInternal.PositionLimitsInternal->data[0] = 0.0;
  obj->JointInternal.PositionLimitsInternal->data
    [obj->JointInternal.PositionLimitsInternal->size[0]] = 0.0;
  obj->JointInternal.JointAxisInternal[0] = 0.0;
  obj->JointInternal.JointAxisInternal[1] = 0.0;
  obj->JointInternal.JointAxisInternal[2] = 0.0;
  b_kstr = obj->JointInternal.HomePositionInternal->size[0];
  obj->JointInternal.HomePositionInternal->size[0] = 1;
  cartes_emxEnsureCapacity_real_T(obj->JointInternal.HomePositionInternal,
    b_kstr, localB);
  obj->JointInternal.HomePositionInternal->data[0] = 0.0;
  return b_obj;
}

static v_robotics_manip_internal_Rig_T *cartesian_RigidBody_RigidBody_b
  (v_robotics_manip_internal_Rig_T *obj, B_MATLABSystem_cartesian_traj_T *localB)
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
  cartes_emxEnsureCapacity_char_T(obj->NameInternal, b_kstr, localB);
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
  cartes_emxEnsureCapacity_char_T(obj->JointInternal.NameInternal, b_kstr,
    localB);
  for (b_kstr = 0; b_kstr < 11; b_kstr++) {
    obj->JointInternal.NameInternal->data[b_kstr] = tmp_4[b_kstr];
  }

  b_kstr = obj->JointInternal.Type->size[0] * obj->JointInternal.Type->size[1];
  obj->JointInternal.Type->size[0] = 1;
  obj->JointInternal.Type->size[1] = 8;
  cartes_emxEnsureCapacity_char_T(obj->JointInternal.Type, b_kstr, localB);
  for (b_kstr = 0; b_kstr < 8; b_kstr++) {
    obj->JointInternal.Type->data[b_kstr] = tmp_5[b_kstr];
  }

  cartesian_trajec_emxInit_char_T(&switch_expression, 2, localB);
  b_kstr = switch_expression->size[0] * switch_expression->size[1];
  switch_expression->size[0] = 1;
  switch_expression->size[1] = obj->JointInternal.Type->size[1];
  cartes_emxEnsureCapacity_char_T(switch_expression, b_kstr, localB);
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
  cartes_emxEnsureCapacity_real_T(obj->JointInternal.MotionSubspace, b_kstr,
    localB);
  for (b_kstr = 0; b_kstr < 6; b_kstr++) {
    obj->JointInternal.MotionSubspace->data[b_kstr] = msubspace_data[b_kstr];
  }

  b_kstr = obj->JointInternal.PositionLimitsInternal->size[0] *
    obj->JointInternal.PositionLimitsInternal->size[1];
  obj->JointInternal.PositionLimitsInternal->size[0] = 1;
  obj->JointInternal.PositionLimitsInternal->size[1] = 2;
  cartes_emxEnsureCapacity_real_T(obj->JointInternal.PositionLimitsInternal,
    b_kstr, localB);
  for (b_kstr = 0; b_kstr < 2; b_kstr++) {
    obj->JointInternal.PositionLimitsInternal->data[b_kstr] = poslim_data[b_kstr];
  }

  b_kstr = obj->JointInternal.HomePositionInternal->size[0];
  obj->JointInternal.HomePositionInternal->size[0] = 1;
  cartes_emxEnsureCapacity_real_T(obj->JointInternal.HomePositionInternal,
    b_kstr, localB);
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
  cartes_emxEnsureCapacity_real_T(obj->JointInternal.MotionSubspace, b_kstr,
    localB);
  for (b_kstr = 0; b_kstr < 6; b_kstr++) {
    obj->JointInternal.MotionSubspace->data[b_kstr] = tmp_9[b_kstr];
  }

  obj->JointInternal.InTree = true;
  b_kstr = obj->JointInternal.PositionLimitsInternal->size[0] *
    obj->JointInternal.PositionLimitsInternal->size[1];
  obj->JointInternal.PositionLimitsInternal->size[0] = 1;
  obj->JointInternal.PositionLimitsInternal->size[1] = 2;
  cartes_emxEnsureCapacity_real_T(obj->JointInternal.PositionLimitsInternal,
    b_kstr, localB);
  obj->JointInternal.PositionLimitsInternal->data[0] = -3.10668606855;
  obj->JointInternal.PositionLimitsInternal->data
    [obj->JointInternal.PositionLimitsInternal->size[0]] = 3.10668606855;
  obj->JointInternal.JointAxisInternal[0] = 0.0;
  obj->JointInternal.JointAxisInternal[1] = 0.0;
  obj->JointInternal.JointAxisInternal[2] = 1.0;
  b_kstr = obj->JointInternal.HomePositionInternal->size[0];
  obj->JointInternal.HomePositionInternal->size[0] = 1;
  cartes_emxEnsureCapacity_real_T(obj->JointInternal.HomePositionInternal,
    b_kstr, localB);
  obj->JointInternal.HomePositionInternal->data[0] = 0.0;
  return b_obj;
}

static v_robotics_manip_internal_Rig_T *cartesia_RigidBody_RigidBody_by
  (v_robotics_manip_internal_Rig_T *obj, B_MATLABSystem_cartesian_traj_T *localB)
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
  cartes_emxEnsureCapacity_char_T(obj->NameInternal, b_kstr, localB);
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
  cartes_emxEnsureCapacity_char_T(obj->JointInternal.NameInternal, b_kstr,
    localB);
  for (b_kstr = 0; b_kstr < 11; b_kstr++) {
    obj->JointInternal.NameInternal->data[b_kstr] = tmp_4[b_kstr];
  }

  b_kstr = obj->JointInternal.Type->size[0] * obj->JointInternal.Type->size[1];
  obj->JointInternal.Type->size[0] = 1;
  obj->JointInternal.Type->size[1] = 8;
  cartes_emxEnsureCapacity_char_T(obj->JointInternal.Type, b_kstr, localB);
  for (b_kstr = 0; b_kstr < 8; b_kstr++) {
    obj->JointInternal.Type->data[b_kstr] = tmp_5[b_kstr];
  }

  cartesian_trajec_emxInit_char_T(&switch_expression, 2, localB);
  b_kstr = switch_expression->size[0] * switch_expression->size[1];
  switch_expression->size[0] = 1;
  switch_expression->size[1] = obj->JointInternal.Type->size[1];
  cartes_emxEnsureCapacity_char_T(switch_expression, b_kstr, localB);
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
  cartes_emxEnsureCapacity_real_T(obj->JointInternal.MotionSubspace, b_kstr,
    localB);
  for (b_kstr = 0; b_kstr < 6; b_kstr++) {
    obj->JointInternal.MotionSubspace->data[b_kstr] = msubspace_data[b_kstr];
  }

  b_kstr = obj->JointInternal.PositionLimitsInternal->size[0] *
    obj->JointInternal.PositionLimitsInternal->size[1];
  obj->JointInternal.PositionLimitsInternal->size[0] = 1;
  obj->JointInternal.PositionLimitsInternal->size[1] = 2;
  cartes_emxEnsureCapacity_real_T(obj->JointInternal.PositionLimitsInternal,
    b_kstr, localB);
  for (b_kstr = 0; b_kstr < 2; b_kstr++) {
    obj->JointInternal.PositionLimitsInternal->data[b_kstr] = poslim_data[b_kstr];
  }

  b_kstr = obj->JointInternal.HomePositionInternal->size[0];
  obj->JointInternal.HomePositionInternal->size[0] = 1;
  cartes_emxEnsureCapacity_real_T(obj->JointInternal.HomePositionInternal,
    b_kstr, localB);
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
  cartes_emxEnsureCapacity_real_T(obj->JointInternal.MotionSubspace, b_kstr,
    localB);
  for (b_kstr = 0; b_kstr < 6; b_kstr++) {
    obj->JointInternal.MotionSubspace->data[b_kstr] = tmp_9[b_kstr];
  }

  obj->JointInternal.InTree = true;
  b_kstr = obj->JointInternal.PositionLimitsInternal->size[0] *
    obj->JointInternal.PositionLimitsInternal->size[1];
  obj->JointInternal.PositionLimitsInternal->size[0] = 1;
  obj->JointInternal.PositionLimitsInternal->size[1] = 2;
  cartes_emxEnsureCapacity_real_T(obj->JointInternal.PositionLimitsInternal,
    b_kstr, localB);
  obj->JointInternal.PositionLimitsInternal->data[0] = -1.71042266695;
  obj->JointInternal.PositionLimitsInternal->data
    [obj->JointInternal.PositionLimitsInternal->size[0]] = 1.71042266695;
  obj->JointInternal.JointAxisInternal[0] = 0.0;
  obj->JointInternal.JointAxisInternal[1] = 0.0;
  obj->JointInternal.JointAxisInternal[2] = -1.0;
  b_kstr = obj->JointInternal.HomePositionInternal->size[0];
  obj->JointInternal.HomePositionInternal->size[0] = 1;
  cartes_emxEnsureCapacity_real_T(obj->JointInternal.HomePositionInternal,
    b_kstr, localB);
  obj->JointInternal.HomePositionInternal->data[0] = 0.0;
  return b_obj;
}

static v_robotics_manip_internal_Rig_T *cartesi_RigidBody_RigidBody_bya
  (v_robotics_manip_internal_Rig_T *obj, B_MATLABSystem_cartesian_traj_T *localB)
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
  cartes_emxEnsureCapacity_char_T(obj->NameInternal, b_kstr, localB);
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
  cartes_emxEnsureCapacity_char_T(obj->JointInternal.NameInternal, b_kstr,
    localB);
  for (b_kstr = 0; b_kstr < 11; b_kstr++) {
    obj->JointInternal.NameInternal->data[b_kstr] = tmp_4[b_kstr];
  }

  b_kstr = obj->JointInternal.Type->size[0] * obj->JointInternal.Type->size[1];
  obj->JointInternal.Type->size[0] = 1;
  obj->JointInternal.Type->size[1] = 8;
  cartes_emxEnsureCapacity_char_T(obj->JointInternal.Type, b_kstr, localB);
  for (b_kstr = 0; b_kstr < 8; b_kstr++) {
    obj->JointInternal.Type->data[b_kstr] = tmp_5[b_kstr];
  }

  cartesian_trajec_emxInit_char_T(&switch_expression, 2, localB);
  b_kstr = switch_expression->size[0] * switch_expression->size[1];
  switch_expression->size[0] = 1;
  switch_expression->size[1] = obj->JointInternal.Type->size[1];
  cartes_emxEnsureCapacity_char_T(switch_expression, b_kstr, localB);
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
  cartes_emxEnsureCapacity_real_T(obj->JointInternal.MotionSubspace, b_kstr,
    localB);
  for (b_kstr = 0; b_kstr < 6; b_kstr++) {
    obj->JointInternal.MotionSubspace->data[b_kstr] = msubspace_data[b_kstr];
  }

  b_kstr = obj->JointInternal.PositionLimitsInternal->size[0] *
    obj->JointInternal.PositionLimitsInternal->size[1];
  obj->JointInternal.PositionLimitsInternal->size[0] = 1;
  obj->JointInternal.PositionLimitsInternal->size[1] = 2;
  cartes_emxEnsureCapacity_real_T(obj->JointInternal.PositionLimitsInternal,
    b_kstr, localB);
  for (b_kstr = 0; b_kstr < 2; b_kstr++) {
    obj->JointInternal.PositionLimitsInternal->data[b_kstr] = poslim_data[b_kstr];
  }

  b_kstr = obj->JointInternal.HomePositionInternal->size[0];
  obj->JointInternal.HomePositionInternal->size[0] = 1;
  cartes_emxEnsureCapacity_real_T(obj->JointInternal.HomePositionInternal,
    b_kstr, localB);
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
  cartes_emxEnsureCapacity_real_T(obj->JointInternal.MotionSubspace, b_kstr,
    localB);
  for (b_kstr = 0; b_kstr < 6; b_kstr++) {
    obj->JointInternal.MotionSubspace->data[b_kstr] = tmp_9[b_kstr];
  }

  obj->JointInternal.InTree = true;
  b_kstr = obj->JointInternal.PositionLimitsInternal->size[0] *
    obj->JointInternal.PositionLimitsInternal->size[1];
  obj->JointInternal.PositionLimitsInternal->size[0] = 1;
  obj->JointInternal.PositionLimitsInternal->size[1] = 2;
  cartes_emxEnsureCapacity_real_T(obj->JointInternal.PositionLimitsInternal,
    b_kstr, localB);
  obj->JointInternal.PositionLimitsInternal->data[0] = -1.71042266695;
  obj->JointInternal.PositionLimitsInternal->data
    [obj->JointInternal.PositionLimitsInternal->size[0]] = 1.71042266695;
  obj->JointInternal.JointAxisInternal[0] = 0.0;
  obj->JointInternal.JointAxisInternal[1] = 0.0;
  obj->JointInternal.JointAxisInternal[2] = 1.0;
  b_kstr = obj->JointInternal.HomePositionInternal->size[0];
  obj->JointInternal.HomePositionInternal->size[0] = 1;
  cartes_emxEnsureCapacity_real_T(obj->JointInternal.HomePositionInternal,
    b_kstr, localB);
  obj->JointInternal.HomePositionInternal->data[0] = 0.0;
  return b_obj;
}

static v_robotics_manip_internal_Rig_T *cartes_RigidBody_RigidBody_byap
  (v_robotics_manip_internal_Rig_T *obj, B_MATLABSystem_cartesian_traj_T *localB)
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
  cartes_emxEnsureCapacity_char_T(obj->NameInternal, b_kstr, localB);
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
  cartes_emxEnsureCapacity_char_T(obj->JointInternal.NameInternal, b_kstr,
    localB);
  for (b_kstr = 0; b_kstr < 11; b_kstr++) {
    obj->JointInternal.NameInternal->data[b_kstr] = tmp_4[b_kstr];
  }

  b_kstr = obj->JointInternal.Type->size[0] * obj->JointInternal.Type->size[1];
  obj->JointInternal.Type->size[0] = 1;
  obj->JointInternal.Type->size[1] = 8;
  cartes_emxEnsureCapacity_char_T(obj->JointInternal.Type, b_kstr, localB);
  for (b_kstr = 0; b_kstr < 8; b_kstr++) {
    obj->JointInternal.Type->data[b_kstr] = tmp_5[b_kstr];
  }

  cartesian_trajec_emxInit_char_T(&switch_expression, 2, localB);
  b_kstr = switch_expression->size[0] * switch_expression->size[1];
  switch_expression->size[0] = 1;
  switch_expression->size[1] = obj->JointInternal.Type->size[1];
  cartes_emxEnsureCapacity_char_T(switch_expression, b_kstr, localB);
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
  cartes_emxEnsureCapacity_real_T(obj->JointInternal.MotionSubspace, b_kstr,
    localB);
  for (b_kstr = 0; b_kstr < 6; b_kstr++) {
    obj->JointInternal.MotionSubspace->data[b_kstr] = msubspace_data[b_kstr];
  }

  b_kstr = obj->JointInternal.PositionLimitsInternal->size[0] *
    obj->JointInternal.PositionLimitsInternal->size[1];
  obj->JointInternal.PositionLimitsInternal->size[0] = 1;
  obj->JointInternal.PositionLimitsInternal->size[1] = 2;
  cartes_emxEnsureCapacity_real_T(obj->JointInternal.PositionLimitsInternal,
    b_kstr, localB);
  for (b_kstr = 0; b_kstr < 2; b_kstr++) {
    obj->JointInternal.PositionLimitsInternal->data[b_kstr] = poslim_data[b_kstr];
  }

  b_kstr = obj->JointInternal.HomePositionInternal->size[0];
  obj->JointInternal.HomePositionInternal->size[0] = 1;
  cartes_emxEnsureCapacity_real_T(obj->JointInternal.HomePositionInternal,
    b_kstr, localB);
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
  cartes_emxEnsureCapacity_real_T(obj->JointInternal.MotionSubspace, b_kstr,
    localB);
  for (b_kstr = 0; b_kstr < 6; b_kstr++) {
    obj->JointInternal.MotionSubspace->data[b_kstr] = tmp_9[b_kstr];
  }

  obj->JointInternal.InTree = true;
  b_kstr = obj->JointInternal.PositionLimitsInternal->size[0] *
    obj->JointInternal.PositionLimitsInternal->size[1];
  obj->JointInternal.PositionLimitsInternal->size[0] = 1;
  obj->JointInternal.PositionLimitsInternal->size[1] = 2;
  cartes_emxEnsureCapacity_real_T(obj->JointInternal.PositionLimitsInternal,
    b_kstr, localB);
  obj->JointInternal.PositionLimitsInternal->data[0] = -3.10668606855;
  obj->JointInternal.PositionLimitsInternal->data
    [obj->JointInternal.PositionLimitsInternal->size[0]] = 3.10668606855;
  obj->JointInternal.JointAxisInternal[0] = 0.0;
  obj->JointInternal.JointAxisInternal[1] = 0.0;
  obj->JointInternal.JointAxisInternal[2] = 1.0;
  b_kstr = obj->JointInternal.HomePositionInternal->size[0];
  obj->JointInternal.HomePositionInternal->size[0] = 1;
  cartes_emxEnsureCapacity_real_T(obj->JointInternal.HomePositionInternal,
    b_kstr, localB);
  obj->JointInternal.HomePositionInternal->data[0] = 0.0;
  return b_obj;
}

static v_robotics_manip_internal_Rig_T *carte_RigidBody_RigidBody_byapf
  (v_robotics_manip_internal_Rig_T *obj, B_MATLABSystem_cartesian_traj_T *localB)
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
  cartes_emxEnsureCapacity_char_T(obj->NameInternal, b_kstr, localB);
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
  cartes_emxEnsureCapacity_char_T(obj->JointInternal.NameInternal, b_kstr,
    localB);
  for (b_kstr = 0; b_kstr < 11; b_kstr++) {
    obj->JointInternal.NameInternal->data[b_kstr] = tmp_4[b_kstr];
  }

  b_kstr = obj->JointInternal.Type->size[0] * obj->JointInternal.Type->size[1];
  obj->JointInternal.Type->size[0] = 1;
  obj->JointInternal.Type->size[1] = 8;
  cartes_emxEnsureCapacity_char_T(obj->JointInternal.Type, b_kstr, localB);
  for (b_kstr = 0; b_kstr < 8; b_kstr++) {
    obj->JointInternal.Type->data[b_kstr] = tmp_5[b_kstr];
  }

  cartesian_trajec_emxInit_char_T(&switch_expression, 2, localB);
  b_kstr = switch_expression->size[0] * switch_expression->size[1];
  switch_expression->size[0] = 1;
  switch_expression->size[1] = obj->JointInternal.Type->size[1];
  cartes_emxEnsureCapacity_char_T(switch_expression, b_kstr, localB);
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
  cartes_emxEnsureCapacity_real_T(obj->JointInternal.MotionSubspace, b_kstr,
    localB);
  for (b_kstr = 0; b_kstr < 6; b_kstr++) {
    obj->JointInternal.MotionSubspace->data[b_kstr] = msubspace_data[b_kstr];
  }

  b_kstr = obj->JointInternal.PositionLimitsInternal->size[0] *
    obj->JointInternal.PositionLimitsInternal->size[1];
  obj->JointInternal.PositionLimitsInternal->size[0] = 1;
  obj->JointInternal.PositionLimitsInternal->size[1] = 2;
  cartes_emxEnsureCapacity_real_T(obj->JointInternal.PositionLimitsInternal,
    b_kstr, localB);
  for (b_kstr = 0; b_kstr < 2; b_kstr++) {
    obj->JointInternal.PositionLimitsInternal->data[b_kstr] = poslim_data[b_kstr];
  }

  b_kstr = obj->JointInternal.HomePositionInternal->size[0];
  obj->JointInternal.HomePositionInternal->size[0] = 1;
  cartes_emxEnsureCapacity_real_T(obj->JointInternal.HomePositionInternal,
    b_kstr, localB);
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
  cartes_emxEnsureCapacity_real_T(obj->JointInternal.MotionSubspace, b_kstr,
    localB);
  for (b_kstr = 0; b_kstr < 6; b_kstr++) {
    obj->JointInternal.MotionSubspace->data[b_kstr] = tmp_9[b_kstr];
  }

  obj->JointInternal.InTree = true;
  b_kstr = obj->JointInternal.PositionLimitsInternal->size[0] *
    obj->JointInternal.PositionLimitsInternal->size[1];
  obj->JointInternal.PositionLimitsInternal->size[0] = 1;
  obj->JointInternal.PositionLimitsInternal->size[1] = 2;
  cartes_emxEnsureCapacity_real_T(obj->JointInternal.PositionLimitsInternal,
    b_kstr, localB);
  obj->JointInternal.PositionLimitsInternal->data[0] = -1.79768912955;
  obj->JointInternal.PositionLimitsInternal->data
    [obj->JointInternal.PositionLimitsInternal->size[0]] = 1.79768912955;
  obj->JointInternal.JointAxisInternal[0] = 0.0;
  obj->JointInternal.JointAxisInternal[1] = 1.0;
  obj->JointInternal.JointAxisInternal[2] = 0.0;
  b_kstr = obj->JointInternal.HomePositionInternal->size[0];
  obj->JointInternal.HomePositionInternal->size[0] = 1;
  cartes_emxEnsureCapacity_real_T(obj->JointInternal.HomePositionInternal,
    b_kstr, localB);
  obj->JointInternal.HomePositionInternal->data[0] = 0.0;
  return b_obj;
}

static v_robotics_manip_internal_Rig_T *cart_RigidBody_RigidBody_byapfa
  (v_robotics_manip_internal_Rig_T *obj, B_MATLABSystem_cartesian_traj_T *localB)
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
  cartes_emxEnsureCapacity_char_T(obj->NameInternal, b_kstr, localB);
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
  cartes_emxEnsureCapacity_char_T(obj->JointInternal.NameInternal, b_kstr,
    localB);
  for (b_kstr = 0; b_kstr < 11; b_kstr++) {
    obj->JointInternal.NameInternal->data[b_kstr] = tmp_4[b_kstr];
  }

  b_kstr = obj->JointInternal.Type->size[0] * obj->JointInternal.Type->size[1];
  obj->JointInternal.Type->size[0] = 1;
  obj->JointInternal.Type->size[1] = 8;
  cartes_emxEnsureCapacity_char_T(obj->JointInternal.Type, b_kstr, localB);
  for (b_kstr = 0; b_kstr < 8; b_kstr++) {
    obj->JointInternal.Type->data[b_kstr] = tmp_5[b_kstr];
  }

  cartesian_trajec_emxInit_char_T(&switch_expression, 2, localB);
  b_kstr = switch_expression->size[0] * switch_expression->size[1];
  switch_expression->size[0] = 1;
  switch_expression->size[1] = obj->JointInternal.Type->size[1];
  cartes_emxEnsureCapacity_char_T(switch_expression, b_kstr, localB);
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
  cartes_emxEnsureCapacity_real_T(obj->JointInternal.MotionSubspace, b_kstr,
    localB);
  for (b_kstr = 0; b_kstr < 6; b_kstr++) {
    obj->JointInternal.MotionSubspace->data[b_kstr] = msubspace_data[b_kstr];
  }

  b_kstr = obj->JointInternal.PositionLimitsInternal->size[0] *
    obj->JointInternal.PositionLimitsInternal->size[1];
  obj->JointInternal.PositionLimitsInternal->size[0] = 1;
  obj->JointInternal.PositionLimitsInternal->size[1] = 2;
  cartes_emxEnsureCapacity_real_T(obj->JointInternal.PositionLimitsInternal,
    b_kstr, localB);
  for (b_kstr = 0; b_kstr < 2; b_kstr++) {
    obj->JointInternal.PositionLimitsInternal->data[b_kstr] = poslim_data[b_kstr];
  }

  b_kstr = obj->JointInternal.HomePositionInternal->size[0];
  obj->JointInternal.HomePositionInternal->size[0] = 1;
  cartes_emxEnsureCapacity_real_T(obj->JointInternal.HomePositionInternal,
    b_kstr, localB);
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
  cartes_emxEnsureCapacity_real_T(obj->JointInternal.MotionSubspace, b_kstr,
    localB);
  for (b_kstr = 0; b_kstr < 6; b_kstr++) {
    obj->JointInternal.MotionSubspace->data[b_kstr] = tmp_9[b_kstr];
  }

  obj->JointInternal.InTree = true;
  b_kstr = obj->JointInternal.PositionLimitsInternal->size[0] *
    obj->JointInternal.PositionLimitsInternal->size[1];
  obj->JointInternal.PositionLimitsInternal->size[0] = 1;
  obj->JointInternal.PositionLimitsInternal->size[1] = 2;
  cartes_emxEnsureCapacity_real_T(obj->JointInternal.PositionLimitsInternal,
    b_kstr, localB);
  obj->JointInternal.PositionLimitsInternal->data[0] = -4.71238898038;
  obj->JointInternal.PositionLimitsInternal->data
    [obj->JointInternal.PositionLimitsInternal->size[0]] = 4.71238898038;
  obj->JointInternal.JointAxisInternal[0] = 0.0;
  obj->JointInternal.JointAxisInternal[1] = 0.0;
  obj->JointInternal.JointAxisInternal[2] = 1.0;
  b_kstr = obj->JointInternal.HomePositionInternal->size[0];
  obj->JointInternal.HomePositionInternal->size[0] = 1;
  cartes_emxEnsureCapacity_real_T(obj->JointInternal.HomePositionInternal,
    b_kstr, localB);
  obj->JointInternal.HomePositionInternal->data[0] = 0.0;
  return b_obj;
}

static y_robotics_manip_internal_Rig_T *car_RigidBodyTree_RigidBodyTree
  (y_robotics_manip_internal_Rig_T *obj, v_robotics_manip_internal_Rig_T *iobj_0,
   v_robotics_manip_internal_Rig_T *iobj_1, v_robotics_manip_internal_Rig_T
   *iobj_2, v_robotics_manip_internal_Rig_T *iobj_3,
   v_robotics_manip_internal_Rig_T *iobj_4, v_robotics_manip_internal_Rig_T
   *iobj_5, v_robotics_manip_internal_Rig_T *iobj_6,
   v_robotics_manip_internal_Rig_T *iobj_7, B_MATLABSystem_cartesian_traj_T
   *localB)
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
  obj->Bodies[0] = cartesian_t_RigidBody_RigidBody(iobj_7, localB);
  obj->Bodies[0]->Index = 1.0;
  obj->Bodies[1] = cartesian_RigidBody_RigidBody_b(iobj_0, localB);
  obj->Bodies[1]->Index = 2.0;
  obj->Bodies[2] = cartesia_RigidBody_RigidBody_by(iobj_1, localB);
  obj->Bodies[2]->Index = 3.0;
  obj->Bodies[3] = cartesi_RigidBody_RigidBody_bya(iobj_2, localB);
  obj->Bodies[3]->Index = 4.0;
  obj->Bodies[4] = cartes_RigidBody_RigidBody_byap(iobj_3, localB);
  obj->Bodies[4]->Index = 5.0;
  obj->Bodies[5] = carte_RigidBody_RigidBody_byapf(iobj_4, localB);
  obj->Bodies[5]->Index = 6.0;
  obj->Bodies[6] = cart_RigidBody_RigidBody_byapfa(iobj_5, localB);
  obj->Bodies[6]->Index = 7.0;
  b_kstr = iobj_6->NameInternal->size[0] * iobj_6->NameInternal->size[1];
  iobj_6->NameInternal->size[0] = 1;
  iobj_6->NameInternal->size[1] = 11;
  cartes_emxEnsureCapacity_char_T(iobj_6->NameInternal, b_kstr, localB);
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
  cartes_emxEnsureCapacity_char_T(iobj_6->JointInternal.NameInternal, b_kstr,
    localB);
  for (b_kstr = 0; b_kstr < 12; b_kstr++) {
    iobj_6->JointInternal.NameInternal->data[b_kstr] = tmp_3[b_kstr];
  }

  b_kstr = iobj_6->JointInternal.Type->size[0] * iobj_6->
    JointInternal.Type->size[1];
  iobj_6->JointInternal.Type->size[0] = 1;
  iobj_6->JointInternal.Type->size[1] = 5;
  cartes_emxEnsureCapacity_char_T(iobj_6->JointInternal.Type, b_kstr, localB);
  for (b_kstr = 0; b_kstr < 5; b_kstr++) {
    iobj_6->JointInternal.Type->data[b_kstr] = tmp_4[b_kstr];
  }

  cartesian_trajec_emxInit_char_T(&switch_expression, 2, localB);
  b_kstr = switch_expression->size[0] * switch_expression->size[1];
  switch_expression->size[0] = 1;
  switch_expression->size[1] = iobj_6->JointInternal.Type->size[1];
  cartes_emxEnsureCapacity_char_T(switch_expression, b_kstr, localB);
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
  cartes_emxEnsureCapacity_real_T(iobj_6->JointInternal.MotionSubspace, b_kstr,
    localB);
  for (b_kstr = 0; b_kstr < 6; b_kstr++) {
    iobj_6->JointInternal.MotionSubspace->data[b_kstr] = msubspace_data[b_kstr];
  }

  b_kstr = iobj_6->JointInternal.PositionLimitsInternal->size[0] *
    iobj_6->JointInternal.PositionLimitsInternal->size[1];
  iobj_6->JointInternal.PositionLimitsInternal->size[0] = 1;
  iobj_6->JointInternal.PositionLimitsInternal->size[1] = 2;
  cartes_emxEnsureCapacity_real_T(iobj_6->JointInternal.PositionLimitsInternal,
    b_kstr, localB);
  for (b_kstr = 0; b_kstr < 2; b_kstr++) {
    iobj_6->JointInternal.PositionLimitsInternal->data[b_kstr] =
      poslim_data[b_kstr];
  }

  b_kstr = iobj_6->JointInternal.HomePositionInternal->size[0];
  iobj_6->JointInternal.HomePositionInternal->size[0] = 1;
  cartes_emxEnsureCapacity_real_T(iobj_6->JointInternal.HomePositionInternal,
    b_kstr, localB);
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
  cartes_emxEnsureCapacity_real_T(iobj_6->JointInternal.MotionSubspace, b_kstr,
    localB);
  for (b_kstr = 0; b_kstr < 6; b_kstr++) {
    iobj_6->JointInternal.MotionSubspace->data[b_kstr] = 0.0;
  }

  iobj_6->JointInternal.InTree = true;
  b_kstr = iobj_6->JointInternal.PositionLimitsInternal->size[0] *
    iobj_6->JointInternal.PositionLimitsInternal->size[1];
  iobj_6->JointInternal.PositionLimitsInternal->size[0] = 1;
  iobj_6->JointInternal.PositionLimitsInternal->size[1] = 2;
  cartes_emxEnsureCapacity_real_T(iobj_6->JointInternal.PositionLimitsInternal,
    b_kstr, localB);
  iobj_6->JointInternal.PositionLimitsInternal->data[0] = 0.0;
  iobj_6->JointInternal.PositionLimitsInternal->data
    [iobj_6->JointInternal.PositionLimitsInternal->size[0]] = 0.0;
  iobj_6->JointInternal.JointAxisInternal[0] = 0.0;
  iobj_6->JointInternal.JointAxisInternal[1] = 0.0;
  iobj_6->JointInternal.JointAxisInternal[2] = 0.0;
  b_kstr = iobj_6->JointInternal.HomePositionInternal->size[0];
  iobj_6->JointInternal.HomePositionInternal->size[0] = 1;
  cartes_emxEnsureCapacity_real_T(iobj_6->JointInternal.HomePositionInternal,
    b_kstr, localB);
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
  cartes_emxEnsureCapacity_char_T(obj->Base.NameInternal, b_kstr, localB);
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
  cartes_emxEnsureCapacity_char_T(obj->Base.JointInternal.NameInternal, b_kstr,
    localB);
  for (b_kstr = 0; b_kstr < 9; b_kstr++) {
    obj_0->JointInternal.NameInternal->data[b_kstr] = tmp_a[b_kstr];
  }

  b_kstr = obj->Base.JointInternal.Type->size[0] * obj->
    Base.JointInternal.Type->size[1];
  obj->Base.JointInternal.Type->size[0] = 1;
  obj->Base.JointInternal.Type->size[1] = 5;
  cartes_emxEnsureCapacity_char_T(obj->Base.JointInternal.Type, b_kstr, localB);
  for (b_kstr = 0; b_kstr < 5; b_kstr++) {
    obj_0->JointInternal.Type->data[b_kstr] = tmp_4[b_kstr];
  }

  b_kstr = switch_expression->size[0] * switch_expression->size[1];
  switch_expression->size[0] = 1;
  switch_expression->size[1] = obj->Base.JointInternal.Type->size[1];
  cartes_emxEnsureCapacity_char_T(switch_expression, b_kstr, localB);
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
  cartes_emxEnsureCapacity_real_T(obj->Base.JointInternal.MotionSubspace, b_kstr,
    localB);
  for (b_kstr = 0; b_kstr < 6; b_kstr++) {
    obj_0->JointInternal.MotionSubspace->data[b_kstr] = msubspace_data[b_kstr];
  }

  b_kstr = obj->Base.JointInternal.PositionLimitsInternal->size[0] *
    obj->Base.JointInternal.PositionLimitsInternal->size[1];
  obj->Base.JointInternal.PositionLimitsInternal->size[0] = 1;
  obj->Base.JointInternal.PositionLimitsInternal->size[1] = 2;
  cartes_emxEnsureCapacity_real_T(obj->Base.JointInternal.PositionLimitsInternal,
    b_kstr, localB);
  for (b_kstr = 0; b_kstr < 2; b_kstr++) {
    obj_0->JointInternal.PositionLimitsInternal->data[b_kstr] =
      poslim_data[b_kstr];
  }

  b_kstr = obj->Base.JointInternal.HomePositionInternal->size[0];
  obj->Base.JointInternal.HomePositionInternal->size[0] = 1;
  cartes_emxEnsureCapacity_real_T(obj->Base.JointInternal.HomePositionInternal,
    b_kstr, localB);
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

static void cartesi_genrand_uint32_vector_d(uint32_T mt[625], uint32_T u[2],
  B_MATLABSystem_cartesian_traj_T *localB)
{
  for (localB->b_j_i = 0; localB->b_j_i < 2; localB->b_j_i++) {
    localB->mti = mt[624] + 1U;
    if (localB->mti >= 625U) {
      for (localB->b_kk = 0; localB->b_kk < 227; localB->b_kk++) {
        localB->y_d = (mt[localB->b_kk + 1] & 2147483647U) | (mt[localB->b_kk] &
          2147483648U);
        if ((localB->y_d & 1U) == 0U) {
          localB->y_d >>= 1U;
        } else {
          localB->y_d = localB->y_d >> 1U ^ 2567483615U;
        }

        mt[localB->b_kk] = mt[localB->b_kk + 397] ^ localB->y_d;
      }

      for (localB->b_kk = 0; localB->b_kk < 396; localB->b_kk++) {
        localB->y_d = (mt[localB->b_kk + 227] & 2147483648U) | (mt[localB->b_kk
          + 228] & 2147483647U);
        if ((localB->y_d & 1U) == 0U) {
          localB->y_d >>= 1U;
        } else {
          localB->y_d = localB->y_d >> 1U ^ 2567483615U;
        }

        mt[localB->b_kk + 227] = mt[localB->b_kk] ^ localB->y_d;
      }

      localB->y_d = (mt[623] & 2147483648U) | (mt[0] & 2147483647U);
      if ((localB->y_d & 1U) == 0U) {
        localB->y_d >>= 1U;
      } else {
        localB->y_d = localB->y_d >> 1U ^ 2567483615U;
      }

      mt[623] = mt[396] ^ localB->y_d;
      localB->mti = 1U;
    }

    localB->y_d = mt[static_cast<int32_T>(localB->mti) - 1];
    mt[624] = localB->mti;
    localB->y_d ^= localB->y_d >> 11U;
    localB->y_d ^= localB->y_d << 7U & 2636928640U;
    localB->y_d ^= localB->y_d << 15U & 4022730752U;
    u[localB->b_j_i] = localB->y_d >> 18U ^ localB->y_d;
  }
}

static boolean_T cartesian_trajec_is_valid_state(const uint32_T mt[625],
  B_MATLABSystem_cartesian_traj_T *localB)
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
    localB->k_f = 0;
    exitg1 = false;
    while ((!exitg1) && (localB->k_f + 1 < 625)) {
      if (mt[localB->k_f] == 0U) {
        localB->k_f++;
      } else {
        isvalid = true;
        exitg1 = true;
      }
    }
  }

  return isvalid;
}

static void cartesian__eml_rand_mt19937ar_a(const uint32_T state[625], uint32_T
  b_state[625], real_T *r, B_MATLABSystem_cartesian_traj_T *localB)
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
    cartesi_genrand_uint32_vector_d(b_state, localB->b_u, localB);
    *r = (static_cast<real_T>(localB->b_u[0] >> 5U) * 6.7108864E+7 +
          static_cast<real_T>(localB->b_u[1] >> 6U)) * 1.1102230246251565E-16;
    if (*r == 0.0) {
      if (!cartesian_trajec_is_valid_state(b_state, localB)) {
        localB->r = 5489U;
        b_state[0] = 5489U;
        for (localB->b_mti = 0; localB->b_mti < 623; localB->b_mti++) {
          localB->r = ((localB->r >> 30U ^ localB->r) * 1812433253U +
                       localB->b_mti) + 1U;
          b_state[localB->b_mti + 1] = localB->r;
        }

        b_state[624] = 624U;
      }
    } else {
      exitg1 = 1;
    }
  } while (exitg1 == 0);
}

static void cartesian_trajectory_plann_rand(real_T r[5],
  B_MATLABSystem_cartesian_traj_T *localB, DW_MATLABSystem_cartesian_tra_T
  *localDW)
{
  for (localB->b_k_h = 0; localB->b_k_h < 5; localB->b_k_h++) {
    memcpy(&localB->uv[0], &localDW->state_e[0], 625U * sizeof(uint32_T));
    cartesian__eml_rand_mt19937ar_a(localB->uv, localDW->state_e, &r
      [localB->b_k_h], localB);
  }
}

static w_robotics_manip_internal_Rig_T *car_RigidBody_RigidBody_byapfac
  (w_robotics_manip_internal_Rig_T *obj, c_rigidBodyJoint_cartesian_tr_T *iobj_0,
   B_MATLABSystem_cartesian_traj_T *localB)
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
  cartes_emxEnsureCapacity_char_T(obj->NameInternal, b_kstr, localB);
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
  cartes_emxEnsureCapacity_char_T(iobj_0->NameInternal, b_kstr, localB);
  for (b_kstr = 0; b_kstr < 14; b_kstr++) {
    iobj_0->NameInternal->data[b_kstr] = tmp_2[b_kstr];
  }

  b_kstr = iobj_0->Type->size[0] * iobj_0->Type->size[1];
  iobj_0->Type->size[0] = 1;
  iobj_0->Type->size[1] = 5;
  cartes_emxEnsureCapacity_char_T(iobj_0->Type, b_kstr, localB);
  for (b_kstr = 0; b_kstr < 5; b_kstr++) {
    iobj_0->Type->data[b_kstr] = tmp_3[b_kstr];
  }

  cartesian_trajec_emxInit_char_T(&switch_expression, 2, localB);
  b_kstr = switch_expression->size[0] * switch_expression->size[1];
  switch_expression->size[0] = 1;
  switch_expression->size[1] = iobj_0->Type->size[1];
  cartes_emxEnsureCapacity_char_T(switch_expression, b_kstr, localB);
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
  cartes_emxEnsureCapacity_real_T(iobj_0->MotionSubspace, b_kstr, localB);
  for (b_kstr = 0; b_kstr < 6; b_kstr++) {
    iobj_0->MotionSubspace->data[b_kstr] = msubspace_data[b_kstr];
  }

  b_kstr = iobj_0->PositionLimitsInternal->size[0] *
    iobj_0->PositionLimitsInternal->size[1];
  iobj_0->PositionLimitsInternal->size[0] = 1;
  iobj_0->PositionLimitsInternal->size[1] = 2;
  cartes_emxEnsureCapacity_real_T(iobj_0->PositionLimitsInternal, b_kstr, localB);
  for (b_kstr = 0; b_kstr < 2; b_kstr++) {
    iobj_0->PositionLimitsInternal->data[b_kstr] = poslim_data[b_kstr];
  }

  b_kstr = iobj_0->HomePositionInternal->size[0];
  iobj_0->HomePositionInternal->size[0] = 1;
  cartes_emxEnsureCapacity_real_T(iobj_0->HomePositionInternal, b_kstr, localB);
  for (b_kstr = 0; b_kstr < 1; b_kstr++) {
    iobj_0->HomePositionInternal->data[0] = 0.0;
  }

  obj->JointInternal = iobj_0;
  obj->Index = -1.0;
  obj->ParentIndex = -1.0;
  return b_obj;
}

static w_robotics_manip_internal_Rig_T *ca_RigidBody_RigidBody_byapfac3
  (w_robotics_manip_internal_Rig_T *obj, c_rigidBodyJoint_cartesian_tr_T *iobj_0,
   B_MATLABSystem_cartesian_traj_T *localB)
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
  cartes_emxEnsureCapacity_char_T(obj->NameInternal, b_kstr, localB);
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
  cartes_emxEnsureCapacity_char_T(iobj_0->NameInternal, b_kstr, localB);
  for (b_kstr = 0; b_kstr < 14; b_kstr++) {
    iobj_0->NameInternal->data[b_kstr] = tmp_2[b_kstr];
  }

  b_kstr = iobj_0->Type->size[0] * iobj_0->Type->size[1];
  iobj_0->Type->size[0] = 1;
  iobj_0->Type->size[1] = 5;
  cartes_emxEnsureCapacity_char_T(iobj_0->Type, b_kstr, localB);
  for (b_kstr = 0; b_kstr < 5; b_kstr++) {
    iobj_0->Type->data[b_kstr] = tmp_3[b_kstr];
  }

  cartesian_trajec_emxInit_char_T(&switch_expression, 2, localB);
  b_kstr = switch_expression->size[0] * switch_expression->size[1];
  switch_expression->size[0] = 1;
  switch_expression->size[1] = iobj_0->Type->size[1];
  cartes_emxEnsureCapacity_char_T(switch_expression, b_kstr, localB);
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
  cartes_emxEnsureCapacity_real_T(iobj_0->MotionSubspace, b_kstr, localB);
  for (b_kstr = 0; b_kstr < 6; b_kstr++) {
    iobj_0->MotionSubspace->data[b_kstr] = msubspace_data[b_kstr];
  }

  b_kstr = iobj_0->PositionLimitsInternal->size[0] *
    iobj_0->PositionLimitsInternal->size[1];
  iobj_0->PositionLimitsInternal->size[0] = 1;
  iobj_0->PositionLimitsInternal->size[1] = 2;
  cartes_emxEnsureCapacity_real_T(iobj_0->PositionLimitsInternal, b_kstr, localB);
  for (b_kstr = 0; b_kstr < 2; b_kstr++) {
    iobj_0->PositionLimitsInternal->data[b_kstr] = poslim_data[b_kstr];
  }

  b_kstr = iobj_0->HomePositionInternal->size[0];
  iobj_0->HomePositionInternal->size[0] = 1;
  cartes_emxEnsureCapacity_real_T(iobj_0->HomePositionInternal, b_kstr, localB);
  for (b_kstr = 0; b_kstr < 1; b_kstr++) {
    iobj_0->HomePositionInternal->data[0] = 0.0;
  }

  obj->JointInternal = iobj_0;
  obj->Index = -1.0;
  obj->ParentIndex = -1.0;
  return b_obj;
}

static w_robotics_manip_internal_Rig_T *c_RigidBody_RigidBody_byapfac3i
  (w_robotics_manip_internal_Rig_T *obj, c_rigidBodyJoint_cartesian_tr_T *iobj_0,
   B_MATLABSystem_cartesian_traj_T *localB)
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
  cartes_emxEnsureCapacity_char_T(obj->NameInternal, b_kstr, localB);
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
  cartes_emxEnsureCapacity_char_T(iobj_0->NameInternal, b_kstr, localB);
  for (b_kstr = 0; b_kstr < 14; b_kstr++) {
    iobj_0->NameInternal->data[b_kstr] = tmp_2[b_kstr];
  }

  b_kstr = iobj_0->Type->size[0] * iobj_0->Type->size[1];
  iobj_0->Type->size[0] = 1;
  iobj_0->Type->size[1] = 5;
  cartes_emxEnsureCapacity_char_T(iobj_0->Type, b_kstr, localB);
  for (b_kstr = 0; b_kstr < 5; b_kstr++) {
    iobj_0->Type->data[b_kstr] = tmp_3[b_kstr];
  }

  cartesian_trajec_emxInit_char_T(&switch_expression, 2, localB);
  b_kstr = switch_expression->size[0] * switch_expression->size[1];
  switch_expression->size[0] = 1;
  switch_expression->size[1] = iobj_0->Type->size[1];
  cartes_emxEnsureCapacity_char_T(switch_expression, b_kstr, localB);
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
  cartes_emxEnsureCapacity_real_T(iobj_0->MotionSubspace, b_kstr, localB);
  for (b_kstr = 0; b_kstr < 6; b_kstr++) {
    iobj_0->MotionSubspace->data[b_kstr] = msubspace_data[b_kstr];
  }

  b_kstr = iobj_0->PositionLimitsInternal->size[0] *
    iobj_0->PositionLimitsInternal->size[1];
  iobj_0->PositionLimitsInternal->size[0] = 1;
  iobj_0->PositionLimitsInternal->size[1] = 2;
  cartes_emxEnsureCapacity_real_T(iobj_0->PositionLimitsInternal, b_kstr, localB);
  for (b_kstr = 0; b_kstr < 2; b_kstr++) {
    iobj_0->PositionLimitsInternal->data[b_kstr] = poslim_data[b_kstr];
  }

  b_kstr = iobj_0->HomePositionInternal->size[0];
  iobj_0->HomePositionInternal->size[0] = 1;
  cartes_emxEnsureCapacity_real_T(iobj_0->HomePositionInternal, b_kstr, localB);
  for (b_kstr = 0; b_kstr < 1; b_kstr++) {
    iobj_0->HomePositionInternal->data[0] = 0.0;
  }

  obj->JointInternal = iobj_0;
  obj->Index = -1.0;
  obj->ParentIndex = -1.0;
  return b_obj;
}

static w_robotics_manip_internal_Rig_T *RigidBody_RigidBody_byapfac3iu
  (w_robotics_manip_internal_Rig_T *obj, c_rigidBodyJoint_cartesian_tr_T *iobj_0,
   B_MATLABSystem_cartesian_traj_T *localB)
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
  cartes_emxEnsureCapacity_char_T(obj->NameInternal, b_kstr, localB);
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
  cartes_emxEnsureCapacity_char_T(iobj_0->NameInternal, b_kstr, localB);
  for (b_kstr = 0; b_kstr < 14; b_kstr++) {
    iobj_0->NameInternal->data[b_kstr] = tmp_2[b_kstr];
  }

  b_kstr = iobj_0->Type->size[0] * iobj_0->Type->size[1];
  iobj_0->Type->size[0] = 1;
  iobj_0->Type->size[1] = 5;
  cartes_emxEnsureCapacity_char_T(iobj_0->Type, b_kstr, localB);
  for (b_kstr = 0; b_kstr < 5; b_kstr++) {
    iobj_0->Type->data[b_kstr] = tmp_3[b_kstr];
  }

  cartesian_trajec_emxInit_char_T(&switch_expression, 2, localB);
  b_kstr = switch_expression->size[0] * switch_expression->size[1];
  switch_expression->size[0] = 1;
  switch_expression->size[1] = iobj_0->Type->size[1];
  cartes_emxEnsureCapacity_char_T(switch_expression, b_kstr, localB);
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
  cartes_emxEnsureCapacity_real_T(iobj_0->MotionSubspace, b_kstr, localB);
  for (b_kstr = 0; b_kstr < 6; b_kstr++) {
    iobj_0->MotionSubspace->data[b_kstr] = msubspace_data[b_kstr];
  }

  b_kstr = iobj_0->PositionLimitsInternal->size[0] *
    iobj_0->PositionLimitsInternal->size[1];
  iobj_0->PositionLimitsInternal->size[0] = 1;
  iobj_0->PositionLimitsInternal->size[1] = 2;
  cartes_emxEnsureCapacity_real_T(iobj_0->PositionLimitsInternal, b_kstr, localB);
  for (b_kstr = 0; b_kstr < 2; b_kstr++) {
    iobj_0->PositionLimitsInternal->data[b_kstr] = poslim_data[b_kstr];
  }

  b_kstr = iobj_0->HomePositionInternal->size[0];
  iobj_0->HomePositionInternal->size[0] = 1;
  cartes_emxEnsureCapacity_real_T(iobj_0->HomePositionInternal, b_kstr, localB);
  for (b_kstr = 0; b_kstr < 1; b_kstr++) {
    iobj_0->HomePositionInternal->data[0] = 0.0;
  }

  obj->JointInternal = iobj_0;
  obj->Index = -1.0;
  obj->ParentIndex = -1.0;
  return b_obj;
}

static w_robotics_manip_internal_Rig_T *RigidBody_RigidBody_byapfac3iu2
  (w_robotics_manip_internal_Rig_T *obj, c_rigidBodyJoint_cartesian_tr_T *iobj_0,
   B_MATLABSystem_cartesian_traj_T *localB)
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
  cartes_emxEnsureCapacity_char_T(obj->NameInternal, b_kstr, localB);
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
  cartes_emxEnsureCapacity_char_T(iobj_0->NameInternal, b_kstr, localB);
  for (b_kstr = 0; b_kstr < 14; b_kstr++) {
    iobj_0->NameInternal->data[b_kstr] = tmp_2[b_kstr];
  }

  b_kstr = iobj_0->Type->size[0] * iobj_0->Type->size[1];
  iobj_0->Type->size[0] = 1;
  iobj_0->Type->size[1] = 5;
  cartes_emxEnsureCapacity_char_T(iobj_0->Type, b_kstr, localB);
  for (b_kstr = 0; b_kstr < 5; b_kstr++) {
    iobj_0->Type->data[b_kstr] = tmp_3[b_kstr];
  }

  cartesian_trajec_emxInit_char_T(&switch_expression, 2, localB);
  b_kstr = switch_expression->size[0] * switch_expression->size[1];
  switch_expression->size[0] = 1;
  switch_expression->size[1] = iobj_0->Type->size[1];
  cartes_emxEnsureCapacity_char_T(switch_expression, b_kstr, localB);
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
  cartes_emxEnsureCapacity_real_T(iobj_0->MotionSubspace, b_kstr, localB);
  for (b_kstr = 0; b_kstr < 6; b_kstr++) {
    iobj_0->MotionSubspace->data[b_kstr] = msubspace_data[b_kstr];
  }

  b_kstr = iobj_0->PositionLimitsInternal->size[0] *
    iobj_0->PositionLimitsInternal->size[1];
  iobj_0->PositionLimitsInternal->size[0] = 1;
  iobj_0->PositionLimitsInternal->size[1] = 2;
  cartes_emxEnsureCapacity_real_T(iobj_0->PositionLimitsInternal, b_kstr, localB);
  for (b_kstr = 0; b_kstr < 2; b_kstr++) {
    iobj_0->PositionLimitsInternal->data[b_kstr] = poslim_data[b_kstr];
  }

  b_kstr = iobj_0->HomePositionInternal->size[0];
  iobj_0->HomePositionInternal->size[0] = 1;
  cartes_emxEnsureCapacity_real_T(iobj_0->HomePositionInternal, b_kstr, localB);
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
  w_robotics_manip_internal_Rig_T *iobj_6, c_rigidBodyJoint_cartesian_tr_T
  *iobj_7, c_rigidBodyJoint_cartesian_tr_T *iobj_8,
  c_rigidBodyJoint_cartesian_tr_T *iobj_9, c_rigidBodyJoint_cartesian_tr_T
  *iobj_10, c_rigidBodyJoint_cartesian_tr_T *iobj_11,
  c_rigidBodyJoint_cartesian_tr_T *iobj_12, c_rigidBodyJoint_cartesian_tr_T
  *iobj_13, c_rigidBodyJoint_cartesian_tr_T *iobj_14,
  w_robotics_manip_internal_Rig_T *iobj_15, B_MATLABSystem_cartesian_traj_T
  *localB, DW_MATLABSystem_cartesian_tra_T *localDW)
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
  obj->Bodies[0] = car_RigidBody_RigidBody_byapfac(iobj_15, iobj_14, localB);
  obj->Bodies[1] = ca_RigidBody_RigidBody_byapfac3(iobj_0, iobj_7, localB);
  obj->Bodies[2] = c_RigidBody_RigidBody_byapfac3i(iobj_1, iobj_8, localB);
  obj->Bodies[3] = RigidBody_RigidBody_byapfac3iu(iobj_2, iobj_9, localB);
  obj->Bodies[4] = RigidBody_RigidBody_byapfac3iu2(iobj_3, iobj_10, localB);
  localB->b_kstr_o = iobj_4->NameInternal->size[0] * iobj_4->NameInternal->size
    [1];
  iobj_4->NameInternal->size[0] = 1;
  iobj_4->NameInternal->size[1] = 10;
  cartes_emxEnsureCapacity_char_T(iobj_4->NameInternal, localB->b_kstr_o, localB);
  for (localB->b_kstr_o = 0; localB->b_kstr_o < 10; localB->b_kstr_o++) {
    iobj_4->NameInternal->data[localB->b_kstr_o] = tmp[localB->b_kstr_o];
  }

  iobj_11->InTree = false;
  for (localB->b_kstr_o = 0; localB->b_kstr_o < 16; localB->b_kstr_o++) {
    iobj_11->JointToParentTransform[localB->b_kstr_o] = tmp_0[localB->b_kstr_o];
  }

  for (localB->b_kstr_o = 0; localB->b_kstr_o < 16; localB->b_kstr_o++) {
    iobj_11->ChildToJointTransform[localB->b_kstr_o] = tmp_0[localB->b_kstr_o];
  }

  localB->b_kstr_o = iobj_11->NameInternal->size[0] * iobj_11->
    NameInternal->size[1];
  iobj_11->NameInternal->size[0] = 1;
  iobj_11->NameInternal->size[1] = 14;
  cartes_emxEnsureCapacity_char_T(iobj_11->NameInternal, localB->b_kstr_o,
    localB);
  for (localB->b_kstr_o = 0; localB->b_kstr_o < 14; localB->b_kstr_o++) {
    iobj_11->NameInternal->data[localB->b_kstr_o] = tmp_1[localB->b_kstr_o];
  }

  localB->b_kstr_o = iobj_11->Type->size[0] * iobj_11->Type->size[1];
  iobj_11->Type->size[0] = 1;
  iobj_11->Type->size[1] = 5;
  cartes_emxEnsureCapacity_char_T(iobj_11->Type, localB->b_kstr_o, localB);
  for (localB->b_kstr_o = 0; localB->b_kstr_o < 5; localB->b_kstr_o++) {
    iobj_11->Type->data[localB->b_kstr_o] = tmp_2[localB->b_kstr_o];
  }

  cartesian_trajec_emxInit_char_T(&switch_expression, 2, localB);
  localB->b_kstr_o = switch_expression->size[0] * switch_expression->size[1];
  switch_expression->size[0] = 1;
  switch_expression->size[1] = iobj_11->Type->size[1];
  cartes_emxEnsureCapacity_char_T(switch_expression, localB->b_kstr_o, localB);
  localB->loop_ub_c = iobj_11->Type->size[0] * iobj_11->Type->size[1] - 1;
  for (localB->b_kstr_o = 0; localB->b_kstr_o <= localB->loop_ub_c;
       localB->b_kstr_o++) {
    switch_expression->data[localB->b_kstr_o] = iobj_11->Type->data
      [localB->b_kstr_o];
  }

  for (localB->b_kstr_o = 0; localB->b_kstr_o < 8; localB->b_kstr_o++) {
    localB->b_p[localB->b_kstr_o] = tmp_3[localB->b_kstr_o];
  }

  localB->b_bool_kb = false;
  if (switch_expression->size[1] == 8) {
    localB->b_kstr_o = 1;
    do {
      exitg1 = 0;
      if (localB->b_kstr_o - 1 < 8) {
        localB->loop_ub_c = localB->b_kstr_o - 1;
        if (switch_expression->data[localB->loop_ub_c] != localB->b_p
            [localB->loop_ub_c]) {
          exitg1 = 1;
        } else {
          localB->b_kstr_o++;
        }
      } else {
        localB->b_bool_kb = true;
        exitg1 = 1;
      }
    } while (exitg1 == 0);
  }

  if (localB->b_bool_kb) {
    localB->b_kstr_o = 0;
  } else {
    for (localB->b_kstr_o = 0; localB->b_kstr_o < 9; localB->b_kstr_o++) {
      localB->b_m[localB->b_kstr_o] = tmp_4[localB->b_kstr_o];
    }

    localB->b_bool_kb = false;
    if (switch_expression->size[1] == 9) {
      localB->b_kstr_o = 1;
      do {
        exitg1 = 0;
        if (localB->b_kstr_o - 1 < 9) {
          localB->loop_ub_c = localB->b_kstr_o - 1;
          if (switch_expression->data[localB->loop_ub_c] != localB->b_m
              [localB->loop_ub_c]) {
            exitg1 = 1;
          } else {
            localB->b_kstr_o++;
          }
        } else {
          localB->b_bool_kb = true;
          exitg1 = 1;
        }
      } while (exitg1 == 0);
    }

    if (localB->b_bool_kb) {
      localB->b_kstr_o = 1;
    } else {
      localB->b_kstr_o = -1;
    }
  }

  switch (localB->b_kstr_o) {
   case 0:
    localB->iv1[0] = 0;
    localB->iv1[1] = 0;
    localB->iv1[2] = 1;
    localB->iv1[3] = 0;
    localB->iv1[4] = 0;
    localB->iv1[5] = 0;
    for (localB->b_kstr_o = 0; localB->b_kstr_o < 6; localB->b_kstr_o++) {
      localB->msubspace_data_a[localB->b_kstr_o] = localB->iv1[localB->b_kstr_o];
    }

    localB->poslim_data_c[0] = -3.1415926535897931;
    localB->poslim_data_c[1] = 3.1415926535897931;
    iobj_11->VelocityNumber = 1.0;
    iobj_11->PositionNumber = 1.0;
    iobj_11->JointAxisInternal[0] = 0.0;
    iobj_11->JointAxisInternal[1] = 0.0;
    iobj_11->JointAxisInternal[2] = 1.0;
    break;

   case 1:
    localB->iv1[0] = 0;
    localB->iv1[1] = 0;
    localB->iv1[2] = 0;
    localB->iv1[3] = 0;
    localB->iv1[4] = 0;
    localB->iv1[5] = 1;
    for (localB->b_kstr_o = 0; localB->b_kstr_o < 6; localB->b_kstr_o++) {
      localB->msubspace_data_a[localB->b_kstr_o] = localB->iv1[localB->b_kstr_o];
    }

    localB->poslim_data_c[0] = -0.5;
    localB->poslim_data_c[1] = 0.5;
    iobj_11->VelocityNumber = 1.0;
    iobj_11->PositionNumber = 1.0;
    iobj_11->JointAxisInternal[0] = 0.0;
    iobj_11->JointAxisInternal[1] = 0.0;
    iobj_11->JointAxisInternal[2] = 1.0;
    break;

   default:
    for (localB->b_kstr_o = 0; localB->b_kstr_o < 6; localB->b_kstr_o++) {
      localB->msubspace_data_a[localB->b_kstr_o] = 0;
    }

    localB->poslim_data_c[0] = 0.0;
    localB->poslim_data_c[1] = 0.0;
    iobj_11->VelocityNumber = 0.0;
    iobj_11->PositionNumber = 0.0;
    iobj_11->JointAxisInternal[0] = 0.0;
    iobj_11->JointAxisInternal[1] = 0.0;
    iobj_11->JointAxisInternal[2] = 0.0;
    break;
  }

  localB->b_kstr_o = iobj_11->MotionSubspace->size[0] * iobj_11->
    MotionSubspace->size[1];
  iobj_11->MotionSubspace->size[0] = 6;
  iobj_11->MotionSubspace->size[1] = 1;
  cartes_emxEnsureCapacity_real_T(iobj_11->MotionSubspace, localB->b_kstr_o,
    localB);
  for (localB->b_kstr_o = 0; localB->b_kstr_o < 6; localB->b_kstr_o++) {
    iobj_11->MotionSubspace->data[localB->b_kstr_o] = localB->
      msubspace_data_a[localB->b_kstr_o];
  }

  localB->b_kstr_o = iobj_11->PositionLimitsInternal->size[0] *
    iobj_11->PositionLimitsInternal->size[1];
  iobj_11->PositionLimitsInternal->size[0] = 1;
  iobj_11->PositionLimitsInternal->size[1] = 2;
  cartes_emxEnsureCapacity_real_T(iobj_11->PositionLimitsInternal,
    localB->b_kstr_o, localB);
  for (localB->b_kstr_o = 0; localB->b_kstr_o < 2; localB->b_kstr_o++) {
    iobj_11->PositionLimitsInternal->data[localB->b_kstr_o] =
      localB->poslim_data_c[localB->b_kstr_o];
  }

  localB->b_kstr_o = iobj_11->HomePositionInternal->size[0];
  iobj_11->HomePositionInternal->size[0] = 1;
  cartes_emxEnsureCapacity_real_T(iobj_11->HomePositionInternal,
    localB->b_kstr_o, localB);
  for (localB->b_kstr_o = 0; localB->b_kstr_o < 1; localB->b_kstr_o++) {
    iobj_11->HomePositionInternal->data[0] = 0.0;
  }

  iobj_4->JointInternal = iobj_11;
  iobj_4->Index = -1.0;
  iobj_4->ParentIndex = -1.0;
  obj->Bodies[5] = iobj_4;
  localB->b_kstr_o = iobj_5->NameInternal->size[0] * iobj_5->NameInternal->size
    [1];
  iobj_5->NameInternal->size[0] = 1;
  iobj_5->NameInternal->size[1] = 10;
  cartes_emxEnsureCapacity_char_T(iobj_5->NameInternal, localB->b_kstr_o, localB);
  for (localB->b_kstr_o = 0; localB->b_kstr_o < 10; localB->b_kstr_o++) {
    iobj_5->NameInternal->data[localB->b_kstr_o] = tmp_5[localB->b_kstr_o];
  }

  iobj_12->InTree = false;
  for (localB->b_kstr_o = 0; localB->b_kstr_o < 16; localB->b_kstr_o++) {
    iobj_12->JointToParentTransform[localB->b_kstr_o] = tmp_0[localB->b_kstr_o];
  }

  for (localB->b_kstr_o = 0; localB->b_kstr_o < 16; localB->b_kstr_o++) {
    iobj_12->ChildToJointTransform[localB->b_kstr_o] = tmp_0[localB->b_kstr_o];
  }

  localB->b_kstr_o = iobj_12->NameInternal->size[0] * iobj_12->
    NameInternal->size[1];
  iobj_12->NameInternal->size[0] = 1;
  iobj_12->NameInternal->size[1] = 14;
  cartes_emxEnsureCapacity_char_T(iobj_12->NameInternal, localB->b_kstr_o,
    localB);
  for (localB->b_kstr_o = 0; localB->b_kstr_o < 14; localB->b_kstr_o++) {
    iobj_12->NameInternal->data[localB->b_kstr_o] = tmp_6[localB->b_kstr_o];
  }

  localB->b_kstr_o = iobj_12->Type->size[0] * iobj_12->Type->size[1];
  iobj_12->Type->size[0] = 1;
  iobj_12->Type->size[1] = 5;
  cartes_emxEnsureCapacity_char_T(iobj_12->Type, localB->b_kstr_o, localB);
  for (localB->b_kstr_o = 0; localB->b_kstr_o < 5; localB->b_kstr_o++) {
    iobj_12->Type->data[localB->b_kstr_o] = tmp_2[localB->b_kstr_o];
  }

  localB->b_kstr_o = switch_expression->size[0] * switch_expression->size[1];
  switch_expression->size[0] = 1;
  switch_expression->size[1] = iobj_12->Type->size[1];
  cartes_emxEnsureCapacity_char_T(switch_expression, localB->b_kstr_o, localB);
  localB->loop_ub_c = iobj_12->Type->size[0] * iobj_12->Type->size[1] - 1;
  for (localB->b_kstr_o = 0; localB->b_kstr_o <= localB->loop_ub_c;
       localB->b_kstr_o++) {
    switch_expression->data[localB->b_kstr_o] = iobj_12->Type->data
      [localB->b_kstr_o];
  }

  localB->b_bool_kb = false;
  if (switch_expression->size[1] == 8) {
    localB->b_kstr_o = 1;
    do {
      exitg1 = 0;
      if (localB->b_kstr_o - 1 < 8) {
        localB->loop_ub_c = localB->b_kstr_o - 1;
        if (switch_expression->data[localB->loop_ub_c] != localB->b_p
            [localB->loop_ub_c]) {
          exitg1 = 1;
        } else {
          localB->b_kstr_o++;
        }
      } else {
        localB->b_bool_kb = true;
        exitg1 = 1;
      }
    } while (exitg1 == 0);
  }

  if (localB->b_bool_kb) {
    localB->b_kstr_o = 0;
  } else {
    for (localB->b_kstr_o = 0; localB->b_kstr_o < 9; localB->b_kstr_o++) {
      localB->b_m[localB->b_kstr_o] = tmp_4[localB->b_kstr_o];
    }

    localB->b_bool_kb = false;
    if (switch_expression->size[1] == 9) {
      localB->b_kstr_o = 1;
      do {
        exitg1 = 0;
        if (localB->b_kstr_o - 1 < 9) {
          localB->loop_ub_c = localB->b_kstr_o - 1;
          if (switch_expression->data[localB->loop_ub_c] != localB->b_m
              [localB->loop_ub_c]) {
            exitg1 = 1;
          } else {
            localB->b_kstr_o++;
          }
        } else {
          localB->b_bool_kb = true;
          exitg1 = 1;
        }
      } while (exitg1 == 0);
    }

    if (localB->b_bool_kb) {
      localB->b_kstr_o = 1;
    } else {
      localB->b_kstr_o = -1;
    }
  }

  switch (localB->b_kstr_o) {
   case 0:
    localB->iv1[0] = 0;
    localB->iv1[1] = 0;
    localB->iv1[2] = 1;
    localB->iv1[3] = 0;
    localB->iv1[4] = 0;
    localB->iv1[5] = 0;
    for (localB->b_kstr_o = 0; localB->b_kstr_o < 6; localB->b_kstr_o++) {
      localB->msubspace_data_a[localB->b_kstr_o] = localB->iv1[localB->b_kstr_o];
    }

    localB->poslim_data_c[0] = -3.1415926535897931;
    localB->poslim_data_c[1] = 3.1415926535897931;
    iobj_12->VelocityNumber = 1.0;
    iobj_12->PositionNumber = 1.0;
    iobj_12->JointAxisInternal[0] = 0.0;
    iobj_12->JointAxisInternal[1] = 0.0;
    iobj_12->JointAxisInternal[2] = 1.0;
    break;

   case 1:
    localB->iv1[0] = 0;
    localB->iv1[1] = 0;
    localB->iv1[2] = 0;
    localB->iv1[3] = 0;
    localB->iv1[4] = 0;
    localB->iv1[5] = 1;
    for (localB->b_kstr_o = 0; localB->b_kstr_o < 6; localB->b_kstr_o++) {
      localB->msubspace_data_a[localB->b_kstr_o] = localB->iv1[localB->b_kstr_o];
    }

    localB->poslim_data_c[0] = -0.5;
    localB->poslim_data_c[1] = 0.5;
    iobj_12->VelocityNumber = 1.0;
    iobj_12->PositionNumber = 1.0;
    iobj_12->JointAxisInternal[0] = 0.0;
    iobj_12->JointAxisInternal[1] = 0.0;
    iobj_12->JointAxisInternal[2] = 1.0;
    break;

   default:
    for (localB->b_kstr_o = 0; localB->b_kstr_o < 6; localB->b_kstr_o++) {
      localB->msubspace_data_a[localB->b_kstr_o] = 0;
    }

    localB->poslim_data_c[0] = 0.0;
    localB->poslim_data_c[1] = 0.0;
    iobj_12->VelocityNumber = 0.0;
    iobj_12->PositionNumber = 0.0;
    iobj_12->JointAxisInternal[0] = 0.0;
    iobj_12->JointAxisInternal[1] = 0.0;
    iobj_12->JointAxisInternal[2] = 0.0;
    break;
  }

  localB->b_kstr_o = iobj_12->MotionSubspace->size[0] * iobj_12->
    MotionSubspace->size[1];
  iobj_12->MotionSubspace->size[0] = 6;
  iobj_12->MotionSubspace->size[1] = 1;
  cartes_emxEnsureCapacity_real_T(iobj_12->MotionSubspace, localB->b_kstr_o,
    localB);
  for (localB->b_kstr_o = 0; localB->b_kstr_o < 6; localB->b_kstr_o++) {
    iobj_12->MotionSubspace->data[localB->b_kstr_o] = localB->
      msubspace_data_a[localB->b_kstr_o];
  }

  localB->b_kstr_o = iobj_12->PositionLimitsInternal->size[0] *
    iobj_12->PositionLimitsInternal->size[1];
  iobj_12->PositionLimitsInternal->size[0] = 1;
  iobj_12->PositionLimitsInternal->size[1] = 2;
  cartes_emxEnsureCapacity_real_T(iobj_12->PositionLimitsInternal,
    localB->b_kstr_o, localB);
  for (localB->b_kstr_o = 0; localB->b_kstr_o < 2; localB->b_kstr_o++) {
    iobj_12->PositionLimitsInternal->data[localB->b_kstr_o] =
      localB->poslim_data_c[localB->b_kstr_o];
  }

  localB->b_kstr_o = iobj_12->HomePositionInternal->size[0];
  iobj_12->HomePositionInternal->size[0] = 1;
  cartes_emxEnsureCapacity_real_T(iobj_12->HomePositionInternal,
    localB->b_kstr_o, localB);
  for (localB->b_kstr_o = 0; localB->b_kstr_o < 1; localB->b_kstr_o++) {
    iobj_12->HomePositionInternal->data[0] = 0.0;
  }

  iobj_5->JointInternal = iobj_12;
  iobj_5->Index = -1.0;
  iobj_5->ParentIndex = -1.0;
  obj->Bodies[6] = iobj_5;
  localB->b_kstr_o = iobj_6->NameInternal->size[0] * iobj_6->NameInternal->size
    [1];
  iobj_6->NameInternal->size[0] = 1;
  iobj_6->NameInternal->size[1] = 10;
  cartes_emxEnsureCapacity_char_T(iobj_6->NameInternal, localB->b_kstr_o, localB);
  for (localB->b_kstr_o = 0; localB->b_kstr_o < 10; localB->b_kstr_o++) {
    iobj_6->NameInternal->data[localB->b_kstr_o] = tmp_7[localB->b_kstr_o];
  }

  iobj_13->InTree = false;
  for (localB->b_kstr_o = 0; localB->b_kstr_o < 16; localB->b_kstr_o++) {
    iobj_13->JointToParentTransform[localB->b_kstr_o] = tmp_0[localB->b_kstr_o];
  }

  for (localB->b_kstr_o = 0; localB->b_kstr_o < 16; localB->b_kstr_o++) {
    iobj_13->ChildToJointTransform[localB->b_kstr_o] = tmp_0[localB->b_kstr_o];
  }

  localB->b_kstr_o = iobj_13->NameInternal->size[0] * iobj_13->
    NameInternal->size[1];
  iobj_13->NameInternal->size[0] = 1;
  iobj_13->NameInternal->size[1] = 14;
  cartes_emxEnsureCapacity_char_T(iobj_13->NameInternal, localB->b_kstr_o,
    localB);
  for (localB->b_kstr_o = 0; localB->b_kstr_o < 14; localB->b_kstr_o++) {
    iobj_13->NameInternal->data[localB->b_kstr_o] = tmp_8[localB->b_kstr_o];
  }

  localB->b_kstr_o = iobj_13->Type->size[0] * iobj_13->Type->size[1];
  iobj_13->Type->size[0] = 1;
  iobj_13->Type->size[1] = 5;
  cartes_emxEnsureCapacity_char_T(iobj_13->Type, localB->b_kstr_o, localB);
  for (localB->b_kstr_o = 0; localB->b_kstr_o < 5; localB->b_kstr_o++) {
    iobj_13->Type->data[localB->b_kstr_o] = tmp_2[localB->b_kstr_o];
  }

  localB->b_kstr_o = switch_expression->size[0] * switch_expression->size[1];
  switch_expression->size[0] = 1;
  switch_expression->size[1] = iobj_13->Type->size[1];
  cartes_emxEnsureCapacity_char_T(switch_expression, localB->b_kstr_o, localB);
  localB->loop_ub_c = iobj_13->Type->size[0] * iobj_13->Type->size[1] - 1;
  for (localB->b_kstr_o = 0; localB->b_kstr_o <= localB->loop_ub_c;
       localB->b_kstr_o++) {
    switch_expression->data[localB->b_kstr_o] = iobj_13->Type->data
      [localB->b_kstr_o];
  }

  localB->b_bool_kb = false;
  if (switch_expression->size[1] == 8) {
    localB->b_kstr_o = 1;
    do {
      exitg1 = 0;
      if (localB->b_kstr_o - 1 < 8) {
        localB->loop_ub_c = localB->b_kstr_o - 1;
        if (switch_expression->data[localB->loop_ub_c] != localB->b_p
            [localB->loop_ub_c]) {
          exitg1 = 1;
        } else {
          localB->b_kstr_o++;
        }
      } else {
        localB->b_bool_kb = true;
        exitg1 = 1;
      }
    } while (exitg1 == 0);
  }

  if (localB->b_bool_kb) {
    localB->b_kstr_o = 0;
  } else {
    for (localB->b_kstr_o = 0; localB->b_kstr_o < 9; localB->b_kstr_o++) {
      localB->b_m[localB->b_kstr_o] = tmp_4[localB->b_kstr_o];
    }

    localB->b_bool_kb = false;
    if (switch_expression->size[1] == 9) {
      localB->b_kstr_o = 1;
      do {
        exitg1 = 0;
        if (localB->b_kstr_o - 1 < 9) {
          localB->loop_ub_c = localB->b_kstr_o - 1;
          if (switch_expression->data[localB->loop_ub_c] != localB->b_m
              [localB->loop_ub_c]) {
            exitg1 = 1;
          } else {
            localB->b_kstr_o++;
          }
        } else {
          localB->b_bool_kb = true;
          exitg1 = 1;
        }
      } while (exitg1 == 0);
    }

    if (localB->b_bool_kb) {
      localB->b_kstr_o = 1;
    } else {
      localB->b_kstr_o = -1;
    }
  }

  cartesian_trajec_emxFree_char_T(&switch_expression);
  switch (localB->b_kstr_o) {
   case 0:
    localB->iv1[0] = 0;
    localB->iv1[1] = 0;
    localB->iv1[2] = 1;
    localB->iv1[3] = 0;
    localB->iv1[4] = 0;
    localB->iv1[5] = 0;
    for (localB->b_kstr_o = 0; localB->b_kstr_o < 6; localB->b_kstr_o++) {
      localB->msubspace_data_a[localB->b_kstr_o] = localB->iv1[localB->b_kstr_o];
    }

    localB->poslim_data_c[0] = -3.1415926535897931;
    localB->poslim_data_c[1] = 3.1415926535897931;
    iobj_13->VelocityNumber = 1.0;
    iobj_13->PositionNumber = 1.0;
    iobj_13->JointAxisInternal[0] = 0.0;
    iobj_13->JointAxisInternal[1] = 0.0;
    iobj_13->JointAxisInternal[2] = 1.0;
    break;

   case 1:
    localB->iv1[0] = 0;
    localB->iv1[1] = 0;
    localB->iv1[2] = 0;
    localB->iv1[3] = 0;
    localB->iv1[4] = 0;
    localB->iv1[5] = 1;
    for (localB->b_kstr_o = 0; localB->b_kstr_o < 6; localB->b_kstr_o++) {
      localB->msubspace_data_a[localB->b_kstr_o] = localB->iv1[localB->b_kstr_o];
    }

    localB->poslim_data_c[0] = -0.5;
    localB->poslim_data_c[1] = 0.5;
    iobj_13->VelocityNumber = 1.0;
    iobj_13->PositionNumber = 1.0;
    iobj_13->JointAxisInternal[0] = 0.0;
    iobj_13->JointAxisInternal[1] = 0.0;
    iobj_13->JointAxisInternal[2] = 1.0;
    break;

   default:
    for (localB->b_kstr_o = 0; localB->b_kstr_o < 6; localB->b_kstr_o++) {
      localB->msubspace_data_a[localB->b_kstr_o] = 0;
    }

    localB->poslim_data_c[0] = 0.0;
    localB->poslim_data_c[1] = 0.0;
    iobj_13->VelocityNumber = 0.0;
    iobj_13->PositionNumber = 0.0;
    iobj_13->JointAxisInternal[0] = 0.0;
    iobj_13->JointAxisInternal[1] = 0.0;
    iobj_13->JointAxisInternal[2] = 0.0;
    break;
  }

  localB->b_kstr_o = iobj_13->MotionSubspace->size[0] * iobj_13->
    MotionSubspace->size[1];
  iobj_13->MotionSubspace->size[0] = 6;
  iobj_13->MotionSubspace->size[1] = 1;
  cartes_emxEnsureCapacity_real_T(iobj_13->MotionSubspace, localB->b_kstr_o,
    localB);
  for (localB->b_kstr_o = 0; localB->b_kstr_o < 6; localB->b_kstr_o++) {
    iobj_13->MotionSubspace->data[localB->b_kstr_o] = localB->
      msubspace_data_a[localB->b_kstr_o];
  }

  localB->b_kstr_o = iobj_13->PositionLimitsInternal->size[0] *
    iobj_13->PositionLimitsInternal->size[1];
  iobj_13->PositionLimitsInternal->size[0] = 1;
  iobj_13->PositionLimitsInternal->size[1] = 2;
  cartes_emxEnsureCapacity_real_T(iobj_13->PositionLimitsInternal,
    localB->b_kstr_o, localB);
  for (localB->b_kstr_o = 0; localB->b_kstr_o < 2; localB->b_kstr_o++) {
    iobj_13->PositionLimitsInternal->data[localB->b_kstr_o] =
      localB->poslim_data_c[localB->b_kstr_o];
  }

  localB->b_kstr_o = iobj_13->HomePositionInternal->size[0];
  iobj_13->HomePositionInternal->size[0] = 1;
  cartes_emxEnsureCapacity_real_T(iobj_13->HomePositionInternal,
    localB->b_kstr_o, localB);
  for (localB->b_kstr_o = 0; localB->b_kstr_o < 1; localB->b_kstr_o++) {
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
  cartesian_trajectory_plann_rand(localB->unusedExpr_c, localB, localDW);
  for (localB->b_kstr_o = 0; localB->b_kstr_o < 8; localB->b_kstr_o++) {
    obj->PositionDoFMap[localB->b_kstr_o] = 0.0;
  }

  for (localB->b_kstr_o = 0; localB->b_kstr_o < 8; localB->b_kstr_o++) {
    obj->PositionDoFMap[localB->b_kstr_o + 8] = -1.0;
  }

  for (localB->b_kstr_o = 0; localB->b_kstr_o < 8; localB->b_kstr_o++) {
    obj->VelocityDoFMap[localB->b_kstr_o] = 0.0;
  }

  for (localB->b_kstr_o = 0; localB->b_kstr_o < 8; localB->b_kstr_o++) {
    obj->VelocityDoFMap[localB->b_kstr_o + 8] = -1.0;
  }
}

static x_robotics_manip_internal_Rig_T *c_RigidBodyTree_RigidBodyTree_i
  (x_robotics_manip_internal_Rig_T *obj, w_robotics_manip_internal_Rig_T *iobj_0,
   w_robotics_manip_internal_Rig_T *iobj_1, w_robotics_manip_internal_Rig_T
   *iobj_2, w_robotics_manip_internal_Rig_T *iobj_3,
   w_robotics_manip_internal_Rig_T *iobj_4, w_robotics_manip_internal_Rig_T
   *iobj_5, w_robotics_manip_internal_Rig_T *iobj_6,
   c_rigidBodyJoint_cartesian_tr_T *iobj_7, c_rigidBodyJoint_cartesian_tr_T
   *iobj_8, c_rigidBodyJoint_cartesian_tr_T *iobj_9,
   c_rigidBodyJoint_cartesian_tr_T *iobj_10, c_rigidBodyJoint_cartesian_tr_T
   *iobj_11, c_rigidBodyJoint_cartesian_tr_T *iobj_12,
   c_rigidBodyJoint_cartesian_tr_T *iobj_13, c_rigidBodyJoint_cartesian_tr_T
   *iobj_14, c_rigidBodyJoint_cartesian_tr_T *iobj_15,
   w_robotics_manip_internal_Rig_T *iobj_16, B_MATLABSystem_cartesian_traj_T
   *localB, DW_MATLABSystem_cartesian_tra_T *localDW)
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
  localB->b_kstr_k = obj->Base.NameInternal->size[0] * obj->
    Base.NameInternal->size[1];
  obj->Base.NameInternal->size[0] = 1;
  obj->Base.NameInternal->size[1] = 4;
  cartes_emxEnsureCapacity_char_T(obj->Base.NameInternal, localB->b_kstr_k,
    localB);
  obj->Base.NameInternal->data[0] = 'b';
  obj->Base.NameInternal->data[1] = 'a';
  obj->Base.NameInternal->data[2] = 's';
  obj->Base.NameInternal->data[3] = 'e';
  iobj_15->InTree = false;
  for (localB->b_kstr_k = 0; localB->b_kstr_k < 16; localB->b_kstr_k++) {
    iobj_15->JointToParentTransform[localB->b_kstr_k] = tmp[localB->b_kstr_k];
  }

  for (localB->b_kstr_k = 0; localB->b_kstr_k < 16; localB->b_kstr_k++) {
    iobj_15->ChildToJointTransform[localB->b_kstr_k] = tmp[localB->b_kstr_k];
  }

  localB->b_kstr_k = iobj_15->NameInternal->size[0] * iobj_15->
    NameInternal->size[1];
  iobj_15->NameInternal->size[0] = 1;
  iobj_15->NameInternal->size[1] = 8;
  cartes_emxEnsureCapacity_char_T(iobj_15->NameInternal, localB->b_kstr_k,
    localB);
  for (localB->b_kstr_k = 0; localB->b_kstr_k < 8; localB->b_kstr_k++) {
    iobj_15->NameInternal->data[localB->b_kstr_k] = tmp_0[localB->b_kstr_k];
  }

  localB->b_kstr_k = iobj_15->Type->size[0] * iobj_15->Type->size[1];
  iobj_15->Type->size[0] = 1;
  iobj_15->Type->size[1] = 5;
  cartes_emxEnsureCapacity_char_T(iobj_15->Type, localB->b_kstr_k, localB);
  for (localB->b_kstr_k = 0; localB->b_kstr_k < 5; localB->b_kstr_k++) {
    iobj_15->Type->data[localB->b_kstr_k] = tmp_1[localB->b_kstr_k];
  }

  cartesian_trajec_emxInit_char_T(&switch_expression, 2, localB);
  localB->b_kstr_k = switch_expression->size[0] * switch_expression->size[1];
  switch_expression->size[0] = 1;
  switch_expression->size[1] = iobj_15->Type->size[1];
  cartes_emxEnsureCapacity_char_T(switch_expression, localB->b_kstr_k, localB);
  localB->loop_ub_j = iobj_15->Type->size[0] * iobj_15->Type->size[1] - 1;
  for (localB->b_kstr_k = 0; localB->b_kstr_k <= localB->loop_ub_j;
       localB->b_kstr_k++) {
    switch_expression->data[localB->b_kstr_k] = iobj_15->Type->data
      [localB->b_kstr_k];
  }

  for (localB->b_kstr_k = 0; localB->b_kstr_k < 8; localB->b_kstr_k++) {
    localB->b_k[localB->b_kstr_k] = tmp_2[localB->b_kstr_k];
  }

  localB->b_bool_j = false;
  if (switch_expression->size[1] == 8) {
    localB->b_kstr_k = 1;
    do {
      exitg1 = 0;
      if (localB->b_kstr_k - 1 < 8) {
        localB->loop_ub_j = localB->b_kstr_k - 1;
        if (switch_expression->data[localB->loop_ub_j] != localB->b_k
            [localB->loop_ub_j]) {
          exitg1 = 1;
        } else {
          localB->b_kstr_k++;
        }
      } else {
        localB->b_bool_j = true;
        exitg1 = 1;
      }
    } while (exitg1 == 0);
  }

  if (localB->b_bool_j) {
    localB->b_kstr_k = 0;
  } else {
    for (localB->b_kstr_k = 0; localB->b_kstr_k < 9; localB->b_kstr_k++) {
      localB->b_o[localB->b_kstr_k] = tmp_3[localB->b_kstr_k];
    }

    localB->b_bool_j = false;
    if (switch_expression->size[1] == 9) {
      localB->b_kstr_k = 1;
      do {
        exitg1 = 0;
        if (localB->b_kstr_k - 1 < 9) {
          localB->loop_ub_j = localB->b_kstr_k - 1;
          if (switch_expression->data[localB->loop_ub_j] != localB->b_o
              [localB->loop_ub_j]) {
            exitg1 = 1;
          } else {
            localB->b_kstr_k++;
          }
        } else {
          localB->b_bool_j = true;
          exitg1 = 1;
        }
      } while (exitg1 == 0);
    }

    if (localB->b_bool_j) {
      localB->b_kstr_k = 1;
    } else {
      localB->b_kstr_k = -1;
    }
  }

  cartesian_trajec_emxFree_char_T(&switch_expression);
  switch (localB->b_kstr_k) {
   case 0:
    localB->iv[0] = 0;
    localB->iv[1] = 0;
    localB->iv[2] = 1;
    localB->iv[3] = 0;
    localB->iv[4] = 0;
    localB->iv[5] = 0;
    for (localB->b_kstr_k = 0; localB->b_kstr_k < 6; localB->b_kstr_k++) {
      localB->msubspace_data[localB->b_kstr_k] = localB->iv[localB->b_kstr_k];
    }

    localB->poslim_data[0] = -3.1415926535897931;
    localB->poslim_data[1] = 3.1415926535897931;
    iobj_15->VelocityNumber = 1.0;
    iobj_15->PositionNumber = 1.0;
    iobj_15->JointAxisInternal[0] = 0.0;
    iobj_15->JointAxisInternal[1] = 0.0;
    iobj_15->JointAxisInternal[2] = 1.0;
    break;

   case 1:
    localB->iv[0] = 0;
    localB->iv[1] = 0;
    localB->iv[2] = 0;
    localB->iv[3] = 0;
    localB->iv[4] = 0;
    localB->iv[5] = 1;
    for (localB->b_kstr_k = 0; localB->b_kstr_k < 6; localB->b_kstr_k++) {
      localB->msubspace_data[localB->b_kstr_k] = localB->iv[localB->b_kstr_k];
    }

    localB->poslim_data[0] = -0.5;
    localB->poslim_data[1] = 0.5;
    iobj_15->VelocityNumber = 1.0;
    iobj_15->PositionNumber = 1.0;
    iobj_15->JointAxisInternal[0] = 0.0;
    iobj_15->JointAxisInternal[1] = 0.0;
    iobj_15->JointAxisInternal[2] = 1.0;
    break;

   default:
    for (localB->b_kstr_k = 0; localB->b_kstr_k < 6; localB->b_kstr_k++) {
      localB->msubspace_data[localB->b_kstr_k] = 0;
    }

    localB->poslim_data[0] = 0.0;
    localB->poslim_data[1] = 0.0;
    iobj_15->VelocityNumber = 0.0;
    iobj_15->PositionNumber = 0.0;
    iobj_15->JointAxisInternal[0] = 0.0;
    iobj_15->JointAxisInternal[1] = 0.0;
    iobj_15->JointAxisInternal[2] = 0.0;
    break;
  }

  localB->b_kstr_k = iobj_15->MotionSubspace->size[0] * iobj_15->
    MotionSubspace->size[1];
  iobj_15->MotionSubspace->size[0] = 6;
  iobj_15->MotionSubspace->size[1] = 1;
  cartes_emxEnsureCapacity_real_T(iobj_15->MotionSubspace, localB->b_kstr_k,
    localB);
  for (localB->b_kstr_k = 0; localB->b_kstr_k < 6; localB->b_kstr_k++) {
    iobj_15->MotionSubspace->data[localB->b_kstr_k] = localB->
      msubspace_data[localB->b_kstr_k];
  }

  localB->b_kstr_k = iobj_15->PositionLimitsInternal->size[0] *
    iobj_15->PositionLimitsInternal->size[1];
  iobj_15->PositionLimitsInternal->size[0] = 1;
  iobj_15->PositionLimitsInternal->size[1] = 2;
  cartes_emxEnsureCapacity_real_T(iobj_15->PositionLimitsInternal,
    localB->b_kstr_k, localB);
  for (localB->b_kstr_k = 0; localB->b_kstr_k < 2; localB->b_kstr_k++) {
    iobj_15->PositionLimitsInternal->data[localB->b_kstr_k] =
      localB->poslim_data[localB->b_kstr_k];
  }

  localB->b_kstr_k = iobj_15->HomePositionInternal->size[0];
  iobj_15->HomePositionInternal->size[0] = 1;
  cartes_emxEnsureCapacity_real_T(iobj_15->HomePositionInternal,
    localB->b_kstr_k, localB);
  for (localB->b_kstr_k = 0; localB->b_kstr_k < 1; localB->b_kstr_k++) {
    iobj_15->HomePositionInternal->data[0] = 0.0;
  }

  obj->Base.JointInternal = iobj_15;
  obj->Base.Index = -1.0;
  obj->Base.ParentIndex = -1.0;
  obj->Base.Index = 0.0;
  cartesian_trajectory_plann_rand(localB->unusedExpr, localB, localDW);
  ca_RigidBodyTree_clearAllBodies(obj, iobj_0, iobj_1, iobj_2, iobj_3, iobj_4,
    iobj_5, iobj_6, iobj_8, iobj_9, iobj_10, iobj_11, iobj_12, iobj_13, iobj_14,
    iobj_7, iobj_16, localB, localDW);
  return b_obj;
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

static c_rigidBodyJoint_cartesian_tr_T *c_rigidBodyJoint_rigidBodyJoint
  (c_rigidBodyJoint_cartesian_tr_T *obj, const emxArray_char_T_cartesian_tra_T
   *jname, const emxArray_char_T_cartesian_tra_T *jtype,
   B_MATLABSystem_cartesian_traj_T *localB)
{
  c_rigidBodyJoint_cartesian_tr_T *b_obj;
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
  for (localB->minnanb = 0; localB->minnanb < 16; localB->minnanb++) {
    obj->JointToParentTransform[localB->minnanb] = tmp[localB->minnanb];
  }

  for (localB->minnanb = 0; localB->minnanb < 16; localB->minnanb++) {
    obj->ChildToJointTransform[localB->minnanb] = tmp[localB->minnanb];
  }

  b_obj = obj;
  localB->minnanb = obj->NameInternal->size[0] * obj->NameInternal->size[1];
  obj->NameInternal->size[0] = 1;
  obj->NameInternal->size[1] = jname->size[1];
  cartes_emxEnsureCapacity_char_T(obj->NameInternal, localB->minnanb, localB);
  localB->loop_ub_dr = jname->size[0] * jname->size[1] - 1;
  for (localB->minnanb = 0; localB->minnanb <= localB->loop_ub_dr;
       localB->minnanb++) {
    obj->NameInternal->data[localB->minnanb] = jname->data[localB->minnanb];
  }

  localB->partial_match_size_idx_1 = 0;
  for (localB->minnanb = 0; localB->minnanb < 8; localB->minnanb++) {
    localB->vstr[localB->minnanb] = tmp_0[localB->minnanb];
  }

  guard1 = false;
  guard2 = false;
  guard3 = false;
  guard4 = false;
  if (jtype->size[1] <= 8) {
    localB->loop_ub_dr = jtype->size[1];
    for (localB->minnanb = 0; localB->minnanb < 8; localB->minnanb++) {
      localB->b_n[localB->minnanb] = tmp_0[localB->minnanb];
    }

    localB->b_bool_e = false;
    localB->minnanb = jtype->size[1];
    if (localB->minnanb >= 8) {
      localB->minnanb = 8;
    }

    guard11 = false;
    if (localB->loop_ub_dr <= localB->minnanb) {
      if (localB->minnanb < localB->loop_ub_dr) {
        localB->loop_ub_dr = localB->minnanb;
      }

      localB->minnanb = localB->loop_ub_dr - 1;
      guard11 = true;
    } else {
      if (jtype->size[1] == 8) {
        localB->minnanb = 7;
        guard11 = true;
      }
    }

    if (guard11) {
      localB->loop_ub_dr = 1;
      do {
        exitg1 = 0;
        if (localB->loop_ub_dr - 1 <= localB->minnanb) {
          localB->kstr_k = localB->loop_ub_dr - 1;
          if (tmp_3[static_cast<uint8_T>(jtype->data[localB->kstr_k]) & 127] !=
              tmp_3[static_cast<int32_T>(localB->b_n[localB->kstr_k])]) {
            exitg1 = 1;
          } else {
            localB->loop_ub_dr++;
          }
        } else {
          localB->b_bool_e = true;
          exitg1 = 1;
        }
      } while (exitg1 == 0);
    }

    if (localB->b_bool_e) {
      if (jtype->size[1] == 8) {
        localB->nmatched = 1;
        localB->partial_match_size_idx_1 = 8;
        for (localB->minnanb = 0; localB->minnanb < 8; localB->minnanb++) {
          localB->b_f[localB->minnanb] = localB->vstr[localB->minnanb];
        }
      } else {
        localB->partial_match_size_idx_1 = 8;
        for (localB->minnanb = 0; localB->minnanb < 8; localB->minnanb++) {
          localB->partial_match_data[localB->minnanb] = localB->vstr
            [localB->minnanb];
        }

        localB->matched = true;
        localB->nmatched = 1;
        guard3 = true;
      }
    } else {
      guard4 = true;
    }
  } else {
    guard4 = true;
  }

  if (guard4) {
    localB->matched = false;
    localB->nmatched = 0;
    guard3 = true;
  }

  if (guard3) {
    for (localB->minnanb = 0; localB->minnanb < 9; localB->minnanb++) {
      localB->b_vstr[localB->minnanb] = tmp_1[localB->minnanb];
    }

    if (jtype->size[1] <= 9) {
      localB->loop_ub_dr = jtype->size[1];
      for (localB->minnanb = 0; localB->minnanb < 9; localB->minnanb++) {
        localB->b_f[localB->minnanb] = tmp_1[localB->minnanb];
      }

      localB->b_bool_e = false;
      localB->minnanb = jtype->size[1];
      if (localB->minnanb >= 9) {
        localB->minnanb = 9;
      }

      guard11 = false;
      if (localB->loop_ub_dr <= localB->minnanb) {
        if (localB->minnanb < localB->loop_ub_dr) {
          localB->loop_ub_dr = localB->minnanb;
        }

        localB->minnanb = localB->loop_ub_dr - 1;
        guard11 = true;
      } else {
        if (jtype->size[1] == 9) {
          localB->minnanb = 8;
          guard11 = true;
        }
      }

      if (guard11) {
        localB->loop_ub_dr = 1;
        do {
          exitg1 = 0;
          if (localB->loop_ub_dr - 1 <= localB->minnanb) {
            localB->kstr_k = localB->loop_ub_dr - 1;
            if (tmp_3[static_cast<uint8_T>(jtype->data[localB->kstr_k]) & 127]
                != tmp_3[static_cast<int32_T>(localB->b_f[localB->kstr_k])]) {
              exitg1 = 1;
            } else {
              localB->loop_ub_dr++;
            }
          } else {
            localB->b_bool_e = true;
            exitg1 = 1;
          }
        } while (exitg1 == 0);
      }

      if (localB->b_bool_e) {
        if (jtype->size[1] == 9) {
          localB->nmatched = 1;
          localB->partial_match_size_idx_1 = 9;
          for (localB->minnanb = 0; localB->minnanb < 9; localB->minnanb++) {
            localB->b_f[localB->minnanb] = localB->b_vstr[localB->minnanb];
          }
        } else {
          if (!localB->matched) {
            localB->partial_match_size_idx_1 = 9;
            for (localB->minnanb = 0; localB->minnanb < 9; localB->minnanb++) {
              localB->partial_match_data[localB->minnanb] = localB->
                b_vstr[localB->minnanb];
            }
          }

          localB->matched = true;
          localB->nmatched++;
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
    for (localB->minnanb = 0; localB->minnanb < 5; localB->minnanb++) {
      localB->c_vstr[localB->minnanb] = tmp_2[localB->minnanb];
    }

    if (jtype->size[1] <= 5) {
      localB->loop_ub_dr = jtype->size[1];
      for (localB->minnanb = 0; localB->minnanb < 5; localB->minnanb++) {
        localB->b_l[localB->minnanb] = tmp_2[localB->minnanb];
      }

      localB->b_bool_e = false;
      localB->minnanb = jtype->size[1];
      if (localB->minnanb >= 5) {
        localB->minnanb = 5;
      }

      guard11 = false;
      if (localB->loop_ub_dr <= localB->minnanb) {
        if (localB->minnanb < localB->loop_ub_dr) {
          localB->loop_ub_dr = localB->minnanb;
        }

        localB->minnanb = localB->loop_ub_dr - 1;
        guard11 = true;
      } else {
        if (jtype->size[1] == 5) {
          localB->minnanb = 4;
          guard11 = true;
        }
      }

      if (guard11) {
        localB->loop_ub_dr = 1;
        do {
          exitg1 = 0;
          if (localB->loop_ub_dr - 1 <= localB->minnanb) {
            localB->kstr_k = localB->loop_ub_dr - 1;
            if (tmp_3[static_cast<uint8_T>(jtype->data[localB->kstr_k]) & 127]
                != tmp_3[static_cast<int32_T>(localB->b_l[localB->kstr_k])]) {
              exitg1 = 1;
            } else {
              localB->loop_ub_dr++;
            }
          } else {
            localB->b_bool_e = true;
            exitg1 = 1;
          }
        } while (exitg1 == 0);
      }

      if (localB->b_bool_e) {
        if (jtype->size[1] == 5) {
          localB->nmatched = 1;
          localB->partial_match_size_idx_1 = 5;
          for (localB->minnanb = 0; localB->minnanb < 5; localB->minnanb++) {
            localB->b_f[localB->minnanb] = localB->c_vstr[localB->minnanb];
          }
        } else {
          if (!localB->matched) {
            localB->partial_match_size_idx_1 = 5;
            for (localB->minnanb = 0; localB->minnanb < 5; localB->minnanb++) {
              localB->partial_match_data[localB->minnanb] = localB->
                c_vstr[localB->minnanb];
            }
          }

          localB->nmatched++;
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
    if (localB->nmatched == 0) {
      localB->partial_match_size_idx_1 = 0;
    } else {
      localB->loop_ub_dr = localB->partial_match_size_idx_1 - 1;
      if (0 <= localB->loop_ub_dr) {
        memcpy(&localB->b_f[0], &localB->partial_match_data[0],
               (localB->loop_ub_dr + 1) * sizeof(char_T));
      }
    }
  }

  if ((localB->nmatched == 0) || ((jtype->size[1] == 0) !=
       (localB->partial_match_size_idx_1 == 0))) {
    localB->partial_match_size_idx_1 = 0;
  } else {
    localB->loop_ub_dr = localB->partial_match_size_idx_1 - 1;
    if (0 <= localB->loop_ub_dr) {
      memcpy(&localB->partial_match_data[0], &localB->b_f[0],
             (localB->loop_ub_dr + 1) * sizeof(char_T));
    }
  }

  localB->minnanb = obj->Type->size[0] * obj->Type->size[1];
  obj->Type->size[0] = 1;
  obj->Type->size[1] = localB->partial_match_size_idx_1;
  cartes_emxEnsureCapacity_char_T(obj->Type, localB->minnanb, localB);
  localB->loop_ub_dr = localB->partial_match_size_idx_1 - 1;
  for (localB->minnanb = 0; localB->minnanb <= localB->loop_ub_dr;
       localB->minnanb++) {
    obj->Type->data[localB->minnanb] = localB->partial_match_data
      [localB->minnanb];
  }

  cartesian_trajec_emxInit_char_T(&switch_expression, 2, localB);
  localB->minnanb = switch_expression->size[0] * switch_expression->size[1];
  switch_expression->size[0] = 1;
  switch_expression->size[1] = obj->Type->size[1];
  cartes_emxEnsureCapacity_char_T(switch_expression, localB->minnanb, localB);
  localB->loop_ub_dr = obj->Type->size[0] * obj->Type->size[1] - 1;
  for (localB->minnanb = 0; localB->minnanb <= localB->loop_ub_dr;
       localB->minnanb++) {
    switch_expression->data[localB->minnanb] = obj->Type->data[localB->minnanb];
  }

  for (localB->minnanb = 0; localB->minnanb < 8; localB->minnanb++) {
    localB->b_n[localB->minnanb] = tmp_0[localB->minnanb];
  }

  localB->b_bool_e = false;
  if (switch_expression->size[1] == 8) {
    localB->loop_ub_dr = 1;
    do {
      exitg1 = 0;
      if (localB->loop_ub_dr - 1 < 8) {
        localB->kstr_k = localB->loop_ub_dr - 1;
        if (switch_expression->data[localB->kstr_k] != localB->b_n
            [localB->kstr_k]) {
          exitg1 = 1;
        } else {
          localB->loop_ub_dr++;
        }
      } else {
        localB->b_bool_e = true;
        exitg1 = 1;
      }
    } while (exitg1 == 0);
  }

  if (localB->b_bool_e) {
    localB->minnanb = 0;
  } else {
    for (localB->minnanb = 0; localB->minnanb < 9; localB->minnanb++) {
      localB->b_f[localB->minnanb] = tmp_1[localB->minnanb];
    }

    localB->b_bool_e = false;
    if (switch_expression->size[1] == 9) {
      localB->loop_ub_dr = 1;
      do {
        exitg1 = 0;
        if (localB->loop_ub_dr - 1 < 9) {
          localB->kstr_k = localB->loop_ub_dr - 1;
          if (switch_expression->data[localB->kstr_k] != localB->b_f
              [localB->kstr_k]) {
            exitg1 = 1;
          } else {
            localB->loop_ub_dr++;
          }
        } else {
          localB->b_bool_e = true;
          exitg1 = 1;
        }
      } while (exitg1 == 0);
    }

    if (localB->b_bool_e) {
      localB->minnanb = 1;
    } else {
      localB->minnanb = -1;
    }
  }

  cartesian_trajec_emxFree_char_T(&switch_expression);
  switch (localB->minnanb) {
   case 0:
    localB->iv3[0] = 0;
    localB->iv3[1] = 0;
    localB->iv3[2] = 1;
    localB->iv3[3] = 0;
    localB->iv3[4] = 0;
    localB->iv3[5] = 0;
    for (localB->minnanb = 0; localB->minnanb < 6; localB->minnanb++) {
      localB->msubspace_data_af[localB->minnanb] = localB->iv3[localB->minnanb];
    }

    localB->poslim_data_p[0] = -3.1415926535897931;
    localB->poslim_data_p[1] = 3.1415926535897931;
    obj->VelocityNumber = 1.0;
    obj->PositionNumber = 1.0;
    obj->JointAxisInternal[0] = 0.0;
    obj->JointAxisInternal[1] = 0.0;
    obj->JointAxisInternal[2] = 1.0;
    break;

   case 1:
    localB->iv3[0] = 0;
    localB->iv3[1] = 0;
    localB->iv3[2] = 0;
    localB->iv3[3] = 0;
    localB->iv3[4] = 0;
    localB->iv3[5] = 1;
    for (localB->minnanb = 0; localB->minnanb < 6; localB->minnanb++) {
      localB->msubspace_data_af[localB->minnanb] = localB->iv3[localB->minnanb];
    }

    localB->poslim_data_p[0] = -0.5;
    localB->poslim_data_p[1] = 0.5;
    obj->VelocityNumber = 1.0;
    obj->PositionNumber = 1.0;
    obj->JointAxisInternal[0] = 0.0;
    obj->JointAxisInternal[1] = 0.0;
    obj->JointAxisInternal[2] = 1.0;
    break;

   default:
    for (localB->minnanb = 0; localB->minnanb < 6; localB->minnanb++) {
      localB->msubspace_data_af[localB->minnanb] = 0;
    }

    localB->poslim_data_p[0] = 0.0;
    localB->poslim_data_p[1] = 0.0;
    obj->VelocityNumber = 0.0;
    obj->PositionNumber = 0.0;
    obj->JointAxisInternal[0] = 0.0;
    obj->JointAxisInternal[1] = 0.0;
    obj->JointAxisInternal[2] = 0.0;
    break;
  }

  localB->minnanb = obj->MotionSubspace->size[0] * obj->MotionSubspace->size[1];
  obj->MotionSubspace->size[0] = 6;
  obj->MotionSubspace->size[1] = 1;
  cartes_emxEnsureCapacity_real_T(obj->MotionSubspace, localB->minnanb, localB);
  for (localB->minnanb = 0; localB->minnanb < 6; localB->minnanb++) {
    obj->MotionSubspace->data[localB->minnanb] = localB->
      msubspace_data_af[localB->minnanb];
  }

  localB->minnanb = obj->PositionLimitsInternal->size[0] *
    obj->PositionLimitsInternal->size[1];
  obj->PositionLimitsInternal->size[0] = 1;
  obj->PositionLimitsInternal->size[1] = 2;
  cartes_emxEnsureCapacity_real_T(obj->PositionLimitsInternal, localB->minnanb,
    localB);
  for (localB->minnanb = 0; localB->minnanb < 2; localB->minnanb++) {
    obj->PositionLimitsInternal->data[localB->minnanb] = localB->
      poslim_data_p[localB->minnanb];
  }

  localB->minnanb = obj->HomePositionInternal->size[0];
  obj->HomePositionInternal->size[0] = 1;
  cartes_emxEnsureCapacity_real_T(obj->HomePositionInternal, localB->minnanb,
    localB);
  for (localB->minnanb = 0; localB->minnanb < 1; localB->minnanb++) {
    obj->HomePositionInternal->data[0] = 0.0;
  }

  return b_obj;
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

static w_robotics_manip_internal_Rig_T *cartesian_trajec_RigidBody_copy(const
  v_robotics_manip_internal_Rig_T *obj, c_rigidBodyJoint_cartesian_tr_T *iobj_0,
  c_rigidBodyJoint_cartesian_tr_T *iobj_1, w_robotics_manip_internal_Rig_T
  *iobj_2, B_MATLABSystem_cartesian_traj_T *localB)
{
  w_robotics_manip_internal_Rig_T *newbody;
  c_rigidBodyJoint_cartesian_tr_T *newjoint;
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
  cartesian_trajec_emxInit_char_T(&jtype, 2, localB);
  localB->b_kstr_jz = jtype->size[0] * jtype->size[1];
  jtype->size[0] = 1;
  jtype->size[1] = obj->NameInternal->size[1];
  cartes_emxEnsureCapacity_char_T(jtype, localB->b_kstr_jz, localB);
  localB->loop_ub_l = obj->NameInternal->size[0] * obj->NameInternal->size[1] -
    1;
  for (localB->b_kstr_jz = 0; localB->b_kstr_jz <= localB->loop_ub_l;
       localB->b_kstr_jz++) {
    jtype->data[localB->b_kstr_jz] = obj->NameInternal->data[localB->b_kstr_jz];
  }

  newbody = iobj_2;
  localB->b_kstr_jz = iobj_2->NameInternal->size[0] * iobj_2->NameInternal->
    size[1];
  iobj_2->NameInternal->size[0] = 1;
  iobj_2->NameInternal->size[1] = jtype->size[1];
  cartes_emxEnsureCapacity_char_T(iobj_2->NameInternal, localB->b_kstr_jz,
    localB);
  localB->loop_ub_l = jtype->size[0] * jtype->size[1] - 1;
  for (localB->b_kstr_jz = 0; localB->b_kstr_jz <= localB->loop_ub_l;
       localB->b_kstr_jz++) {
    iobj_2->NameInternal->data[localB->b_kstr_jz] = jtype->data
      [localB->b_kstr_jz];
  }

  iobj_0->InTree = false;
  for (localB->b_kstr_jz = 0; localB->b_kstr_jz < 16; localB->b_kstr_jz++) {
    iobj_0->JointToParentTransform[localB->b_kstr_jz] = tmp[localB->b_kstr_jz];
  }

  for (localB->b_kstr_jz = 0; localB->b_kstr_jz < 16; localB->b_kstr_jz++) {
    iobj_0->ChildToJointTransform[localB->b_kstr_jz] = tmp[localB->b_kstr_jz];
  }

  localB->b_kstr_jz = iobj_0->NameInternal->size[0] * iobj_0->NameInternal->
    size[1];
  iobj_0->NameInternal->size[0] = 1;
  iobj_0->NameInternal->size[1] = jtype->size[1] + 4;
  cartes_emxEnsureCapacity_char_T(iobj_0->NameInternal, localB->b_kstr_jz,
    localB);
  localB->loop_ub_l = jtype->size[0] * jtype->size[1];
  for (localB->b_kstr_jz = 0; localB->b_kstr_jz < localB->loop_ub_l;
       localB->b_kstr_jz++) {
    iobj_0->NameInternal->data[localB->b_kstr_jz] = jtype->data
      [localB->b_kstr_jz];
  }

  iobj_0->NameInternal->data[localB->loop_ub_l] = '_';
  iobj_0->NameInternal->data[localB->loop_ub_l + 1] = 'j';
  iobj_0->NameInternal->data[localB->loop_ub_l + 2] = 'n';
  iobj_0->NameInternal->data[localB->loop_ub_l + 3] = 't';
  localB->b_kstr_jz = iobj_0->Type->size[0] * iobj_0->Type->size[1];
  iobj_0->Type->size[0] = 1;
  iobj_0->Type->size[1] = 5;
  cartes_emxEnsureCapacity_char_T(iobj_0->Type, localB->b_kstr_jz, localB);
  for (localB->b_kstr_jz = 0; localB->b_kstr_jz < 5; localB->b_kstr_jz++) {
    iobj_0->Type->data[localB->b_kstr_jz] = tmp_0[localB->b_kstr_jz];
  }

  localB->b_kstr_jz = jtype->size[0] * jtype->size[1];
  jtype->size[0] = 1;
  jtype->size[1] = iobj_0->Type->size[1];
  cartes_emxEnsureCapacity_char_T(jtype, localB->b_kstr_jz, localB);
  localB->loop_ub_l = iobj_0->Type->size[0] * iobj_0->Type->size[1] - 1;
  for (localB->b_kstr_jz = 0; localB->b_kstr_jz <= localB->loop_ub_l;
       localB->b_kstr_jz++) {
    jtype->data[localB->b_kstr_jz] = iobj_0->Type->data[localB->b_kstr_jz];
  }

  for (localB->b_kstr_jz = 0; localB->b_kstr_jz < 8; localB->b_kstr_jz++) {
    localB->b_ch[localB->b_kstr_jz] = tmp_1[localB->b_kstr_jz];
  }

  localB->b_bool_g = false;
  if (jtype->size[1] == 8) {
    localB->b_kstr_jz = 1;
    do {
      exitg1 = 0;
      if (localB->b_kstr_jz - 1 < 8) {
        localB->loop_ub_l = localB->b_kstr_jz - 1;
        if (jtype->data[localB->loop_ub_l] != localB->b_ch[localB->loop_ub_l]) {
          exitg1 = 1;
        } else {
          localB->b_kstr_jz++;
        }
      } else {
        localB->b_bool_g = true;
        exitg1 = 1;
      }
    } while (exitg1 == 0);
  }

  if (localB->b_bool_g) {
    localB->b_kstr_jz = 0;
  } else {
    for (localB->b_kstr_jz = 0; localB->b_kstr_jz < 9; localB->b_kstr_jz++) {
      localB->b_c[localB->b_kstr_jz] = tmp_2[localB->b_kstr_jz];
    }

    localB->b_bool_g = false;
    if (jtype->size[1] == 9) {
      localB->b_kstr_jz = 1;
      do {
        exitg1 = 0;
        if (localB->b_kstr_jz - 1 < 9) {
          localB->loop_ub_l = localB->b_kstr_jz - 1;
          if (jtype->data[localB->loop_ub_l] != localB->b_c[localB->loop_ub_l])
          {
            exitg1 = 1;
          } else {
            localB->b_kstr_jz++;
          }
        } else {
          localB->b_bool_g = true;
          exitg1 = 1;
        }
      } while (exitg1 == 0);
    }

    if (localB->b_bool_g) {
      localB->b_kstr_jz = 1;
    } else {
      localB->b_kstr_jz = -1;
    }
  }

  switch (localB->b_kstr_jz) {
   case 0:
    localB->iv2[0] = 0;
    localB->iv2[1] = 0;
    localB->iv2[2] = 1;
    localB->iv2[3] = 0;
    localB->iv2[4] = 0;
    localB->iv2[5] = 0;
    for (localB->b_kstr_jz = 0; localB->b_kstr_jz < 6; localB->b_kstr_jz++) {
      localB->msubspace_data_d[localB->b_kstr_jz] = localB->iv2
        [localB->b_kstr_jz];
    }

    localB->poslim_data_ct[0] = -3.1415926535897931;
    localB->poslim_data_ct[1] = 3.1415926535897931;
    iobj_0->VelocityNumber = 1.0;
    iobj_0->PositionNumber = 1.0;
    iobj_0->JointAxisInternal[0] = 0.0;
    iobj_0->JointAxisInternal[1] = 0.0;
    iobj_0->JointAxisInternal[2] = 1.0;
    break;

   case 1:
    localB->iv2[0] = 0;
    localB->iv2[1] = 0;
    localB->iv2[2] = 0;
    localB->iv2[3] = 0;
    localB->iv2[4] = 0;
    localB->iv2[5] = 1;
    for (localB->b_kstr_jz = 0; localB->b_kstr_jz < 6; localB->b_kstr_jz++) {
      localB->msubspace_data_d[localB->b_kstr_jz] = localB->iv2
        [localB->b_kstr_jz];
    }

    localB->poslim_data_ct[0] = -0.5;
    localB->poslim_data_ct[1] = 0.5;
    iobj_0->VelocityNumber = 1.0;
    iobj_0->PositionNumber = 1.0;
    iobj_0->JointAxisInternal[0] = 0.0;
    iobj_0->JointAxisInternal[1] = 0.0;
    iobj_0->JointAxisInternal[2] = 1.0;
    break;

   default:
    for (localB->b_kstr_jz = 0; localB->b_kstr_jz < 6; localB->b_kstr_jz++) {
      localB->msubspace_data_d[localB->b_kstr_jz] = 0;
    }

    localB->poslim_data_ct[0] = 0.0;
    localB->poslim_data_ct[1] = 0.0;
    iobj_0->VelocityNumber = 0.0;
    iobj_0->PositionNumber = 0.0;
    iobj_0->JointAxisInternal[0] = 0.0;
    iobj_0->JointAxisInternal[1] = 0.0;
    iobj_0->JointAxisInternal[2] = 0.0;
    break;
  }

  localB->b_kstr_jz = iobj_0->MotionSubspace->size[0] * iobj_0->
    MotionSubspace->size[1];
  iobj_0->MotionSubspace->size[0] = 6;
  iobj_0->MotionSubspace->size[1] = 1;
  cartes_emxEnsureCapacity_real_T(iobj_0->MotionSubspace, localB->b_kstr_jz,
    localB);
  for (localB->b_kstr_jz = 0; localB->b_kstr_jz < 6; localB->b_kstr_jz++) {
    iobj_0->MotionSubspace->data[localB->b_kstr_jz] = localB->
      msubspace_data_d[localB->b_kstr_jz];
  }

  localB->b_kstr_jz = iobj_0->PositionLimitsInternal->size[0] *
    iobj_0->PositionLimitsInternal->size[1];
  iobj_0->PositionLimitsInternal->size[0] = 1;
  iobj_0->PositionLimitsInternal->size[1] = 2;
  cartes_emxEnsureCapacity_real_T(iobj_0->PositionLimitsInternal,
    localB->b_kstr_jz, localB);
  for (localB->b_kstr_jz = 0; localB->b_kstr_jz < 2; localB->b_kstr_jz++) {
    iobj_0->PositionLimitsInternal->data[localB->b_kstr_jz] =
      localB->poslim_data_ct[localB->b_kstr_jz];
  }

  localB->b_kstr_jz = iobj_0->HomePositionInternal->size[0];
  iobj_0->HomePositionInternal->size[0] = 1;
  cartes_emxEnsureCapacity_real_T(iobj_0->HomePositionInternal,
    localB->b_kstr_jz, localB);
  for (localB->b_kstr_jz = 0; localB->b_kstr_jz < 1; localB->b_kstr_jz++) {
    iobj_0->HomePositionInternal->data[0] = 0.0;
  }

  iobj_2->JointInternal = iobj_0;
  iobj_2->Index = -1.0;
  iobj_2->ParentIndex = -1.0;
  localB->b_kstr_jz = jtype->size[0] * jtype->size[1];
  jtype->size[0] = 1;
  jtype->size[1] = obj->JointInternal.Type->size[1];
  cartes_emxEnsureCapacity_char_T(jtype, localB->b_kstr_jz, localB);
  localB->loop_ub_l = obj->JointInternal.Type->size[0] * obj->
    JointInternal.Type->size[1] - 1;
  for (localB->b_kstr_jz = 0; localB->b_kstr_jz <= localB->loop_ub_l;
       localB->b_kstr_jz++) {
    jtype->data[localB->b_kstr_jz] = obj->JointInternal.Type->data
      [localB->b_kstr_jz];
  }

  cartesian_trajec_emxInit_char_T(&jname, 2, localB);
  localB->b_kstr_jz = jname->size[0] * jname->size[1];
  jname->size[0] = 1;
  jname->size[1] = obj->JointInternal.NameInternal->size[1];
  cartes_emxEnsureCapacity_char_T(jname, localB->b_kstr_jz, localB);
  localB->loop_ub_l = obj->JointInternal.NameInternal->size[0] *
    obj->JointInternal.NameInternal->size[1] - 1;
  for (localB->b_kstr_jz = 0; localB->b_kstr_jz <= localB->loop_ub_l;
       localB->b_kstr_jz++) {
    jname->data[localB->b_kstr_jz] = obj->JointInternal.NameInternal->
      data[localB->b_kstr_jz];
  }

  newjoint = c_rigidBodyJoint_rigidBodyJoint(iobj_1, jname, jtype, localB);
  localB->b_kstr_jz = jtype->size[0] * jtype->size[1];
  jtype->size[0] = 1;
  jtype->size[1] = obj->JointInternal.NameInternal->size[1];
  cartes_emxEnsureCapacity_char_T(jtype, localB->b_kstr_jz, localB);
  localB->loop_ub_l = obj->JointInternal.NameInternal->size[0] *
    obj->JointInternal.NameInternal->size[1] - 1;
  cartesian_trajec_emxFree_char_T(&jname);
  for (localB->b_kstr_jz = 0; localB->b_kstr_jz <= localB->loop_ub_l;
       localB->b_kstr_jz++) {
    jtype->data[localB->b_kstr_jz] = obj->JointInternal.NameInternal->
      data[localB->b_kstr_jz];
  }

  if (jtype->size[1] != 0) {
    localB->b_kstr_jz = jtype->size[0] * jtype->size[1];
    jtype->size[0] = 1;
    jtype->size[1] = obj->JointInternal.NameInternal->size[1];
    cartes_emxEnsureCapacity_char_T(jtype, localB->b_kstr_jz, localB);
    localB->loop_ub_l = obj->JointInternal.NameInternal->size[0] *
      obj->JointInternal.NameInternal->size[1] - 1;
    for (localB->b_kstr_jz = 0; localB->b_kstr_jz <= localB->loop_ub_l;
         localB->b_kstr_jz++) {
      jtype->data[localB->b_kstr_jz] = obj->JointInternal.NameInternal->
        data[localB->b_kstr_jz];
    }

    if (!newjoint->InTree) {
      localB->b_kstr_jz = newjoint->NameInternal->size[0] *
        newjoint->NameInternal->size[1];
      newjoint->NameInternal->size[0] = 1;
      newjoint->NameInternal->size[1] = jtype->size[1];
      cartes_emxEnsureCapacity_char_T(newjoint->NameInternal, localB->b_kstr_jz,
        localB);
      localB->loop_ub_l = jtype->size[0] * jtype->size[1] - 1;
      for (localB->b_kstr_jz = 0; localB->b_kstr_jz <= localB->loop_ub_l;
           localB->b_kstr_jz++) {
        newjoint->NameInternal->data[localB->b_kstr_jz] = jtype->data
          [localB->b_kstr_jz];
      }
    }
  }

  cartesian_trajec_emxFree_char_T(&jtype);
  cartesian_trajec_emxInit_real_T(&obj_0, 1, localB);
  localB->loop_ub_l = obj->JointInternal.PositionLimitsInternal->size[0] *
    obj->JointInternal.PositionLimitsInternal->size[1];
  localB->b_kstr_jz = newjoint->PositionLimitsInternal->size[0] *
    newjoint->PositionLimitsInternal->size[1];
  newjoint->PositionLimitsInternal->size[0] =
    obj->JointInternal.PositionLimitsInternal->size[0];
  newjoint->PositionLimitsInternal->size[1] = 2;
  cartes_emxEnsureCapacity_real_T(newjoint->PositionLimitsInternal,
    localB->b_kstr_jz, localB);
  localB->b_kstr_jz = obj_0->size[0];
  obj_0->size[0] = localB->loop_ub_l;
  cartes_emxEnsureCapacity_real_T(obj_0, localB->b_kstr_jz, localB);
  for (localB->b_kstr_jz = 0; localB->b_kstr_jz < localB->loop_ub_l;
       localB->b_kstr_jz++) {
    obj_0->data[localB->b_kstr_jz] = obj->
      JointInternal.PositionLimitsInternal->data[localB->b_kstr_jz];
  }

  localB->loop_ub_l = obj_0->size[0];
  for (localB->b_kstr_jz = 0; localB->b_kstr_jz < localB->loop_ub_l;
       localB->b_kstr_jz++) {
    newjoint->PositionLimitsInternal->data[localB->b_kstr_jz] = obj_0->
      data[localB->b_kstr_jz];
  }

  cartesian_trajec_emxFree_real_T(&obj_0);
  cartesian_trajec_emxInit_real_T(&obj_1, 1, localB);
  localB->b_kstr_jz = obj_1->size[0];
  obj_1->size[0] = obj->JointInternal.HomePositionInternal->size[0];
  cartes_emxEnsureCapacity_real_T(obj_1, localB->b_kstr_jz, localB);
  localB->loop_ub_l = obj->JointInternal.HomePositionInternal->size[0];
  for (localB->b_kstr_jz = 0; localB->b_kstr_jz < localB->loop_ub_l;
       localB->b_kstr_jz++) {
    obj_1->data[localB->b_kstr_jz] = obj->
      JointInternal.HomePositionInternal->data[localB->b_kstr_jz];
  }

  localB->b_kstr_jz = newjoint->HomePositionInternal->size[0];
  newjoint->HomePositionInternal->size[0] = obj_1->size[0];
  cartes_emxEnsureCapacity_real_T(newjoint->HomePositionInternal,
    localB->b_kstr_jz, localB);
  localB->loop_ub_l = obj_1->size[0];
  for (localB->b_kstr_jz = 0; localB->b_kstr_jz < localB->loop_ub_l;
       localB->b_kstr_jz++) {
    newjoint->HomePositionInternal->data[localB->b_kstr_jz] = obj_1->data
      [localB->b_kstr_jz];
  }

  cartesian_trajec_emxFree_real_T(&obj_1);
  localB->obj_idx_0 = obj->JointInternal.JointAxisInternal[0];
  localB->obj_idx_1 = obj->JointInternal.JointAxisInternal[1];
  localB->obj_idx_2 = obj->JointInternal.JointAxisInternal[2];
  newjoint->JointAxisInternal[0] = localB->obj_idx_0;
  newjoint->JointAxisInternal[1] = localB->obj_idx_1;
  newjoint->JointAxisInternal[2] = localB->obj_idx_2;
  cartesian_trajec_emxInit_real_T(&obj_2, 1, localB);
  localB->loop_ub_l = obj->JointInternal.MotionSubspace->size[0] *
    obj->JointInternal.MotionSubspace->size[1];
  localB->b_kstr_jz = newjoint->MotionSubspace->size[0] *
    newjoint->MotionSubspace->size[1];
  newjoint->MotionSubspace->size[0] = 6;
  newjoint->MotionSubspace->size[1] = obj->JointInternal.MotionSubspace->size[1];
  cartes_emxEnsureCapacity_real_T(newjoint->MotionSubspace, localB->b_kstr_jz,
    localB);
  localB->b_kstr_jz = obj_2->size[0];
  obj_2->size[0] = localB->loop_ub_l;
  cartes_emxEnsureCapacity_real_T(obj_2, localB->b_kstr_jz, localB);
  for (localB->b_kstr_jz = 0; localB->b_kstr_jz < localB->loop_ub_l;
       localB->b_kstr_jz++) {
    obj_2->data[localB->b_kstr_jz] = obj->JointInternal.MotionSubspace->
      data[localB->b_kstr_jz];
  }

  localB->loop_ub_l = obj_2->size[0];
  for (localB->b_kstr_jz = 0; localB->b_kstr_jz < localB->loop_ub_l;
       localB->b_kstr_jz++) {
    newjoint->MotionSubspace->data[localB->b_kstr_jz] = obj_2->data
      [localB->b_kstr_jz];
  }

  cartesian_trajec_emxFree_real_T(&obj_2);
  for (localB->b_kstr_jz = 0; localB->b_kstr_jz < 16; localB->b_kstr_jz++) {
    localB->obj[localB->b_kstr_jz] = obj->
      JointInternal.JointToParentTransform[localB->b_kstr_jz];
  }

  for (localB->b_kstr_jz = 0; localB->b_kstr_jz < 16; localB->b_kstr_jz++) {
    newjoint->JointToParentTransform[localB->b_kstr_jz] = localB->obj
      [localB->b_kstr_jz];
  }

  for (localB->b_kstr_jz = 0; localB->b_kstr_jz < 16; localB->b_kstr_jz++) {
    localB->obj[localB->b_kstr_jz] = obj->
      JointInternal.ChildToJointTransform[localB->b_kstr_jz];
  }

  for (localB->b_kstr_jz = 0; localB->b_kstr_jz < 16; localB->b_kstr_jz++) {
    newjoint->ChildToJointTransform[localB->b_kstr_jz] = localB->obj
      [localB->b_kstr_jz];
  }

  iobj_2->JointInternal = newjoint;
  return newbody;
}

static void cartesian_RigidBodyTree_addBody(x_robotics_manip_internal_Rig_T *obj,
  v_robotics_manip_internal_Rig_T *bodyin, const emxArray_char_T_cartesian_tra_T
  *parentName, c_rigidBodyJoint_cartesian_tr_T *iobj_0,
  c_rigidBodyJoint_cartesian_tr_T *iobj_1, w_robotics_manip_internal_Rig_T
  *iobj_2, B_MATLABSystem_cartesian_traj_T *localB)
{
  w_robotics_manip_internal_Rig_T *body;
  c_rigidBodyJoint_cartesian_tr_T *jnt;
  emxArray_char_T_cartesian_tra_T *bname;
  static const char_T tmp[5] = { 'f', 'i', 'x', 'e', 'd' };

  boolean_T exitg1;
  int32_T exitg2;
  cartesian_trajec_emxInit_char_T(&bname, 2, localB);
  localB->pid = -1.0;
  localB->b_kstr_hn = bname->size[0] * bname->size[1];
  bname->size[0] = 1;
  bname->size[1] = obj->Base.NameInternal->size[1];
  cartes_emxEnsureCapacity_char_T(bname, localB->b_kstr_hn, localB);
  localB->loop_ub_g3 = obj->Base.NameInternal->size[0] * obj->
    Base.NameInternal->size[1] - 1;
  for (localB->b_kstr_hn = 0; localB->b_kstr_hn <= localB->loop_ub_g3;
       localB->b_kstr_hn++) {
    bname->data[localB->b_kstr_hn] = obj->Base.NameInternal->data
      [localB->b_kstr_hn];
  }

  if (cartesian_trajectory_pla_strcmp(bname, parentName)) {
    localB->pid = 0.0;
  } else {
    localB->b_index_e = obj->NumBodies;
    localB->b_i_j3 = 0;
    exitg1 = false;
    while ((!exitg1) && (localB->b_i_j3 <= static_cast<int32_T>
                         (localB->b_index_e) - 1)) {
      body = obj->Bodies[localB->b_i_j3];
      localB->b_kstr_hn = bname->size[0] * bname->size[1];
      bname->size[0] = 1;
      bname->size[1] = body->NameInternal->size[1];
      cartes_emxEnsureCapacity_char_T(bname, localB->b_kstr_hn, localB);
      localB->loop_ub_g3 = body->NameInternal->size[0] * body->
        NameInternal->size[1] - 1;
      for (localB->b_kstr_hn = 0; localB->b_kstr_hn <= localB->loop_ub_g3;
           localB->b_kstr_hn++) {
        bname->data[localB->b_kstr_hn] = body->NameInternal->data
          [localB->b_kstr_hn];
      }

      if (cartesian_trajectory_pla_strcmp(bname, parentName)) {
        localB->pid = static_cast<real_T>(localB->b_i_j3) + 1.0;
        exitg1 = true;
      } else {
        localB->b_i_j3++;
      }
    }
  }

  localB->b_index_e = obj->NumBodies + 1.0;
  body = cartesian_trajec_RigidBody_copy(bodyin, iobj_1, iobj_0, iobj_2, localB);
  obj->Bodies[static_cast<int32_T>(localB->b_index_e) - 1] = body;
  body->Index = localB->b_index_e;
  body->ParentIndex = localB->pid;
  body->JointInternal->InTree = true;
  obj->NumBodies++;
  jnt = body->JointInternal;
  localB->b_kstr_hn = bname->size[0] * bname->size[1];
  bname->size[0] = 1;
  bname->size[1] = jnt->Type->size[1];
  cartes_emxEnsureCapacity_char_T(bname, localB->b_kstr_hn, localB);
  localB->loop_ub_g3 = jnt->Type->size[0] * jnt->Type->size[1] - 1;
  for (localB->b_kstr_hn = 0; localB->b_kstr_hn <= localB->loop_ub_g3;
       localB->b_kstr_hn++) {
    bname->data[localB->b_kstr_hn] = jnt->Type->data[localB->b_kstr_hn];
  }

  for (localB->b_kstr_hn = 0; localB->b_kstr_hn < 5; localB->b_kstr_hn++) {
    localB->b_ob[localB->b_kstr_hn] = tmp[localB->b_kstr_hn];
  }

  localB->b_bool_cg = false;
  if (bname->size[1] == 5) {
    localB->b_kstr_hn = 1;
    do {
      exitg2 = 0;
      if (localB->b_kstr_hn - 1 < 5) {
        localB->loop_ub_g3 = localB->b_kstr_hn - 1;
        if (bname->data[localB->loop_ub_g3] != localB->b_ob[localB->loop_ub_g3])
        {
          exitg2 = 1;
        } else {
          localB->b_kstr_hn++;
        }
      } else {
        localB->b_bool_cg = true;
        exitg2 = 1;
      }
    } while (exitg2 == 0);
  }

  cartesian_trajec_emxFree_char_T(&bname);
  if (!localB->b_bool_cg) {
    obj->NumNonFixedBodies++;
    jnt = body->JointInternal;
    localB->b_kstr_hn = static_cast<int32_T>(body->Index) - 1;
    obj->PositionDoFMap[localB->b_kstr_hn] = obj->PositionNumber + 1.0;
    obj->PositionDoFMap[localB->b_kstr_hn + 8] = obj->PositionNumber +
      jnt->PositionNumber;
    jnt = body->JointInternal;
    localB->b_kstr_hn = static_cast<int32_T>(body->Index) - 1;
    obj->VelocityDoFMap[localB->b_kstr_hn] = obj->VelocityNumber + 1.0;
    obj->VelocityDoFMap[localB->b_kstr_hn + 8] = obj->VelocityNumber +
      jnt->VelocityNumber;
  } else {
    localB->b_kstr_hn = static_cast<int32_T>(body->Index);
    obj->PositionDoFMap[localB->b_kstr_hn - 1] = 0.0;
    obj->PositionDoFMap[localB->b_kstr_hn + 7] = -1.0;
    localB->b_kstr_hn = static_cast<int32_T>(body->Index);
    obj->VelocityDoFMap[localB->b_kstr_hn - 1] = 0.0;
    obj->VelocityDoFMap[localB->b_kstr_hn + 7] = -1.0;
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
  c_rigidBodyJoint_cartesian_tr_T *iobj_15, c_rigidBodyJoint_cartesian_tr_T
  *iobj_16, c_rigidBodyJoint_cartesian_tr_T *iobj_17,
  c_rigidBodyJoint_cartesian_tr_T *iobj_18, c_rigidBodyJoint_cartesian_tr_T
  *iobj_19, c_rigidBodyJoint_cartesian_tr_T *iobj_20,
  c_rigidBodyJoint_cartesian_tr_T *iobj_21, c_rigidBodyJoint_cartesian_tr_T
  *iobj_22, c_rigidBodyJoint_cartesian_tr_T *iobj_23,
  c_rigidBodyJoint_cartesian_tr_T *iobj_24, c_rigidBodyJoint_cartesian_tr_T
  *iobj_25, c_rigidBodyJoint_cartesian_tr_T *iobj_26,
  c_rigidBodyJoint_cartesian_tr_T *iobj_27, c_rigidBodyJoint_cartesian_tr_T
  *iobj_28, c_rigidBodyJoint_cartesian_tr_T *iobj_29,
  c_rigidBodyJoint_cartesian_tr_T *iobj_30, c_rigidBodyJoint_cartesian_tr_T
  *iobj_31, c_rigidBodyJoint_cartesian_tr_T *iobj_32,
  c_rigidBodyJoint_cartesian_tr_T *iobj_33, c_rigidBodyJoint_cartesian_tr_T
  *iobj_34, c_rigidBodyJoint_cartesian_tr_T *iobj_35,
  c_rigidBodyJoint_cartesian_tr_T *iobj_36, c_rigidBodyJoint_cartesian_tr_T
  *iobj_37, c_rigidBodyJoint_cartesian_tr_T *iobj_38,
  c_rigidBodyJoint_cartesian_tr_T *iobj_39, w_robotics_manip_internal_Rig_T
  *iobj_40, x_robotics_manip_internal_Rig_T *iobj_41,
  B_MATLABSystem_cartesian_traj_T *localB, DW_MATLABSystem_cartesian_tra_T
  *localDW)
{
  x_robotics_manip_internal_Rig_T *newrobot;
  v_robotics_manip_internal_Rig_T *body;
  v_robotics_manip_internal_Rig_T *parent;
  emxArray_char_T_cartesian_tra_T *b_basename;
  w_robotics_manip_internal_Rig_T *body_0;
  c_rigidBodyJoint_cartesian_tr_T *jnt;
  emxArray_char_T_cartesian_tra_T *bname;
  static const char_T tmp[5] = { 'f', 'i', 'x', 'e', 'd' };

  boolean_T exitg1;
  int32_T exitg2;
  cartesian_trajec_emxInit_char_T(&b_basename, 2, localB);
  newrobot = c_RigidBodyTree_RigidBodyTree_i(iobj_41, iobj_0, iobj_1, iobj_2,
    iobj_3, iobj_4, iobj_5, iobj_6, iobj_15, iobj_16, iobj_17, iobj_18, iobj_19,
    iobj_20, iobj_21, iobj_22, iobj_39, iobj_40, localB, localDW);
  localB->b_kstr_e = b_basename->size[0] * b_basename->size[1];
  b_basename->size[0] = 1;
  b_basename->size[1] = rigidbodytree->Base.NameInternal->size[1];
  cartes_emxEnsureCapacity_char_T(b_basename, localB->b_kstr_e, localB);
  localB->loop_ub_h = rigidbodytree->Base.NameInternal->size[0] *
    rigidbodytree->Base.NameInternal->size[1] - 1;
  for (localB->b_kstr_e = 0; localB->b_kstr_e <= localB->loop_ub_h;
       localB->b_kstr_e++) {
    b_basename->data[localB->b_kstr_e] = rigidbodytree->Base.NameInternal->
      data[localB->b_kstr_e];
  }

  cartesian_trajec_emxInit_char_T(&bname, 2, localB);
  localB->bid_m = -1.0;
  localB->b_kstr_e = bname->size[0] * bname->size[1];
  bname->size[0] = 1;
  bname->size[1] = newrobot->Base.NameInternal->size[1];
  cartes_emxEnsureCapacity_char_T(bname, localB->b_kstr_e, localB);
  localB->loop_ub_h = newrobot->Base.NameInternal->size[0] *
    newrobot->Base.NameInternal->size[1] - 1;
  for (localB->b_kstr_e = 0; localB->b_kstr_e <= localB->loop_ub_h;
       localB->b_kstr_e++) {
    bname->data[localB->b_kstr_e] = newrobot->Base.NameInternal->data
      [localB->b_kstr_e];
  }

  if (cartesian_trajectory_pla_strcmp(bname, b_basename)) {
    localB->bid_m = 0.0;
  } else {
    localB->b_index = newrobot->NumBodies;
    localB->b_i_h = 0;
    exitg1 = false;
    while ((!exitg1) && (localB->b_i_h <= static_cast<int32_T>(localB->b_index)
                         - 1)) {
      body_0 = newrobot->Bodies[localB->b_i_h];
      localB->b_kstr_e = bname->size[0] * bname->size[1];
      bname->size[0] = 1;
      bname->size[1] = body_0->NameInternal->size[1];
      cartes_emxEnsureCapacity_char_T(bname, localB->b_kstr_e, localB);
      localB->loop_ub_h = body_0->NameInternal->size[0] * body_0->
        NameInternal->size[1] - 1;
      for (localB->b_kstr_e = 0; localB->b_kstr_e <= localB->loop_ub_h;
           localB->b_kstr_e++) {
        bname->data[localB->b_kstr_e] = body_0->NameInternal->data
          [localB->b_kstr_e];
      }

      if (cartesian_trajectory_pla_strcmp(bname, b_basename)) {
        localB->bid_m = static_cast<real_T>(localB->b_i_h) + 1.0;
        exitg1 = true;
      } else {
        localB->b_i_h++;
      }
    }
  }

  if ((!(localB->bid_m == 0.0)) && (localB->bid_m < 0.0)) {
    localB->b_kstr_e = newrobot->Base.NameInternal->size[0] *
      newrobot->Base.NameInternal->size[1];
    newrobot->Base.NameInternal->size[0] = 1;
    newrobot->Base.NameInternal->size[1] = b_basename->size[1];
    cartes_emxEnsureCapacity_char_T(newrobot->Base.NameInternal,
      localB->b_kstr_e, localB);
    localB->loop_ub_h = b_basename->size[0] * b_basename->size[1] - 1;
    for (localB->b_kstr_e = 0; localB->b_kstr_e <= localB->loop_ub_h;
         localB->b_kstr_e++) {
      newrobot->Base.NameInternal->data[localB->b_kstr_e] = b_basename->
        data[localB->b_kstr_e];
    }
  }

  if (1.0 <= rigidbodytree->NumBodies) {
    body = rigidbodytree->Bodies[0];
    localB->bid_m = body->ParentIndex;
    if (localB->bid_m > 0.0) {
      parent = rigidbodytree->Bodies[static_cast<int32_T>(localB->bid_m) - 1];
    } else {
      parent = &rigidbodytree->Base;
    }

    localB->b_kstr_e = bname->size[0] * bname->size[1];
    bname->size[0] = 1;
    bname->size[1] = parent->NameInternal->size[1];
    cartes_emxEnsureCapacity_char_T(bname, localB->b_kstr_e, localB);
    localB->loop_ub_h = parent->NameInternal->size[0] * parent->
      NameInternal->size[1] - 1;
    for (localB->b_kstr_e = 0; localB->b_kstr_e <= localB->loop_ub_h;
         localB->b_kstr_e++) {
      bname->data[localB->b_kstr_e] = parent->NameInternal->data
        [localB->b_kstr_e];
    }

    cartesian_RigidBodyTree_addBody(newrobot, body, bname, iobj_24, iobj_23,
      iobj_7, localB);
  }

  if (2.0 <= rigidbodytree->NumBodies) {
    body = rigidbodytree->Bodies[1];
    localB->bid_m = body->ParentIndex;
    if (localB->bid_m > 0.0) {
      parent = rigidbodytree->Bodies[static_cast<int32_T>(localB->bid_m) - 1];
    } else {
      parent = &rigidbodytree->Base;
    }

    localB->b_kstr_e = bname->size[0] * bname->size[1];
    bname->size[0] = 1;
    bname->size[1] = parent->NameInternal->size[1];
    cartes_emxEnsureCapacity_char_T(bname, localB->b_kstr_e, localB);
    localB->loop_ub_h = parent->NameInternal->size[0] * parent->
      NameInternal->size[1] - 1;
    for (localB->b_kstr_e = 0; localB->b_kstr_e <= localB->loop_ub_h;
         localB->b_kstr_e++) {
      bname->data[localB->b_kstr_e] = parent->NameInternal->data
        [localB->b_kstr_e];
    }

    cartesian_RigidBodyTree_addBody(newrobot, body, bname, iobj_26, iobj_25,
      iobj_8, localB);
  }

  if (3.0 <= rigidbodytree->NumBodies) {
    body = rigidbodytree->Bodies[2];
    localB->bid_m = body->ParentIndex;
    if (localB->bid_m > 0.0) {
      parent = rigidbodytree->Bodies[static_cast<int32_T>(localB->bid_m) - 1];
    } else {
      parent = &rigidbodytree->Base;
    }

    localB->b_kstr_e = bname->size[0] * bname->size[1];
    bname->size[0] = 1;
    bname->size[1] = parent->NameInternal->size[1];
    cartes_emxEnsureCapacity_char_T(bname, localB->b_kstr_e, localB);
    localB->loop_ub_h = parent->NameInternal->size[0] * parent->
      NameInternal->size[1] - 1;
    for (localB->b_kstr_e = 0; localB->b_kstr_e <= localB->loop_ub_h;
         localB->b_kstr_e++) {
      bname->data[localB->b_kstr_e] = parent->NameInternal->data
        [localB->b_kstr_e];
    }

    cartesian_RigidBodyTree_addBody(newrobot, body, bname, iobj_28, iobj_27,
      iobj_9, localB);
  }

  if (4.0 <= rigidbodytree->NumBodies) {
    body = rigidbodytree->Bodies[3];
    localB->bid_m = body->ParentIndex;
    if (localB->bid_m > 0.0) {
      parent = rigidbodytree->Bodies[static_cast<int32_T>(localB->bid_m) - 1];
    } else {
      parent = &rigidbodytree->Base;
    }

    localB->b_kstr_e = bname->size[0] * bname->size[1];
    bname->size[0] = 1;
    bname->size[1] = parent->NameInternal->size[1];
    cartes_emxEnsureCapacity_char_T(bname, localB->b_kstr_e, localB);
    localB->loop_ub_h = parent->NameInternal->size[0] * parent->
      NameInternal->size[1] - 1;
    for (localB->b_kstr_e = 0; localB->b_kstr_e <= localB->loop_ub_h;
         localB->b_kstr_e++) {
      bname->data[localB->b_kstr_e] = parent->NameInternal->data
        [localB->b_kstr_e];
    }

    cartesian_RigidBodyTree_addBody(newrobot, body, bname, iobj_30, iobj_29,
      iobj_10, localB);
  }

  if (5.0 <= rigidbodytree->NumBodies) {
    body = rigidbodytree->Bodies[4];
    localB->bid_m = body->ParentIndex;
    if (localB->bid_m > 0.0) {
      parent = rigidbodytree->Bodies[static_cast<int32_T>(localB->bid_m) - 1];
    } else {
      parent = &rigidbodytree->Base;
    }

    localB->b_kstr_e = bname->size[0] * bname->size[1];
    bname->size[0] = 1;
    bname->size[1] = parent->NameInternal->size[1];
    cartes_emxEnsureCapacity_char_T(bname, localB->b_kstr_e, localB);
    localB->loop_ub_h = parent->NameInternal->size[0] * parent->
      NameInternal->size[1] - 1;
    for (localB->b_kstr_e = 0; localB->b_kstr_e <= localB->loop_ub_h;
         localB->b_kstr_e++) {
      bname->data[localB->b_kstr_e] = parent->NameInternal->data
        [localB->b_kstr_e];
    }

    cartesian_RigidBodyTree_addBody(newrobot, body, bname, iobj_32, iobj_31,
      iobj_11, localB);
  }

  if (6.0 <= rigidbodytree->NumBodies) {
    body = rigidbodytree->Bodies[5];
    localB->bid_m = body->ParentIndex;
    if (localB->bid_m > 0.0) {
      parent = rigidbodytree->Bodies[static_cast<int32_T>(localB->bid_m) - 1];
    } else {
      parent = &rigidbodytree->Base;
    }

    localB->b_kstr_e = bname->size[0] * bname->size[1];
    bname->size[0] = 1;
    bname->size[1] = parent->NameInternal->size[1];
    cartes_emxEnsureCapacity_char_T(bname, localB->b_kstr_e, localB);
    localB->loop_ub_h = parent->NameInternal->size[0] * parent->
      NameInternal->size[1] - 1;
    for (localB->b_kstr_e = 0; localB->b_kstr_e <= localB->loop_ub_h;
         localB->b_kstr_e++) {
      bname->data[localB->b_kstr_e] = parent->NameInternal->data
        [localB->b_kstr_e];
    }

    cartesian_RigidBodyTree_addBody(newrobot, body, bname, iobj_34, iobj_33,
      iobj_12, localB);
  }

  if (7.0 <= rigidbodytree->NumBodies) {
    body = rigidbodytree->Bodies[6];
    localB->bid_m = body->ParentIndex;
    if (localB->bid_m > 0.0) {
      parent = rigidbodytree->Bodies[static_cast<int32_T>(localB->bid_m) - 1];
    } else {
      parent = &rigidbodytree->Base;
    }

    localB->b_kstr_e = bname->size[0] * bname->size[1];
    bname->size[0] = 1;
    bname->size[1] = parent->NameInternal->size[1];
    cartes_emxEnsureCapacity_char_T(bname, localB->b_kstr_e, localB);
    localB->loop_ub_h = parent->NameInternal->size[0] * parent->
      NameInternal->size[1] - 1;
    for (localB->b_kstr_e = 0; localB->b_kstr_e <= localB->loop_ub_h;
         localB->b_kstr_e++) {
      bname->data[localB->b_kstr_e] = parent->NameInternal->data
        [localB->b_kstr_e];
    }

    localB->bid_m = -1.0;
    localB->b_kstr_e = b_basename->size[0] * b_basename->size[1];
    b_basename->size[0] = 1;
    b_basename->size[1] = newrobot->Base.NameInternal->size[1];
    cartes_emxEnsureCapacity_char_T(b_basename, localB->b_kstr_e, localB);
    localB->loop_ub_h = newrobot->Base.NameInternal->size[0] *
      newrobot->Base.NameInternal->size[1] - 1;
    for (localB->b_kstr_e = 0; localB->b_kstr_e <= localB->loop_ub_h;
         localB->b_kstr_e++) {
      b_basename->data[localB->b_kstr_e] = newrobot->Base.NameInternal->
        data[localB->b_kstr_e];
    }

    if (cartesian_trajectory_pla_strcmp(b_basename, bname)) {
      localB->bid_m = 0.0;
    } else {
      localB->b_index = newrobot->NumBodies;
      localB->b_i_h = 0;
      exitg1 = false;
      while ((!exitg1) && (localB->b_i_h <= static_cast<int32_T>(localB->b_index)
                           - 1)) {
        body_0 = newrobot->Bodies[localB->b_i_h];
        localB->b_kstr_e = b_basename->size[0] * b_basename->size[1];
        b_basename->size[0] = 1;
        b_basename->size[1] = body_0->NameInternal->size[1];
        cartes_emxEnsureCapacity_char_T(b_basename, localB->b_kstr_e, localB);
        localB->loop_ub_h = body_0->NameInternal->size[0] * body_0->
          NameInternal->size[1] - 1;
        for (localB->b_kstr_e = 0; localB->b_kstr_e <= localB->loop_ub_h;
             localB->b_kstr_e++) {
          b_basename->data[localB->b_kstr_e] = body_0->NameInternal->data
            [localB->b_kstr_e];
        }

        if (cartesian_trajectory_pla_strcmp(b_basename, bname)) {
          localB->bid_m = static_cast<real_T>(localB->b_i_h) + 1.0;
          exitg1 = true;
        } else {
          localB->b_i_h++;
        }
      }
    }

    localB->b_index = newrobot->NumBodies + 1.0;
    body_0 = cartesian_trajec_RigidBody_copy(body, iobj_35, iobj_36, iobj_13,
      localB);
    newrobot->Bodies[static_cast<int32_T>(localB->b_index) - 1] = body_0;
    body_0->Index = localB->b_index;
    body_0->ParentIndex = localB->bid_m;
    body_0->JointInternal->InTree = true;
    newrobot->NumBodies++;
    jnt = body_0->JointInternal;
    localB->b_kstr_e = bname->size[0] * bname->size[1];
    bname->size[0] = 1;
    bname->size[1] = jnt->Type->size[1];
    cartes_emxEnsureCapacity_char_T(bname, localB->b_kstr_e, localB);
    localB->loop_ub_h = jnt->Type->size[0] * jnt->Type->size[1] - 1;
    for (localB->b_kstr_e = 0; localB->b_kstr_e <= localB->loop_ub_h;
         localB->b_kstr_e++) {
      bname->data[localB->b_kstr_e] = jnt->Type->data[localB->b_kstr_e];
    }

    for (localB->b_kstr_e = 0; localB->b_kstr_e < 5; localB->b_kstr_e++) {
      localB->b_c5[localB->b_kstr_e] = tmp[localB->b_kstr_e];
    }

    localB->b_bool_c = false;
    if (bname->size[1] == 5) {
      localB->b_kstr_e = 1;
      do {
        exitg2 = 0;
        if (localB->b_kstr_e - 1 < 5) {
          localB->loop_ub_h = localB->b_kstr_e - 1;
          if (bname->data[localB->loop_ub_h] != localB->b_c5[localB->loop_ub_h])
          {
            exitg2 = 1;
          } else {
            localB->b_kstr_e++;
          }
        } else {
          localB->b_bool_c = true;
          exitg2 = 1;
        }
      } while (exitg2 == 0);
    }

    if (!localB->b_bool_c) {
      newrobot->NumNonFixedBodies++;
      jnt = body_0->JointInternal;
      localB->b_kstr_e = static_cast<int32_T>(body_0->Index) - 1;
      newrobot->PositionDoFMap[localB->b_kstr_e] = newrobot->PositionNumber +
        1.0;
      newrobot->PositionDoFMap[localB->b_kstr_e + 8] = newrobot->PositionNumber
        + jnt->PositionNumber;
      jnt = body_0->JointInternal;
      localB->b_kstr_e = static_cast<int32_T>(body_0->Index) - 1;
      newrobot->VelocityDoFMap[localB->b_kstr_e] = newrobot->VelocityNumber +
        1.0;
      newrobot->VelocityDoFMap[localB->b_kstr_e + 8] = newrobot->VelocityNumber
        + jnt->VelocityNumber;
    } else {
      localB->b_kstr_e = static_cast<int32_T>(body_0->Index);
      newrobot->PositionDoFMap[localB->b_kstr_e - 1] = 0.0;
      newrobot->PositionDoFMap[localB->b_kstr_e + 7] = -1.0;
      localB->b_kstr_e = static_cast<int32_T>(body_0->Index);
      newrobot->VelocityDoFMap[localB->b_kstr_e - 1] = 0.0;
      newrobot->VelocityDoFMap[localB->b_kstr_e + 7] = -1.0;
    }

    jnt = body_0->JointInternal;
    newrobot->PositionNumber += jnt->PositionNumber;
    jnt = body_0->JointInternal;
    newrobot->VelocityNumber += jnt->VelocityNumber;
  }

  if (8.0 <= rigidbodytree->NumBodies) {
    body = rigidbodytree->Bodies[7];
    localB->bid_m = body->ParentIndex;
    if (localB->bid_m > 0.0) {
      parent = rigidbodytree->Bodies[static_cast<int32_T>(localB->bid_m) - 1];
    } else {
      parent = &rigidbodytree->Base;
    }

    localB->b_kstr_e = bname->size[0] * bname->size[1];
    bname->size[0] = 1;
    bname->size[1] = parent->NameInternal->size[1];
    cartes_emxEnsureCapacity_char_T(bname, localB->b_kstr_e, localB);
    localB->loop_ub_h = parent->NameInternal->size[0] * parent->
      NameInternal->size[1] - 1;
    for (localB->b_kstr_e = 0; localB->b_kstr_e <= localB->loop_ub_h;
         localB->b_kstr_e++) {
      bname->data[localB->b_kstr_e] = parent->NameInternal->data
        [localB->b_kstr_e];
    }

    localB->bid_m = -1.0;
    localB->b_kstr_e = b_basename->size[0] * b_basename->size[1];
    b_basename->size[0] = 1;
    b_basename->size[1] = newrobot->Base.NameInternal->size[1];
    cartes_emxEnsureCapacity_char_T(b_basename, localB->b_kstr_e, localB);
    localB->loop_ub_h = newrobot->Base.NameInternal->size[0] *
      newrobot->Base.NameInternal->size[1] - 1;
    for (localB->b_kstr_e = 0; localB->b_kstr_e <= localB->loop_ub_h;
         localB->b_kstr_e++) {
      b_basename->data[localB->b_kstr_e] = newrobot->Base.NameInternal->
        data[localB->b_kstr_e];
    }

    if (cartesian_trajectory_pla_strcmp(b_basename, bname)) {
      localB->bid_m = 0.0;
    } else {
      localB->b_index = newrobot->NumBodies;
      localB->b_i_h = 0;
      exitg1 = false;
      while ((!exitg1) && (localB->b_i_h <= static_cast<int32_T>(localB->b_index)
                           - 1)) {
        body_0 = newrobot->Bodies[localB->b_i_h];
        localB->b_kstr_e = b_basename->size[0] * b_basename->size[1];
        b_basename->size[0] = 1;
        b_basename->size[1] = body_0->NameInternal->size[1];
        cartes_emxEnsureCapacity_char_T(b_basename, localB->b_kstr_e, localB);
        localB->loop_ub_h = body_0->NameInternal->size[0] * body_0->
          NameInternal->size[1] - 1;
        for (localB->b_kstr_e = 0; localB->b_kstr_e <= localB->loop_ub_h;
             localB->b_kstr_e++) {
          b_basename->data[localB->b_kstr_e] = body_0->NameInternal->data
            [localB->b_kstr_e];
        }

        if (cartesian_trajectory_pla_strcmp(b_basename, bname)) {
          localB->bid_m = static_cast<real_T>(localB->b_i_h) + 1.0;
          exitg1 = true;
        } else {
          localB->b_i_h++;
        }
      }
    }

    localB->b_index = newrobot->NumBodies + 1.0;
    body_0 = cartesian_trajec_RigidBody_copy(body, iobj_37, iobj_38, iobj_14,
      localB);
    newrobot->Bodies[static_cast<int32_T>(localB->b_index) - 1] = body_0;
    body_0->Index = localB->b_index;
    body_0->ParentIndex = localB->bid_m;
    body_0->JointInternal->InTree = true;
    newrobot->NumBodies++;
    jnt = body_0->JointInternal;
    localB->b_kstr_e = bname->size[0] * bname->size[1];
    bname->size[0] = 1;
    bname->size[1] = jnt->Type->size[1];
    cartes_emxEnsureCapacity_char_T(bname, localB->b_kstr_e, localB);
    localB->loop_ub_h = jnt->Type->size[0] * jnt->Type->size[1] - 1;
    for (localB->b_kstr_e = 0; localB->b_kstr_e <= localB->loop_ub_h;
         localB->b_kstr_e++) {
      bname->data[localB->b_kstr_e] = jnt->Type->data[localB->b_kstr_e];
    }

    for (localB->b_kstr_e = 0; localB->b_kstr_e < 5; localB->b_kstr_e++) {
      localB->b_c5[localB->b_kstr_e] = tmp[localB->b_kstr_e];
    }

    localB->b_bool_c = false;
    if (bname->size[1] == 5) {
      localB->b_kstr_e = 1;
      do {
        exitg2 = 0;
        if (localB->b_kstr_e - 1 < 5) {
          localB->loop_ub_h = localB->b_kstr_e - 1;
          if (bname->data[localB->loop_ub_h] != localB->b_c5[localB->loop_ub_h])
          {
            exitg2 = 1;
          } else {
            localB->b_kstr_e++;
          }
        } else {
          localB->b_bool_c = true;
          exitg2 = 1;
        }
      } while (exitg2 == 0);
    }

    if (!localB->b_bool_c) {
      newrobot->NumNonFixedBodies++;
      jnt = body_0->JointInternal;
      localB->b_kstr_e = static_cast<int32_T>(body_0->Index) - 1;
      newrobot->PositionDoFMap[localB->b_kstr_e] = newrobot->PositionNumber +
        1.0;
      newrobot->PositionDoFMap[localB->b_kstr_e + 8] = newrobot->PositionNumber
        + jnt->PositionNumber;
      jnt = body_0->JointInternal;
      localB->b_kstr_e = static_cast<int32_T>(body_0->Index) - 1;
      newrobot->VelocityDoFMap[localB->b_kstr_e] = newrobot->VelocityNumber +
        1.0;
      newrobot->VelocityDoFMap[localB->b_kstr_e + 8] = newrobot->VelocityNumber
        + jnt->VelocityNumber;
    } else {
      localB->b_kstr_e = static_cast<int32_T>(body_0->Index);
      newrobot->PositionDoFMap[localB->b_kstr_e - 1] = 0.0;
      newrobot->PositionDoFMap[localB->b_kstr_e + 7] = -1.0;
      localB->b_kstr_e = static_cast<int32_T>(body_0->Index);
      newrobot->VelocityDoFMap[localB->b_kstr_e - 1] = 0.0;
      newrobot->VelocityDoFMap[localB->b_kstr_e + 7] = -1.0;
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

static void car_inverseKinematics_setupImpl(b_inverseKinematics_cartesian_T *obj,
  f_robotics_manip_internal_IKE_T *iobj_0, B_MATLABSystem_cartesian_traj_T
  *localB)
{
  real_T n;
  emxArray_real_T_cartesian_tra_T *A;
  emxArray_real_T_cartesian_tra_T *b;
  real_T m;
  c_rigidBodyJoint_cartesian_tr_T *joint;
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
  int8_T b_I[16];
  int32_T m_0;
  real_T t;
  int32_T b_kstr;
  char_T b_0[5];
  real_T pnum_0;
  int32_T loop_ub;
  real_T tmp;
  real_T w;
  static const char_T tmp_0[5] = { 'f', 'i', 'x', 'e', 'd' };

  int32_T exitg1;
  cartesian_trajec_emxInit_real_T(&A, 2, localB);
  n = obj->RigidBodyTreeInternal->PositionNumber;
  b_kstr = A->size[0] * A->size[1];
  c = static_cast<int32_T>(n);
  A->size[0] = c;
  loop_ub = static_cast<int32_T>(2.0 * n);
  A->size[1] = loop_ub;
  cartes_emxEnsureCapacity_real_T(A, b_kstr, localB);
  m_0 = loop_ub * c - 1;
  for (b_kstr = 0; b_kstr <= m_0; b_kstr++) {
    A->data[b_kstr] = 0.0;
  }

  cartesian_trajec_emxInit_real_T(&b, 1, localB);
  b_kstr = b->size[0];
  b->size[0] = loop_ub;
  cartes_emxEnsureCapacity_real_T(b, b_kstr, localB);
  for (b_kstr = 0; b_kstr < loop_ub; b_kstr++) {
    b->data[b_kstr] = 0.0;
  }

  n = 1.0;
  m = 1.0;
  pnum = obj->RigidBodyTreeInternal->NumBodies;
  d = static_cast<int32_T>(pnum) - 1;
  cartesian_trajec_emxInit_real_T(&e, 2, localB);
  cartesian_trajec_emxInit_real_T(&s, 2, localB);
  cartesian_trajec_emxInit_char_T(&a, 2, localB);
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
    cartes_emxEnsureCapacity_char_T(a, b_kstr, localB);
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
      cartes_emxEnsureCapacity_real_T(s, b_kstr, localB);
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
      cartes_emxEnsureCapacity_real_T(s, b_kstr, localB);
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
      cartes_emxEnsureCapacity_real_T(e, b_kstr, localB);
      loop_ub = joint->PositionLimitsInternal->size[0] *
        joint->PositionLimitsInternal->size[1] - 1;
      for (b_kstr = 0; b_kstr <= loop_ub; b_kstr++) {
        e->data[b_kstr] = joint->PositionLimitsInternal->data[b_kstr];
      }

      b->data[static_cast<int32_T>(m) - 1] = e->data[1];
      b_kstr = e->size[0] * e->size[1];
      e->size[0] = joint->PositionLimitsInternal->size[0];
      e->size[1] = 2;
      cartes_emxEnsureCapacity_real_T(e, b_kstr, localB);
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
  cartes_emxEnsureCapacity_real_T(obj->Solver->ConstraintMatrix, c, localB);
  loop_ub = b_kstr - 1;
  for (b_kstr = 0; b_kstr <= loop_ub; b_kstr++) {
    obj->Solver->ConstraintMatrix->data[b_kstr] = A->data[b_kstr];
  }

  cartesian_trajec_emxFree_real_T(&A);
  b_kstr = obj->Solver->ConstraintBound->size[0];
  obj->Solver->ConstraintBound->size[0] = b->size[0];
  cartes_emxEnsureCapacity_real_T(obj->Solver->ConstraintBound, b_kstr, localB);
  loop_ub = b->size[0];
  for (b_kstr = 0; b_kstr < loop_ub; b_kstr++) {
    obj->Solver->ConstraintBound->data[b_kstr] = b->data[b_kstr];
  }

  obj_0 = obj->RigidBodyTreeInternal;
  b_kstr = e->size[0] * e->size[1];
  e->size[0] = static_cast<int32_T>(obj_0->PositionNumber);
  e->size[1] = 2;
  cartes_emxEnsureCapacity_real_T(e, b_kstr, localB);
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
    cartes_emxEnsureCapacity_char_T(a, b_kstr, localB);
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
  cartes_emxEnsureCapacity_real_T(obj->Limits, b_kstr, localB);
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
  cartes_emxEnsureCapacity_real_T(obj->Solver->ExtraArgs->ErrTemp, b_kstr,
    localB);
  for (b_kstr = 0; b_kstr < 6; b_kstr++) {
    obj->Solver->ExtraArgs->ErrTemp->data[b_kstr] = 0.0;
  }

  obj->Solver->ExtraArgs->CostTemp = 0.0;
  b_kstr = b->size[0];
  b->size[0] = static_cast<int32_T>(obj->RigidBodyTreeInternal->PositionNumber);
  cartes_emxEnsureCapacity_real_T(b, b_kstr, localB);
  loop_ub = static_cast<int32_T>(obj->RigidBodyTreeInternal->PositionNumber);
  for (b_kstr = 0; b_kstr < loop_ub; b_kstr++) {
    b->data[b_kstr] = 0.0;
  }

  b_kstr = obj->Solver->ExtraArgs->GradTemp->size[0];
  obj->Solver->ExtraArgs->GradTemp->size[0] = b->size[0];
  cartes_emxEnsureCapacity_real_T(obj->Solver->ExtraArgs->GradTemp, b_kstr,
    localB);
  loop_ub = b->size[0];
  for (b_kstr = 0; b_kstr < loop_ub; b_kstr++) {
    obj->Solver->ExtraArgs->GradTemp->data[b_kstr] = b->data[b_kstr];
  }

  cartesian_trajec_emxFree_real_T(&b);
}

static void c_inverseKinematics_setPoseGoal(b_inverseKinematics_cartesian_T *obj,
  const real_T tform[16], const real_T weights[6],
  B_MATLABSystem_cartesian_traj_T *localB)
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
  cartes_emxEnsureCapacity_char_T(args->BodyName, b_j, localB);
  for (b_j = 0; b_j < 11; b_j++) {
    args->BodyName->data[b_j] = tmp[b_j];
  }

  for (b_j = 0; b_j < 16; b_j++) {
    args->Tform[b_j] = tform[b_j];
  }
}

static void RigidBodyTree_validateConfigu_j(x_robotics_manip_internal_Rig_T *obj,
  real_T Q[6], B_MATLABSystem_cartesian_traj_T *localB)
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
  c_rigidBodyJoint_cartesian_tr_T *obj_0;
  int32_T idx;
  int32_T b_kstr;
  char_T b[5];
  int32_T loop_ub;
  emxArray_real_T_cartesian_tra_T *limits_0;
  emxArray_real_T_cartesian_tra_T *limits_1;
  static const char_T tmp[5] = { 'f', 'i', 'x', 'e', 'd' };

  boolean_T guard1 = false;
  int32_T exitg1;
  boolean_T exitg2;
  cartesian_trajec_emxInit_real_T(&limits, 2, localB);
  b_kstr = limits->size[0] * limits->size[1];
  limits->size[0] = static_cast<int32_T>(obj->PositionNumber);
  limits->size[1] = 2;
  cartes_emxEnsureCapacity_real_T(limits, b_kstr, localB);
  loop_ub = (static_cast<int32_T>(obj->PositionNumber) << 1) - 1;
  for (b_kstr = 0; b_kstr <= loop_ub; b_kstr++) {
    limits->data[b_kstr] = 0.0;
  }

  k = 1.0;
  pnum = obj->NumBodies;
  c = static_cast<int32_T>(pnum) - 1;
  cartesian_trajec_emxInit_char_T(&a, 2, localB);
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
    cartes_emxEnsureCapacity_char_T(a, b_kstr, localB);
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
  cartesian_trajec_emxInit_real_T(&limits_0, 1, localB);
  loop_ub = limits->size[0];
  b_kstr = limits_0->size[0];
  limits_0->size[0] = loop_ub;
  cartes_emxEnsureCapacity_real_T(limits_0, b_kstr, localB);
  for (b_kstr = 0; b_kstr < loop_ub; b_kstr++) {
    limits_0->data[b_kstr] = limits->data[b_kstr + limits->size[0]] +
      4.4408920985006262E-16;
  }

  for (b_kstr = 0; b_kstr < 6; b_kstr++) {
    ubOK[b_kstr] = (Q[b_kstr] <= limits_0->data[b_kstr]);
  }

  cartesian_trajec_emxFree_real_T(&limits_0);
  cartesian_trajec_emxInit_real_T(&limits_1, 1, localB);
  loop_ub = limits->size[0];
  b_kstr = limits_1->size[0];
  limits_1->size[0] = loop_ub;
  cartes_emxEnsureCapacity_real_T(limits_1, b_kstr, localB);
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

static void ca_rigidBodyJoint_get_JointAxis(const
  c_rigidBodyJoint_cartesian_tr_T *obj, real_T ax[3],
  B_MATLABSystem_cartesian_traj_T *localB)
{
  emxArray_char_T_cartesian_tra_T *a;
  static const char_T tmp[8] = { 'r', 'e', 'v', 'o', 'l', 'u', 't', 'e' };

  static const char_T tmp_0[9] = { 'p', 'r', 'i', 's', 'm', 'a', 't', 'i', 'c' };

  boolean_T guard1 = false;
  int32_T exitg1;
  cartesian_trajec_emxInit_char_T(&a, 2, localB);
  localB->b_kstr_j = a->size[0] * a->size[1];
  a->size[0] = 1;
  a->size[1] = obj->Type->size[1];
  cartes_emxEnsureCapacity_char_T(a, localB->b_kstr_j, localB);
  localB->loop_ub_o = obj->Type->size[0] * obj->Type->size[1] - 1;
  for (localB->b_kstr_j = 0; localB->b_kstr_j <= localB->loop_ub_o;
       localB->b_kstr_j++) {
    a->data[localB->b_kstr_j] = obj->Type->data[localB->b_kstr_j];
  }

  for (localB->b_kstr_j = 0; localB->b_kstr_j < 8; localB->b_kstr_j++) {
    localB->b_m4[localB->b_kstr_j] = tmp[localB->b_kstr_j];
  }

  localB->b_bool_k = false;
  if (a->size[1] == 8) {
    localB->b_kstr_j = 1;
    do {
      exitg1 = 0;
      if (localB->b_kstr_j - 1 < 8) {
        localB->loop_ub_o = localB->b_kstr_j - 1;
        if (a->data[localB->loop_ub_o] != localB->b_m4[localB->loop_ub_o]) {
          exitg1 = 1;
        } else {
          localB->b_kstr_j++;
        }
      } else {
        localB->b_bool_k = true;
        exitg1 = 1;
      }
    } while (exitg1 == 0);
  }

  guard1 = false;
  if (localB->b_bool_k) {
    guard1 = true;
  } else {
    localB->b_kstr_j = a->size[0] * a->size[1];
    a->size[0] = 1;
    a->size[1] = obj->Type->size[1];
    cartes_emxEnsureCapacity_char_T(a, localB->b_kstr_j, localB);
    localB->loop_ub_o = obj->Type->size[0] * obj->Type->size[1] - 1;
    for (localB->b_kstr_j = 0; localB->b_kstr_j <= localB->loop_ub_o;
         localB->b_kstr_j++) {
      a->data[localB->b_kstr_j] = obj->Type->data[localB->b_kstr_j];
    }

    for (localB->b_kstr_j = 0; localB->b_kstr_j < 9; localB->b_kstr_j++) {
      localB->b_i[localB->b_kstr_j] = tmp_0[localB->b_kstr_j];
    }

    localB->b_bool_k = false;
    if (a->size[1] == 9) {
      localB->b_kstr_j = 1;
      do {
        exitg1 = 0;
        if (localB->b_kstr_j - 1 < 9) {
          localB->loop_ub_o = localB->b_kstr_j - 1;
          if (a->data[localB->loop_ub_o] != localB->b_i[localB->loop_ub_o]) {
            exitg1 = 1;
          } else {
            localB->b_kstr_j++;
          }
        } else {
          localB->b_bool_k = true;
          exitg1 = 1;
        }
      } while (exitg1 == 0);
    }

    if (localB->b_bool_k) {
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

static void rigidBodyJoint_transformBodyToP(const
  c_rigidBodyJoint_cartesian_tr_T *obj, real_T T[16],
  B_MATLABSystem_cartesian_traj_T *localB)
{
  emxArray_char_T_cartesian_tra_T *switch_expression;
  static const char_T tmp[5] = { 'f', 'i', 'x', 'e', 'd' };

  static const char_T tmp_0[8] = { 'r', 'e', 'v', 'o', 'l', 'u', 't', 'e' };

  int32_T exitg1;
  cartesian_trajec_emxInit_char_T(&switch_expression, 2, localB);
  localB->b_kstr_a = switch_expression->size[0] * switch_expression->size[1];
  switch_expression->size[0] = 1;
  switch_expression->size[1] = obj->Type->size[1];
  cartes_emxEnsureCapacity_char_T(switch_expression, localB->b_kstr_a, localB);
  localB->loop_ub_fn = obj->Type->size[0] * obj->Type->size[1] - 1;
  for (localB->b_kstr_a = 0; localB->b_kstr_a <= localB->loop_ub_fn;
       localB->b_kstr_a++) {
    switch_expression->data[localB->b_kstr_a] = obj->Type->data[localB->b_kstr_a];
  }

  for (localB->b_kstr_a = 0; localB->b_kstr_a < 5; localB->b_kstr_a++) {
    localB->b_da[localB->b_kstr_a] = tmp[localB->b_kstr_a];
  }

  localB->b_bool_f = false;
  if (switch_expression->size[1] == 5) {
    localB->b_kstr_a = 1;
    do {
      exitg1 = 0;
      if (localB->b_kstr_a - 1 < 5) {
        localB->loop_ub_fn = localB->b_kstr_a - 1;
        if (switch_expression->data[localB->loop_ub_fn] != localB->b_da
            [localB->loop_ub_fn]) {
          exitg1 = 1;
        } else {
          localB->b_kstr_a++;
        }
      } else {
        localB->b_bool_f = true;
        exitg1 = 1;
      }
    } while (exitg1 == 0);
  }

  if (localB->b_bool_f) {
    localB->b_kstr_a = 0;
  } else {
    for (localB->b_kstr_a = 0; localB->b_kstr_a < 8; localB->b_kstr_a++) {
      localB->b_a[localB->b_kstr_a] = tmp_0[localB->b_kstr_a];
    }

    localB->b_bool_f = false;
    if (switch_expression->size[1] == 8) {
      localB->b_kstr_a = 1;
      do {
        exitg1 = 0;
        if (localB->b_kstr_a - 1 < 8) {
          localB->loop_ub_fn = localB->b_kstr_a - 1;
          if (switch_expression->data[localB->loop_ub_fn] != localB->b_a
              [localB->loop_ub_fn]) {
            exitg1 = 1;
          } else {
            localB->b_kstr_a++;
          }
        } else {
          localB->b_bool_f = true;
          exitg1 = 1;
        }
      } while (exitg1 == 0);
    }

    if (localB->b_bool_f) {
      localB->b_kstr_a = 1;
    } else {
      localB->b_kstr_a = -1;
    }
  }

  cartesian_trajec_emxFree_char_T(&switch_expression);
  switch (localB->b_kstr_a) {
   case 0:
    memset(&localB->TJ_l[0], 0, sizeof(real_T) << 4U);
    localB->TJ_l[0] = 1.0;
    localB->TJ_l[5] = 1.0;
    localB->TJ_l[10] = 1.0;
    localB->TJ_l[15] = 1.0;
    break;

   case 1:
    ca_rigidBodyJoint_get_JointAxis(obj, localB->v_pt, localB);
    localB->axang_idx_0 = localB->v_pt[0];
    localB->axang_idx_1 = localB->v_pt[1];
    localB->axang_idx_2 = localB->v_pt[2];
    localB->b_cj = 1.0 / sqrt((localB->axang_idx_0 * localB->axang_idx_0 +
      localB->axang_idx_1 * localB->axang_idx_1) + localB->axang_idx_2 *
      localB->axang_idx_2);
    localB->v_pt[0] = localB->axang_idx_0 * localB->b_cj;
    localB->v_pt[1] = localB->axang_idx_1 * localB->b_cj;
    localB->v_pt[2] = localB->axang_idx_2 * localB->b_cj;
    localB->axang_idx_0 = localB->v_pt[1] * localB->v_pt[0] * 0.0;
    localB->axang_idx_1 = localB->v_pt[2] * localB->v_pt[0] * 0.0;
    localB->axang_idx_2 = localB->v_pt[2] * localB->v_pt[1] * 0.0;
    cartesian_trajectory_planne_cat(localB->v_pt[0] * localB->v_pt[0] * 0.0 +
      1.0, localB->axang_idx_0 - localB->v_pt[2] * 0.0, localB->axang_idx_1 +
      localB->v_pt[1] * 0.0, localB->axang_idx_0 + localB->v_pt[2] * 0.0,
      localB->v_pt[1] * localB->v_pt[1] * 0.0 + 1.0, localB->axang_idx_2 -
      localB->v_pt[0] * 0.0, localB->axang_idx_1 - localB->v_pt[1] * 0.0,
      localB->axang_idx_2 + localB->v_pt[0] * 0.0, localB->v_pt[2] *
      localB->v_pt[2] * 0.0 + 1.0, localB->tempR_a);
    for (localB->b_kstr_a = 0; localB->b_kstr_a < 3; localB->b_kstr_a++) {
      localB->loop_ub_fn = localB->b_kstr_a + 1;
      localB->R_ax[localB->loop_ub_fn - 1] = localB->tempR_a[(localB->loop_ub_fn
        - 1) * 3];
      localB->loop_ub_fn = localB->b_kstr_a + 1;
      localB->R_ax[localB->loop_ub_fn + 2] = localB->tempR_a[(localB->loop_ub_fn
        - 1) * 3 + 1];
      localB->loop_ub_fn = localB->b_kstr_a + 1;
      localB->R_ax[localB->loop_ub_fn + 5] = localB->tempR_a[(localB->loop_ub_fn
        - 1) * 3 + 2];
    }

    memset(&localB->TJ_l[0], 0, sizeof(real_T) << 4U);
    for (localB->b_kstr_a = 0; localB->b_kstr_a < 3; localB->b_kstr_a++) {
      localB->loop_ub_fn = localB->b_kstr_a << 2;
      localB->TJ_l[localB->loop_ub_fn] = localB->R_ax[3 * localB->b_kstr_a];
      localB->TJ_l[localB->loop_ub_fn + 1] = localB->R_ax[3 * localB->b_kstr_a +
        1];
      localB->TJ_l[localB->loop_ub_fn + 2] = localB->R_ax[3 * localB->b_kstr_a +
        2];
    }

    localB->TJ_l[15] = 1.0;
    break;

   default:
    ca_rigidBodyJoint_get_JointAxis(obj, localB->v_pt, localB);
    memset(&localB->tempR_a[0], 0, 9U * sizeof(real_T));
    localB->tempR_a[0] = 1.0;
    localB->tempR_a[4] = 1.0;
    localB->tempR_a[8] = 1.0;
    for (localB->b_kstr_a = 0; localB->b_kstr_a < 3; localB->b_kstr_a++) {
      localB->loop_ub_fn = localB->b_kstr_a << 2;
      localB->TJ_l[localB->loop_ub_fn] = localB->tempR_a[3 * localB->b_kstr_a];
      localB->TJ_l[localB->loop_ub_fn + 1] = localB->tempR_a[3 *
        localB->b_kstr_a + 1];
      localB->TJ_l[localB->loop_ub_fn + 2] = localB->tempR_a[3 *
        localB->b_kstr_a + 2];
      localB->TJ_l[localB->b_kstr_a + 12] = localB->v_pt[localB->b_kstr_a] * 0.0;
    }

    localB->TJ_l[3] = 0.0;
    localB->TJ_l[7] = 0.0;
    localB->TJ_l[11] = 0.0;
    localB->TJ_l[15] = 1.0;
    break;
  }

  for (localB->b_kstr_a = 0; localB->b_kstr_a < 16; localB->b_kstr_a++) {
    localB->a_j[localB->b_kstr_a] = obj->JointToParentTransform[localB->b_kstr_a];
  }

  for (localB->b_kstr_a = 0; localB->b_kstr_a < 16; localB->b_kstr_a++) {
    localB->b_d[localB->b_kstr_a] = obj->ChildToJointTransform[localB->b_kstr_a];
  }

  for (localB->b_kstr_a = 0; localB->b_kstr_a < 4; localB->b_kstr_a++) {
    for (localB->loop_ub_fn = 0; localB->loop_ub_fn < 4; localB->loop_ub_fn++) {
      localB->a_tmp_tmp_k = localB->loop_ub_fn << 2;
      localB->a_tmp_j = localB->b_kstr_a + localB->a_tmp_tmp_k;
      localB->a_g[localB->a_tmp_j] = 0.0;
      localB->a_g[localB->a_tmp_j] += localB->TJ_l[localB->a_tmp_tmp_k] *
        localB->a_j[localB->b_kstr_a];
      localB->a_g[localB->a_tmp_j] += localB->TJ_l[localB->a_tmp_tmp_k + 1] *
        localB->a_j[localB->b_kstr_a + 4];
      localB->a_g[localB->a_tmp_j] += localB->TJ_l[localB->a_tmp_tmp_k + 2] *
        localB->a_j[localB->b_kstr_a + 8];
      localB->a_g[localB->a_tmp_j] += localB->TJ_l[localB->a_tmp_tmp_k + 3] *
        localB->a_j[localB->b_kstr_a + 12];
    }

    for (localB->loop_ub_fn = 0; localB->loop_ub_fn < 4; localB->loop_ub_fn++) {
      localB->a_tmp_tmp_k = localB->loop_ub_fn << 2;
      localB->a_tmp_j = localB->b_kstr_a + localB->a_tmp_tmp_k;
      T[localB->a_tmp_j] = 0.0;
      T[localB->a_tmp_j] += localB->b_d[localB->a_tmp_tmp_k] * localB->
        a_g[localB->b_kstr_a];
      T[localB->a_tmp_j] += localB->b_d[localB->a_tmp_tmp_k + 1] * localB->
        a_g[localB->b_kstr_a + 4];
      T[localB->a_tmp_j] += localB->b_d[localB->a_tmp_tmp_k + 2] * localB->
        a_g[localB->b_kstr_a + 8];
      T[localB->a_tmp_j] += localB->b_d[localB->a_tmp_tmp_k + 3] * localB->
        a_g[localB->b_kstr_a + 12];
    }
  }
}

static void rigidBodyJoint_transformBodyT_n(const
  c_rigidBodyJoint_cartesian_tr_T *obj, const real_T q_data[], const int32_T
  *q_size, real_T T[16], B_MATLABSystem_cartesian_traj_T *localB)
{
  emxArray_char_T_cartesian_tra_T *switch_expression;
  static const char_T tmp[5] = { 'f', 'i', 'x', 'e', 'd' };

  static const char_T tmp_0[8] = { 'r', 'e', 'v', 'o', 'l', 'u', 't', 'e' };

  int32_T exitg1;
  cartesian_trajec_emxInit_char_T(&switch_expression, 2, localB);
  localB->b_kstr_i = switch_expression->size[0] * switch_expression->size[1];
  switch_expression->size[0] = 1;
  switch_expression->size[1] = obj->Type->size[1];
  cartes_emxEnsureCapacity_char_T(switch_expression, localB->b_kstr_i, localB);
  localB->loop_ub_f = obj->Type->size[0] * obj->Type->size[1] - 1;
  for (localB->b_kstr_i = 0; localB->b_kstr_i <= localB->loop_ub_f;
       localB->b_kstr_i++) {
    switch_expression->data[localB->b_kstr_i] = obj->Type->data[localB->b_kstr_i];
  }

  for (localB->b_kstr_i = 0; localB->b_kstr_i < 5; localB->b_kstr_i++) {
    localB->b_hp[localB->b_kstr_i] = tmp[localB->b_kstr_i];
  }

  localB->b_bool_m = false;
  if (switch_expression->size[1] == 5) {
    localB->b_kstr_i = 1;
    do {
      exitg1 = 0;
      if (localB->b_kstr_i - 1 < 5) {
        localB->loop_ub_f = localB->b_kstr_i - 1;
        if (switch_expression->data[localB->loop_ub_f] != localB->b_hp
            [localB->loop_ub_f]) {
          exitg1 = 1;
        } else {
          localB->b_kstr_i++;
        }
      } else {
        localB->b_bool_m = true;
        exitg1 = 1;
      }
    } while (exitg1 == 0);
  }

  if (localB->b_bool_m) {
    localB->b_kstr_i = 0;
  } else {
    for (localB->b_kstr_i = 0; localB->b_kstr_i < 8; localB->b_kstr_i++) {
      localB->b_h[localB->b_kstr_i] = tmp_0[localB->b_kstr_i];
    }

    localB->b_bool_m = false;
    if (switch_expression->size[1] == 8) {
      localB->b_kstr_i = 1;
      do {
        exitg1 = 0;
        if (localB->b_kstr_i - 1 < 8) {
          localB->loop_ub_f = localB->b_kstr_i - 1;
          if (switch_expression->data[localB->loop_ub_f] != localB->b_h
              [localB->loop_ub_f]) {
            exitg1 = 1;
          } else {
            localB->b_kstr_i++;
          }
        } else {
          localB->b_bool_m = true;
          exitg1 = 1;
        }
      } while (exitg1 == 0);
    }

    if (localB->b_bool_m) {
      localB->b_kstr_i = 1;
    } else {
      localB->b_kstr_i = -1;
    }
  }

  cartesian_trajec_emxFree_char_T(&switch_expression);
  switch (localB->b_kstr_i) {
   case 0:
    memset(&localB->TJ[0], 0, sizeof(real_T) << 4U);
    localB->TJ[0] = 1.0;
    localB->TJ[5] = 1.0;
    localB->TJ[10] = 1.0;
    localB->TJ[15] = 1.0;
    break;

   case 1:
    ca_rigidBodyJoint_get_JointAxis(obj, localB->v_pe, localB);
    localB->loop_ub_f = (*q_size != 0) - 1;
    localB->result_data[0] = localB->v_pe[0];
    localB->result_data[1] = localB->v_pe[1];
    localB->result_data[2] = localB->v_pe[2];
    for (localB->b_kstr_i = 0; localB->b_kstr_i <= localB->loop_ub_f;
         localB->b_kstr_i++) {
      localB->result_data[3] = q_data[0];
    }

    localB->cth = 1.0 / sqrt((localB->result_data[0] * localB->result_data[0] +
      localB->result_data[1] * localB->result_data[1]) + localB->result_data[2] *
      localB->result_data[2]);
    localB->v_pe[0] = localB->result_data[0] * localB->cth;
    localB->v_pe[1] = localB->result_data[1] * localB->cth;
    localB->v_pe[2] = localB->result_data[2] * localB->cth;
    localB->cth = cos(localB->result_data[3]);
    localB->sth = sin(localB->result_data[3]);
    localB->tempR_tmp = localB->v_pe[1] * localB->v_pe[0] * (1.0 - localB->cth);
    localB->tempR_tmp_n = localB->v_pe[2] * localB->sth;
    localB->tempR_tmp_d = localB->v_pe[2] * localB->v_pe[0] * (1.0 - localB->cth);
    localB->tempR_tmp_na = localB->v_pe[1] * localB->sth;
    localB->tempR_tmp_c = localB->v_pe[2] * localB->v_pe[1] * (1.0 - localB->cth);
    localB->sth *= localB->v_pe[0];
    cartesian_trajectory_planne_cat(localB->v_pe[0] * localB->v_pe[0] * (1.0 -
      localB->cth) + localB->cth, localB->tempR_tmp - localB->tempR_tmp_n,
      localB->tempR_tmp_d + localB->tempR_tmp_na, localB->tempR_tmp +
      localB->tempR_tmp_n, localB->v_pe[1] * localB->v_pe[1] * (1.0 -
      localB->cth) + localB->cth, localB->tempR_tmp_c - localB->sth,
      localB->tempR_tmp_d - localB->tempR_tmp_na, localB->tempR_tmp_c +
      localB->sth, localB->v_pe[2] * localB->v_pe[2] * (1.0 - localB->cth) +
      localB->cth, localB->tempR);
    for (localB->b_kstr_i = 0; localB->b_kstr_i < 3; localB->b_kstr_i++) {
      localB->loop_ub_f = localB->b_kstr_i + 1;
      localB->R_e[localB->loop_ub_f - 1] = localB->tempR[(localB->loop_ub_f - 1)
        * 3];
      localB->loop_ub_f = localB->b_kstr_i + 1;
      localB->R_e[localB->loop_ub_f + 2] = localB->tempR[(localB->loop_ub_f - 1)
        * 3 + 1];
      localB->loop_ub_f = localB->b_kstr_i + 1;
      localB->R_e[localB->loop_ub_f + 5] = localB->tempR[(localB->loop_ub_f - 1)
        * 3 + 2];
    }

    memset(&localB->TJ[0], 0, sizeof(real_T) << 4U);
    for (localB->b_kstr_i = 0; localB->b_kstr_i < 3; localB->b_kstr_i++) {
      localB->loop_ub_f = localB->b_kstr_i << 2;
      localB->TJ[localB->loop_ub_f] = localB->R_e[3 * localB->b_kstr_i];
      localB->TJ[localB->loop_ub_f + 1] = localB->R_e[3 * localB->b_kstr_i + 1];
      localB->TJ[localB->loop_ub_f + 2] = localB->R_e[3 * localB->b_kstr_i + 2];
    }

    localB->TJ[15] = 1.0;
    break;

   default:
    ca_rigidBodyJoint_get_JointAxis(obj, localB->v_pe, localB);
    memset(&localB->tempR[0], 0, 9U * sizeof(real_T));
    localB->tempR[0] = 1.0;
    localB->tempR[4] = 1.0;
    localB->tempR[8] = 1.0;
    for (localB->b_kstr_i = 0; localB->b_kstr_i < 3; localB->b_kstr_i++) {
      localB->loop_ub_f = localB->b_kstr_i << 2;
      localB->TJ[localB->loop_ub_f] = localB->tempR[3 * localB->b_kstr_i];
      localB->TJ[localB->loop_ub_f + 1] = localB->tempR[3 * localB->b_kstr_i + 1];
      localB->TJ[localB->loop_ub_f + 2] = localB->tempR[3 * localB->b_kstr_i + 2];
      localB->TJ[localB->b_kstr_i + 12] = localB->v_pe[localB->b_kstr_i] *
        q_data[0];
    }

    localB->TJ[3] = 0.0;
    localB->TJ[7] = 0.0;
    localB->TJ[11] = 0.0;
    localB->TJ[15] = 1.0;
    break;
  }

  for (localB->b_kstr_i = 0; localB->b_kstr_i < 16; localB->b_kstr_i++) {
    localB->a[localB->b_kstr_i] = obj->JointToParentTransform[localB->b_kstr_i];
  }

  for (localB->b_kstr_i = 0; localB->b_kstr_i < 16; localB->b_kstr_i++) {
    localB->b[localB->b_kstr_i] = obj->ChildToJointTransform[localB->b_kstr_i];
  }

  for (localB->b_kstr_i = 0; localB->b_kstr_i < 4; localB->b_kstr_i++) {
    for (localB->loop_ub_f = 0; localB->loop_ub_f < 4; localB->loop_ub_f++) {
      localB->a_tmp_tmp = localB->loop_ub_f << 2;
      localB->a_tmp = localB->b_kstr_i + localB->a_tmp_tmp;
      localB->a_p[localB->a_tmp] = 0.0;
      localB->a_p[localB->a_tmp] += localB->TJ[localB->a_tmp_tmp] * localB->
        a[localB->b_kstr_i];
      localB->a_p[localB->a_tmp] += localB->TJ[localB->a_tmp_tmp + 1] *
        localB->a[localB->b_kstr_i + 4];
      localB->a_p[localB->a_tmp] += localB->TJ[localB->a_tmp_tmp + 2] *
        localB->a[localB->b_kstr_i + 8];
      localB->a_p[localB->a_tmp] += localB->TJ[localB->a_tmp_tmp + 3] *
        localB->a[localB->b_kstr_i + 12];
    }

    for (localB->loop_ub_f = 0; localB->loop_ub_f < 4; localB->loop_ub_f++) {
      localB->a_tmp_tmp = localB->loop_ub_f << 2;
      localB->a_tmp = localB->b_kstr_i + localB->a_tmp_tmp;
      T[localB->a_tmp] = 0.0;
      T[localB->a_tmp] += localB->b[localB->a_tmp_tmp] * localB->a_p
        [localB->b_kstr_i];
      T[localB->a_tmp] += localB->b[localB->a_tmp_tmp + 1] * localB->a_p
        [localB->b_kstr_i + 4];
      T[localB->a_tmp] += localB->b[localB->a_tmp_tmp + 2] * localB->a_p
        [localB->b_kstr_i + 8];
      T[localB->a_tmp] += localB->b[localB->a_tmp_tmp + 3] * localB->a_p
        [localB->b_kstr_i + 12];
    }
  }
}

static void RigidBodyTree_efficientFKAndJac(x_robotics_manip_internal_Rig_T *obj,
  const real_T qv[6], const emxArray_char_T_cartesian_tra_T *body1Name, real_T
  T_data[], int32_T T_size[2], emxArray_real_T_cartesian_tra_T *Jac,
  B_MATLABSystem_cartesian_traj_T *localB)
{
  w_robotics_manip_internal_Rig_T *body1;
  w_robotics_manip_internal_Rig_T *body2;
  emxArray_real_T_cartesian_tra_T *kinematicPathIndices;
  c_rigidBodyJoint_cartesian_tr_T *joint;
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
  cartesian_trajec_emxInit_char_T(&body2Name, 2, localB);
  localB->b_kstr = body2Name->size[0] * body2Name->size[1];
  body2Name->size[0] = 1;
  body2Name->size[1] = obj->Base.NameInternal->size[1];
  cartes_emxEnsureCapacity_char_T(body2Name, localB->b_kstr, localB);
  localB->loop_ub_a = obj->Base.NameInternal->size[0] * obj->
    Base.NameInternal->size[1] - 1;
  for (localB->b_kstr = 0; localB->b_kstr <= localB->loop_ub_a; localB->b_kstr++)
  {
    body2Name->data[localB->b_kstr] = obj->Base.NameInternal->data
      [localB->b_kstr];
  }

  cartesian_trajec_emxInit_char_T(&bname, 2, localB);
  localB->bid1 = -1.0;
  localB->b_kstr = bname->size[0] * bname->size[1];
  bname->size[0] = 1;
  bname->size[1] = obj->Base.NameInternal->size[1];
  cartes_emxEnsureCapacity_char_T(bname, localB->b_kstr, localB);
  localB->loop_ub_a = obj->Base.NameInternal->size[0] * obj->
    Base.NameInternal->size[1] - 1;
  for (localB->b_kstr = 0; localB->b_kstr <= localB->loop_ub_a; localB->b_kstr++)
  {
    bname->data[localB->b_kstr] = obj->Base.NameInternal->data[localB->b_kstr];
  }

  if (cartesian_trajectory_pla_strcmp(bname, body1Name)) {
    localB->bid1 = 0.0;
  } else {
    localB->qidx_idx_1 = obj->NumBodies;
    localB->b_i_e = 0;
    exitg1 = false;
    while ((!exitg1) && (localB->b_i_e <= static_cast<int32_T>
                         (localB->qidx_idx_1) - 1)) {
      body1 = obj->Bodies[localB->b_i_e];
      localB->b_kstr = bname->size[0] * bname->size[1];
      bname->size[0] = 1;
      bname->size[1] = body1->NameInternal->size[1];
      cartes_emxEnsureCapacity_char_T(bname, localB->b_kstr, localB);
      localB->loop_ub_a = body1->NameInternal->size[0] * body1->
        NameInternal->size[1] - 1;
      for (localB->b_kstr = 0; localB->b_kstr <= localB->loop_ub_a;
           localB->b_kstr++) {
        bname->data[localB->b_kstr] = body1->NameInternal->data[localB->b_kstr];
      }

      if (cartesian_trajectory_pla_strcmp(bname, body1Name)) {
        localB->bid1 = static_cast<real_T>(localB->b_i_e) + 1.0;
        exitg1 = true;
      } else {
        localB->b_i_e++;
      }
    }
  }

  localB->bid2 = -1.0;
  localB->b_kstr = bname->size[0] * bname->size[1];
  bname->size[0] = 1;
  bname->size[1] = obj->Base.NameInternal->size[1];
  cartes_emxEnsureCapacity_char_T(bname, localB->b_kstr, localB);
  localB->loop_ub_a = obj->Base.NameInternal->size[0] * obj->
    Base.NameInternal->size[1] - 1;
  for (localB->b_kstr = 0; localB->b_kstr <= localB->loop_ub_a; localB->b_kstr++)
  {
    bname->data[localB->b_kstr] = obj->Base.NameInternal->data[localB->b_kstr];
  }

  if (cartesian_trajectory_pla_strcmp(bname, body2Name)) {
    localB->bid2 = 0.0;
  } else {
    localB->qidx_idx_1 = obj->NumBodies;
    localB->b_i_e = 0;
    exitg1 = false;
    while ((!exitg1) && (localB->b_i_e <= static_cast<int32_T>
                         (localB->qidx_idx_1) - 1)) {
      body1 = obj->Bodies[localB->b_i_e];
      localB->b_kstr = bname->size[0] * bname->size[1];
      bname->size[0] = 1;
      bname->size[1] = body1->NameInternal->size[1];
      cartes_emxEnsureCapacity_char_T(bname, localB->b_kstr, localB);
      localB->loop_ub_a = body1->NameInternal->size[0] * body1->
        NameInternal->size[1] - 1;
      for (localB->b_kstr = 0; localB->b_kstr <= localB->loop_ub_a;
           localB->b_kstr++) {
        bname->data[localB->b_kstr] = body1->NameInternal->data[localB->b_kstr];
      }

      if (cartesian_trajectory_pla_strcmp(bname, body2Name)) {
        localB->bid2 = static_cast<real_T>(localB->b_i_e) + 1.0;
        exitg1 = true;
      } else {
        localB->b_i_e++;
      }
    }
  }

  cartesian_trajec_emxFree_char_T(&bname);
  if (localB->bid1 == 0.0) {
    body1 = &obj->Base;
  } else {
    body1 = obj->Bodies[static_cast<int32_T>(localB->bid1) - 1];
  }

  if (localB->bid2 == 0.0) {
    body2 = &obj->Base;
  } else {
    body2 = obj->Bodies[static_cast<int32_T>(localB->bid2) - 1];
  }

  cartesian_trajec_emxInit_real_T(&ancestorIndices1, 2, localB);
  body = body1;
  localB->b_kstr = ancestorIndices1->size[0] * ancestorIndices1->size[1];
  ancestorIndices1->size[0] = 1;
  ancestorIndices1->size[1] = static_cast<int32_T>(obj->NumBodies + 1.0);
  cartes_emxEnsureCapacity_real_T(ancestorIndices1, localB->b_kstr, localB);
  localB->loop_ub_a = static_cast<int32_T>(obj->NumBodies + 1.0) - 1;
  for (localB->b_kstr = 0; localB->b_kstr <= localB->loop_ub_a; localB->b_kstr++)
  {
    ancestorIndices1->data[localB->b_kstr] = 0.0;
  }

  localB->bid1 = 2.0;
  ancestorIndices1->data[0] = body1->Index;
  while (body->ParentIndex > 0.0) {
    body = obj->Bodies[static_cast<int32_T>(body->ParentIndex) - 1];
    ancestorIndices1->data[static_cast<int32_T>(localB->bid1) - 1] = body->Index;
    localB->bid1++;
  }

  if (body->Index > 0.0) {
    ancestorIndices1->data[static_cast<int32_T>(localB->bid1) - 1] =
      body->ParentIndex;
    localB->bid1++;
  }

  cartesian_trajec_emxInit_real_T(&ancestorIndices1_0, 2, localB);
  localB->loop_ub_a = static_cast<int32_T>(localB->bid1 - 1.0);
  localB->b_kstr = ancestorIndices1_0->size[0] * ancestorIndices1_0->size[1];
  ancestorIndices1_0->size[0] = 1;
  ancestorIndices1_0->size[1] = localB->loop_ub_a;
  cartes_emxEnsureCapacity_real_T(ancestorIndices1_0, localB->b_kstr, localB);
  for (localB->b_kstr = 0; localB->b_kstr < localB->loop_ub_a; localB->b_kstr++)
  {
    ancestorIndices1_0->data[localB->b_kstr] = ancestorIndices1->data
      [localB->b_kstr];
  }

  localB->b_kstr = ancestorIndices1->size[0] * ancestorIndices1->size[1];
  ancestorIndices1->size[0] = 1;
  ancestorIndices1->size[1] = ancestorIndices1_0->size[1];
  cartes_emxEnsureCapacity_real_T(ancestorIndices1, localB->b_kstr, localB);
  localB->loop_ub_a = ancestorIndices1_0->size[0] * ancestorIndices1_0->size[1];
  for (localB->b_kstr = 0; localB->b_kstr < localB->loop_ub_a; localB->b_kstr++)
  {
    ancestorIndices1->data[localB->b_kstr] = ancestorIndices1_0->data
      [localB->b_kstr];
  }

  cartesian_trajec_emxFree_real_T(&ancestorIndices1_0);
  cartesian_trajec_emxInit_real_T(&ancestorIndices2, 2, localB);
  body = body2;
  localB->b_kstr = ancestorIndices2->size[0] * ancestorIndices2->size[1];
  ancestorIndices2->size[0] = 1;
  ancestorIndices2->size[1] = static_cast<int32_T>(obj->NumBodies + 1.0);
  cartes_emxEnsureCapacity_real_T(ancestorIndices2, localB->b_kstr, localB);
  localB->loop_ub_a = static_cast<int32_T>(obj->NumBodies + 1.0) - 1;
  for (localB->b_kstr = 0; localB->b_kstr <= localB->loop_ub_a; localB->b_kstr++)
  {
    ancestorIndices2->data[localB->b_kstr] = 0.0;
  }

  localB->bid1 = 2.0;
  ancestorIndices2->data[0] = body2->Index;
  while (body->ParentIndex > 0.0) {
    body = obj->Bodies[static_cast<int32_T>(body->ParentIndex) - 1];
    ancestorIndices2->data[static_cast<int32_T>(localB->bid1) - 1] = body->Index;
    localB->bid1++;
  }

  if (body->Index > 0.0) {
    ancestorIndices2->data[static_cast<int32_T>(localB->bid1) - 1] =
      body->ParentIndex;
    localB->bid1++;
  }

  cartesian_trajec_emxInit_real_T(&ancestorIndices2_0, 2, localB);
  localB->loop_ub_a = static_cast<int32_T>(localB->bid1 - 1.0);
  localB->b_kstr = ancestorIndices2_0->size[0] * ancestorIndices2_0->size[1];
  ancestorIndices2_0->size[0] = 1;
  ancestorIndices2_0->size[1] = localB->loop_ub_a;
  cartes_emxEnsureCapacity_real_T(ancestorIndices2_0, localB->b_kstr, localB);
  for (localB->b_kstr = 0; localB->b_kstr < localB->loop_ub_a; localB->b_kstr++)
  {
    ancestorIndices2_0->data[localB->b_kstr] = ancestorIndices2->data
      [localB->b_kstr];
  }

  localB->b_kstr = ancestorIndices2->size[0] * ancestorIndices2->size[1];
  ancestorIndices2->size[0] = 1;
  ancestorIndices2->size[1] = ancestorIndices2_0->size[1];
  cartes_emxEnsureCapacity_real_T(ancestorIndices2, localB->b_kstr, localB);
  localB->loop_ub_a = ancestorIndices2_0->size[0] * ancestorIndices2_0->size[1];
  for (localB->b_kstr = 0; localB->b_kstr < localB->loop_ub_a; localB->b_kstr++)
  {
    ancestorIndices2->data[localB->b_kstr] = ancestorIndices2_0->data
      [localB->b_kstr];
  }

  cartesian_trajec_emxFree_real_T(&ancestorIndices2_0);
  localB->bid1 = ancestorIndices1->size[1];
  localB->qidx_idx_1 = ancestorIndices2->size[1];
  if (localB->bid1 < localB->qidx_idx_1) {
    localB->qidx_idx_1 = localB->bid1;
  }

  localB->bid1 = static_cast<int32_T>(localB->qidx_idx_1);
  localB->b_i_e = 0;
  exitg1 = false;
  while ((!exitg1) && (localB->b_i_e <= static_cast<int32_T>(localB->qidx_idx_1)
                       - 2)) {
    if (ancestorIndices1->data[static_cast<int32_T>(static_cast<real_T>
         (ancestorIndices1->size[1]) - (static_cast<real_T>(localB->b_i_e) + 1.0))
        - 1] != ancestorIndices2->data[static_cast<int32_T>(static_cast<real_T>
         (ancestorIndices2->size[1]) - (static_cast<real_T>(localB->b_i_e) + 1.0))
        - 1]) {
      localB->bid1 = static_cast<real_T>(localB->b_i_e) + 1.0;
      exitg1 = true;
    } else {
      localB->b_i_e++;
    }
  }

  localB->qidx_idx_1 = static_cast<real_T>(ancestorIndices1->size[1]) -
    localB->bid1;
  if (1.0 > localB->qidx_idx_1) {
    localB->b_i_e = -1;
  } else {
    localB->b_i_e = static_cast<int32_T>(localB->qidx_idx_1) - 1;
  }

  localB->qidx_idx_1 = static_cast<real_T>(ancestorIndices2->size[1]) -
    localB->bid1;
  if (1.0 > localB->qidx_idx_1) {
    localB->j_e = 0;
    localB->jointSign = 1;
    localB->g = -1;
  } else {
    localB->j_e = static_cast<int32_T>(localB->qidx_idx_1) - 1;
    localB->jointSign = -1;
    localB->g = 0;
  }

  cartesian_trajec_emxInit_real_T(&kinematicPathIndices, 2, localB);
  localB->b_kstr = kinematicPathIndices->size[0] * kinematicPathIndices->size[1];
  kinematicPathIndices->size[0] = 1;
  localB->loop_ub_a = div_s32_floor(localB->g - localB->j_e, localB->jointSign);
  kinematicPathIndices->size[1] = (localB->loop_ub_a + localB->b_i_e) + 3;
  cartes_emxEnsureCapacity_real_T(kinematicPathIndices, localB->b_kstr, localB);
  for (localB->b_kstr = 0; localB->b_kstr <= localB->b_i_e; localB->b_kstr++) {
    kinematicPathIndices->data[localB->b_kstr] = ancestorIndices1->data
      [localB->b_kstr];
  }

  kinematicPathIndices->data[localB->b_i_e + 1] = ancestorIndices1->data[
    static_cast<int32_T>((static_cast<real_T>(ancestorIndices1->size[1]) -
    localB->bid1) + 1.0) - 1];
  cartesian_trajec_emxFree_real_T(&ancestorIndices1);
  for (localB->b_kstr = 0; localB->b_kstr <= localB->loop_ub_a; localB->b_kstr++)
  {
    kinematicPathIndices->data[(localB->b_kstr + localB->b_i_e) + 2] =
      ancestorIndices2->data[localB->jointSign * localB->b_kstr + localB->j_e];
  }

  cartesian_trajec_emxFree_real_T(&ancestorIndices2);
  memset(&localB->T1[0], 0, sizeof(real_T) << 4U);
  localB->T1[0] = 1.0;
  localB->T1[5] = 1.0;
  localB->T1[10] = 1.0;
  localB->T1[15] = 1.0;
  localB->b_kstr = Jac->size[0] * Jac->size[1];
  Jac->size[0] = 6;
  Jac->size[1] = static_cast<int32_T>(obj->PositionNumber);
  cartes_emxEnsureCapacity_real_T(Jac, localB->b_kstr, localB);
  localB->loop_ub_a = 6 * static_cast<int32_T>(obj->PositionNumber) - 1;
  for (localB->b_kstr = 0; localB->b_kstr <= localB->loop_ub_a; localB->b_kstr++)
  {
    Jac->data[localB->b_kstr] = 0.0;
  }

  localB->j_e = kinematicPathIndices->size[1] - 2;
  cartesian_trajec_emxInit_real_T(&y, 2, localB);
  cartesian_trajec_emxInit_real_T(&b, 2, localB);
  if (0 <= localB->j_e) {
    for (localB->b_kstr = 0; localB->b_kstr < 5; localB->b_kstr++) {
      localB->b_c2[localB->b_kstr] = tmp[localB->b_kstr];
    }
  }

  for (localB->b_i_e = 0; localB->b_i_e <= localB->j_e; localB->b_i_e++) {
    if (kinematicPathIndices->data[localB->b_i_e] != 0.0) {
      body1 = obj->Bodies[static_cast<int32_T>(kinematicPathIndices->data
        [localB->b_i_e]) - 1];
    } else {
      body1 = &obj->Base;
    }

    localB->b_kstr = static_cast<int32_T>((static_cast<real_T>(localB->b_i_e) +
      1.0) + 1.0) - 1;
    if (kinematicPathIndices->data[localB->b_kstr] != 0.0) {
      body2 = obj->Bodies[static_cast<int32_T>(kinematicPathIndices->data
        [localB->b_kstr]) - 1];
    } else {
      body2 = &obj->Base;
    }

    localB->nextBodyIsParent = (body2->Index == body1->ParentIndex);
    if (localB->nextBodyIsParent) {
      body2 = body1;
      localB->jointSign = 1;
    } else {
      localB->jointSign = -1;
    }

    joint = body2->JointInternal;
    localB->b_kstr = body2Name->size[0] * body2Name->size[1];
    body2Name->size[0] = 1;
    body2Name->size[1] = joint->Type->size[1];
    cartes_emxEnsureCapacity_char_T(body2Name, localB->b_kstr, localB);
    localB->loop_ub_a = joint->Type->size[0] * joint->Type->size[1] - 1;
    for (localB->b_kstr = 0; localB->b_kstr <= localB->loop_ub_a; localB->b_kstr
         ++) {
      body2Name->data[localB->b_kstr] = joint->Type->data[localB->b_kstr];
    }

    localB->b_bool_p = false;
    if (body2Name->size[1] == 5) {
      localB->b_kstr = 1;
      do {
        exitg2 = 0;
        if (localB->b_kstr - 1 < 5) {
          localB->g = localB->b_kstr - 1;
          if (body2Name->data[localB->g] != localB->b_c2[localB->g]) {
            exitg2 = 1;
          } else {
            localB->b_kstr++;
          }
        } else {
          localB->b_bool_p = true;
          exitg2 = 1;
        }
      } while (exitg2 == 0);
    }

    if (localB->b_bool_p) {
      rigidBodyJoint_transformBodyToP(joint, localB->Tc2p, localB);
    } else {
      localB->b_kstr = static_cast<int32_T>(body2->Index);
      localB->bid1 = obj->PositionDoFMap[localB->b_kstr - 1];
      localB->qidx_idx_1 = obj->PositionDoFMap[localB->b_kstr + 7];
      if (localB->bid1 > localB->qidx_idx_1) {
        localB->g = 0;
        localB->b_kstr = -1;
      } else {
        localB->g = static_cast<int32_T>(localB->bid1) - 1;
        localB->b_kstr = static_cast<int32_T>(localB->qidx_idx_1) - 1;
      }

      localB->loop_ub_a = localB->b_kstr - localB->g;
      localB->qv_size = localB->loop_ub_a + 1;
      for (localB->b_kstr = 0; localB->b_kstr <= localB->loop_ub_a;
           localB->b_kstr++) {
        localB->qv_data[localB->b_kstr] = qv[localB->g + localB->b_kstr];
      }

      rigidBodyJoint_transformBodyT_n(joint, localB->qv_data, &localB->qv_size,
        localB->Tc2p, localB);
      localB->b_kstr = static_cast<int32_T>(body2->Index);
      localB->bid1 = obj->VelocityDoFMap[localB->b_kstr - 1];
      localB->qidx_idx_1 = obj->VelocityDoFMap[localB->b_kstr + 7];
      if (localB->nextBodyIsParent) {
        for (localB->b_kstr = 0; localB->b_kstr < 16; localB->b_kstr++) {
          localB->Tj[localB->b_kstr] = joint->ChildToJointTransform
            [localB->b_kstr];
        }
      } else {
        for (localB->b_kstr = 0; localB->b_kstr < 16; localB->b_kstr++) {
          localB->T1j[localB->b_kstr] = joint->JointToParentTransform
            [localB->b_kstr];
        }

        for (localB->b_kstr = 0; localB->b_kstr < 3; localB->b_kstr++) {
          localB->R[3 * localB->b_kstr] = localB->T1j[localB->b_kstr];
          localB->R[3 * localB->b_kstr + 1] = localB->T1j[localB->b_kstr + 4];
          localB->R[3 * localB->b_kstr + 2] = localB->T1j[localB->b_kstr + 8];
        }

        for (localB->b_kstr = 0; localB->b_kstr < 9; localB->b_kstr++) {
          localB->R_a[localB->b_kstr] = -localB->R[localB->b_kstr];
        }

        for (localB->b_kstr = 0; localB->b_kstr < 3; localB->b_kstr++) {
          localB->g = localB->b_kstr << 2;
          localB->Tj[localB->g] = localB->R[3 * localB->b_kstr];
          localB->Tj[localB->g + 1] = localB->R[3 * localB->b_kstr + 1];
          localB->Tj[localB->g + 2] = localB->R[3 * localB->b_kstr + 2];
          localB->Tj[localB->b_kstr + 12] = localB->R_a[localB->b_kstr + 6] *
            localB->T1j[14] + (localB->R_a[localB->b_kstr + 3] * localB->T1j[13]
                               + localB->R_a[localB->b_kstr] * localB->T1j[12]);
        }

        localB->Tj[3] = 0.0;
        localB->Tj[7] = 0.0;
        localB->Tj[11] = 0.0;
        localB->Tj[15] = 1.0;
      }

      for (localB->b_kstr = 0; localB->b_kstr < 4; localB->b_kstr++) {
        for (localB->g = 0; localB->g < 4; localB->g++) {
          localB->loop_ub_a = localB->g << 2;
          localB->n_b = localB->b_kstr + localB->loop_ub_a;
          localB->T1j[localB->n_b] = 0.0;
          localB->T1j[localB->n_b] += localB->T1[localB->loop_ub_a] * localB->
            Tj[localB->b_kstr];
          localB->T1j[localB->n_b] += localB->T1[localB->loop_ub_a + 1] *
            localB->Tj[localB->b_kstr + 4];
          localB->T1j[localB->n_b] += localB->T1[localB->loop_ub_a + 2] *
            localB->Tj[localB->b_kstr + 8];
          localB->T1j[localB->n_b] += localB->T1[localB->loop_ub_a + 3] *
            localB->Tj[localB->b_kstr + 12];
        }
      }

      for (localB->b_kstr = 0; localB->b_kstr < 3; localB->b_kstr++) {
        localB->R[3 * localB->b_kstr] = localB->T1j[localB->b_kstr];
        localB->R[3 * localB->b_kstr + 1] = localB->T1j[localB->b_kstr + 4];
        localB->R[3 * localB->b_kstr + 2] = localB->T1j[localB->b_kstr + 8];
      }

      for (localB->b_kstr = 0; localB->b_kstr < 9; localB->b_kstr++) {
        localB->R_a[localB->b_kstr] = -localB->R[localB->b_kstr];
      }

      for (localB->b_kstr = 0; localB->b_kstr < 3; localB->b_kstr++) {
        localB->g = localB->b_kstr << 2;
        localB->Tj[localB->g] = localB->R[3 * localB->b_kstr];
        localB->Tj[localB->g + 1] = localB->R[3 * localB->b_kstr + 1];
        localB->Tj[localB->g + 2] = localB->R[3 * localB->b_kstr + 2];
        localB->Tj[localB->b_kstr + 12] = localB->R_a[localB->b_kstr + 6] *
          localB->T1j[14] + (localB->R_a[localB->b_kstr + 3] * localB->T1j[13] +
                             localB->R_a[localB->b_kstr] * localB->T1j[12]);
      }

      localB->Tj[3] = 0.0;
      localB->Tj[7] = 0.0;
      localB->Tj[11] = 0.0;
      localB->Tj[15] = 1.0;
      localB->R[0] = 0.0;
      localB->R[3] = -localB->Tj[14];
      localB->R[6] = localB->Tj[13];
      localB->R[1] = localB->Tj[14];
      localB->R[4] = 0.0;
      localB->R[7] = -localB->Tj[12];
      localB->R[2] = -localB->Tj[13];
      localB->R[5] = localB->Tj[12];
      localB->R[8] = 0.0;
      for (localB->b_kstr = 0; localB->b_kstr < 3; localB->b_kstr++) {
        for (localB->g = 0; localB->g < 3; localB->g++) {
          localB->loop_ub_a = localB->b_kstr + 3 * localB->g;
          localB->R_a[localB->loop_ub_a] = 0.0;
          localB->n_b = localB->g << 2;
          localB->R_a[localB->loop_ub_a] += localB->Tj[localB->n_b] * localB->
            R[localB->b_kstr];
          localB->R_a[localB->loop_ub_a] += localB->Tj[localB->n_b + 1] *
            localB->R[localB->b_kstr + 3];
          localB->R_a[localB->loop_ub_a] += localB->Tj[localB->n_b + 2] *
            localB->R[localB->b_kstr + 6];
          localB->X[localB->g + 6 * localB->b_kstr] = localB->Tj[(localB->b_kstr
            << 2) + localB->g];
          localB->X[localB->g + 6 * (localB->b_kstr + 3)] = 0.0;
        }
      }

      for (localB->b_kstr = 0; localB->b_kstr < 3; localB->b_kstr++) {
        localB->X[6 * localB->b_kstr + 3] = localB->R_a[3 * localB->b_kstr];
        localB->g = localB->b_kstr << 2;
        localB->loop_ub_a = 6 * (localB->b_kstr + 3);
        localB->X[localB->loop_ub_a + 3] = localB->Tj[localB->g];
        localB->X[6 * localB->b_kstr + 4] = localB->R_a[3 * localB->b_kstr + 1];
        localB->X[localB->loop_ub_a + 4] = localB->Tj[localB->g + 1];
        localB->X[6 * localB->b_kstr + 5] = localB->R_a[3 * localB->b_kstr + 2];
        localB->X[localB->loop_ub_a + 5] = localB->Tj[localB->g + 2];
      }

      localB->b_kstr = b->size[0] * b->size[1];
      b->size[0] = 6;
      b->size[1] = joint->MotionSubspace->size[1];
      cartes_emxEnsureCapacity_real_T(b, localB->b_kstr, localB);
      localB->loop_ub_a = joint->MotionSubspace->size[0] * joint->
        MotionSubspace->size[1] - 1;
      for (localB->b_kstr = 0; localB->b_kstr <= localB->loop_ub_a;
           localB->b_kstr++) {
        b->data[localB->b_kstr] = joint->MotionSubspace->data[localB->b_kstr];
      }

      localB->n_b = b->size[1] - 1;
      localB->b_kstr = y->size[0] * y->size[1];
      y->size[0] = 6;
      y->size[1] = b->size[1];
      cartes_emxEnsureCapacity_real_T(y, localB->b_kstr, localB);
      for (localB->b_kstr = 0; localB->b_kstr <= localB->n_b; localB->b_kstr++)
      {
        localB->coffset_tmp = localB->b_kstr * 6 - 1;
        for (localB->g = 0; localB->g < 6; localB->g++) {
          localB->bid2 = 0.0;
          for (localB->loop_ub_a = 0; localB->loop_ub_a < 6; localB->loop_ub_a++)
          {
            localB->bid2 += localB->X[localB->loop_ub_a * 6 + localB->g] *
              b->data[(localB->coffset_tmp + localB->loop_ub_a) + 1];
          }

          y->data[(localB->coffset_tmp + localB->g) + 1] = localB->bid2;
        }
      }

      if (localB->bid1 > localB->qidx_idx_1) {
        localB->n_b = 0;
      } else {
        localB->n_b = static_cast<int32_T>(localB->bid1) - 1;
      }

      localB->loop_ub_a = y->size[1];
      for (localB->b_kstr = 0; localB->b_kstr < localB->loop_ub_a;
           localB->b_kstr++) {
        for (localB->g = 0; localB->g < 6; localB->g++) {
          Jac->data[localB->g + 6 * (localB->n_b + localB->b_kstr)] = y->data[6 *
            localB->b_kstr + localB->g] * static_cast<real_T>(localB->jointSign);
        }
      }
    }

    if (localB->nextBodyIsParent) {
      for (localB->b_kstr = 0; localB->b_kstr < 4; localB->b_kstr++) {
        for (localB->g = 0; localB->g < 4; localB->g++) {
          localB->loop_ub_a = localB->g << 2;
          localB->jointSign = localB->b_kstr + localB->loop_ub_a;
          localB->T1j[localB->jointSign] = 0.0;
          localB->T1j[localB->jointSign] += localB->T1[localB->loop_ub_a] *
            localB->Tc2p[localB->b_kstr];
          localB->T1j[localB->jointSign] += localB->T1[localB->loop_ub_a + 1] *
            localB->Tc2p[localB->b_kstr + 4];
          localB->T1j[localB->jointSign] += localB->T1[localB->loop_ub_a + 2] *
            localB->Tc2p[localB->b_kstr + 8];
          localB->T1j[localB->jointSign] += localB->T1[localB->loop_ub_a + 3] *
            localB->Tc2p[localB->b_kstr + 12];
        }
      }

      memcpy(&localB->T1[0], &localB->T1j[0], sizeof(real_T) << 4U);
    } else {
      for (localB->b_kstr = 0; localB->b_kstr < 3; localB->b_kstr++) {
        localB->R[3 * localB->b_kstr] = localB->Tc2p[localB->b_kstr];
        localB->R[3 * localB->b_kstr + 1] = localB->Tc2p[localB->b_kstr + 4];
        localB->R[3 * localB->b_kstr + 2] = localB->Tc2p[localB->b_kstr + 8];
      }

      for (localB->b_kstr = 0; localB->b_kstr < 9; localB->b_kstr++) {
        localB->R_a[localB->b_kstr] = -localB->R[localB->b_kstr];
      }

      for (localB->b_kstr = 0; localB->b_kstr < 3; localB->b_kstr++) {
        localB->loop_ub_a = localB->b_kstr << 2;
        localB->T1j[localB->loop_ub_a] = localB->R[3 * localB->b_kstr];
        localB->T1j[localB->loop_ub_a + 1] = localB->R[3 * localB->b_kstr + 1];
        localB->T1j[localB->loop_ub_a + 2] = localB->R[3 * localB->b_kstr + 2];
        localB->T1j[localB->b_kstr + 12] = localB->R_a[localB->b_kstr + 6] *
          localB->Tc2p[14] + (localB->R_a[localB->b_kstr + 3] * localB->Tc2p[13]
                              + localB->R_a[localB->b_kstr] * localB->Tc2p[12]);
      }

      localB->T1j[3] = 0.0;
      localB->T1j[7] = 0.0;
      localB->T1j[11] = 0.0;
      localB->T1j[15] = 1.0;
      for (localB->b_kstr = 0; localB->b_kstr < 4; localB->b_kstr++) {
        for (localB->g = 0; localB->g < 4; localB->g++) {
          localB->jointSign = localB->g << 2;
          localB->loop_ub_a = localB->b_kstr + localB->jointSign;
          localB->Tc2p[localB->loop_ub_a] = 0.0;
          localB->Tc2p[localB->loop_ub_a] += localB->T1[localB->jointSign] *
            localB->T1j[localB->b_kstr];
          localB->Tc2p[localB->loop_ub_a] += localB->T1[localB->jointSign + 1] *
            localB->T1j[localB->b_kstr + 4];
          localB->Tc2p[localB->loop_ub_a] += localB->T1[localB->jointSign + 2] *
            localB->T1j[localB->b_kstr + 8];
          localB->Tc2p[localB->loop_ub_a] += localB->T1[localB->jointSign + 3] *
            localB->T1j[localB->b_kstr + 12];
        }
      }

      memcpy(&localB->T1[0], &localB->Tc2p[0], sizeof(real_T) << 4U);
    }
  }

  cartesian_trajec_emxFree_real_T(&b);
  cartesian_trajec_emxFree_char_T(&body2Name);
  cartesian_trajec_emxFree_real_T(&kinematicPathIndices);
  for (localB->b_kstr = 0; localB->b_kstr < 3; localB->b_kstr++) {
    localB->b_i_e = localB->b_kstr << 2;
    localB->X[6 * localB->b_kstr] = localB->T1[localB->b_i_e];
    localB->g = 6 * (localB->b_kstr + 3);
    localB->X[localB->g] = 0.0;
    localB->X[6 * localB->b_kstr + 3] = 0.0;
    localB->X[localB->g + 3] = localB->T1[localB->b_i_e];
    localB->bid1 = localB->T1[localB->b_i_e + 1];
    localB->X[6 * localB->b_kstr + 1] = localB->bid1;
    localB->X[localB->g + 1] = 0.0;
    localB->X[6 * localB->b_kstr + 4] = 0.0;
    localB->X[localB->g + 4] = localB->bid1;
    localB->bid1 = localB->T1[localB->b_i_e + 2];
    localB->X[6 * localB->b_kstr + 2] = localB->bid1;
    localB->X[localB->g + 2] = 0.0;
    localB->X[6 * localB->b_kstr + 5] = 0.0;
    localB->X[localB->g + 5] = localB->bid1;
  }

  localB->n_b = Jac->size[1];
  localB->b_kstr = y->size[0] * y->size[1];
  y->size[0] = 6;
  y->size[1] = Jac->size[1];
  cartes_emxEnsureCapacity_real_T(y, localB->b_kstr, localB);
  localB->loop_ub_a = Jac->size[0] * Jac->size[1] - 1;
  for (localB->b_kstr = 0; localB->b_kstr <= localB->loop_ub_a; localB->b_kstr++)
  {
    y->data[localB->b_kstr] = Jac->data[localB->b_kstr];
  }

  localB->b_kstr = Jac->size[0] * Jac->size[1];
  Jac->size[0] = 6;
  Jac->size[1] = localB->n_b;
  cartes_emxEnsureCapacity_real_T(Jac, localB->b_kstr, localB);
  for (localB->b_kstr = 0; localB->b_kstr < localB->n_b; localB->b_kstr++) {
    localB->coffset_tmp = localB->b_kstr * 6 - 1;
    for (localB->b_i_e = 0; localB->b_i_e < 6; localB->b_i_e++) {
      localB->bid2 = 0.0;
      for (localB->loop_ub_a = 0; localB->loop_ub_a < 6; localB->loop_ub_a++) {
        localB->bid2 += localB->X[localB->loop_ub_a * 6 + localB->b_i_e] *
          y->data[(localB->coffset_tmp + localB->loop_ub_a) + 1];
      }

      Jac->data[(localB->coffset_tmp + localB->b_i_e) + 1] = localB->bid2;
    }
  }

  cartesian_trajec_emxFree_real_T(&y);
  T_size[0] = 4;
  T_size[1] = 4;
  memcpy(&T_data[0], &localB->T1[0], sizeof(real_T) << 4U);
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

static void cartesian_trajectory__xaxpy_cxo(int32_T n, real_T a, const real_T x
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

static void cartesian_trajectory_p_xaxpy_cx(int32_T n, real_T a, const real_T x
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

static void cartesian_trajectory_plan_xrotg(real_T a, real_T b, real_T *b_a,
  real_T *b_b, real_T *c, real_T *s, B_MATLABSystem_cartesian_traj_T *localB)
{
  localB->roe = b;
  localB->absa = fabs(a);
  localB->absb = fabs(b);
  if (localB->absa > localB->absb) {
    localB->roe = a;
  }

  localB->scale = localB->absa + localB->absb;
  if (localB->scale == 0.0) {
    *s = 0.0;
    *c = 1.0;
    *b_a = 0.0;
    *b_b = 0.0;
  } else {
    localB->ads = localB->absa / localB->scale;
    localB->bds = localB->absb / localB->scale;
    *b_a = sqrt(localB->ads * localB->ads + localB->bds * localB->bds) *
      localB->scale;
    if (localB->roe < 0.0) {
      *b_a = -*b_a;
    }

    *c = a / *b_a;
    *s = b / *b_a;
    if (localB->absa > localB->absb) {
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

static void cartesian_trajectory_planne_svd(const real_T A[9], real_T U[9],
  real_T s[3], real_T V[9], B_MATLABSystem_cartesian_traj_T *localB)
{
  int32_T qq;
  boolean_T apply_transform;
  int32_T qjj;
  int32_T m;
  int32_T kase;
  int32_T c_q;
  int32_T d_k;
  boolean_T exitg1;
  localB->e_i[0] = 0.0;
  localB->work[0] = 0.0;
  localB->e_i[1] = 0.0;
  localB->work[1] = 0.0;
  localB->e_i[2] = 0.0;
  localB->work[2] = 0.0;
  for (m = 0; m < 9; m++) {
    localB->A[m] = A[m];
    U[m] = 0.0;
    V[m] = 0.0;
  }

  apply_transform = false;
  localB->nrm = cartesian_trajectory_plan_xnrm2(3, localB->A, 1);
  if (localB->nrm > 0.0) {
    apply_transform = true;
    if (localB->A[0] < 0.0) {
      localB->s[0] = -localB->nrm;
    } else {
      localB->s[0] = localB->nrm;
    }

    if (fabs(localB->s[0]) >= 1.0020841800044864E-292) {
      localB->nrm = 1.0 / localB->s[0];
      for (qq = 1; qq < 4; qq++) {
        localB->A[qq - 1] *= localB->nrm;
      }
    } else {
      for (qq = 1; qq < 4; qq++) {
        localB->A[qq - 1] /= localB->s[0];
      }
    }

    localB->A[0]++;
    localB->s[0] = -localB->s[0];
  } else {
    localB->s[0] = 0.0;
  }

  for (m = 2; m < 4; m++) {
    qjj = (m - 1) * 3 + 1;
    if (apply_transform) {
      memcpy(&localB->A_o[0], &localB->A[0], 9U * sizeof(real_T));
      cartesian_trajectory_plan_xaxpy(3, -(cartesian_trajectory_plan_xdotc(3,
        localB->A, 1, localB->A, qjj) / localB->A[0]), 1, localB->A_o, qjj,
        localB->A);
    }

    localB->e_i[m - 1] = localB->A[qjj - 1];
  }

  for (m = 1; m < 4; m++) {
    U[m - 1] = localB->A[m - 1];
  }

  localB->nrm = cartesian_trajectory_pl_xnrm2_a(localB->e_i, 2);
  if (localB->nrm == 0.0) {
    localB->e_i[0] = 0.0;
  } else {
    if (localB->e_i[1] < 0.0) {
      localB->rt = -localB->nrm;
      localB->e_i[0] = -localB->nrm;
    } else {
      localB->rt = localB->nrm;
      localB->e_i[0] = localB->nrm;
    }

    if (fabs(localB->rt) >= 1.0020841800044864E-292) {
      localB->nrm = 1.0 / localB->rt;
      for (qq = 2; qq < 4; qq++) {
        localB->e_i[qq - 1] *= localB->nrm;
      }
    } else {
      for (qq = 2; qq < 4; qq++) {
        localB->e_i[qq - 1] /= localB->rt;
      }
    }

    localB->e_i[1]++;
    localB->e_i[0] = -localB->e_i[0];
    for (m = 2; m < 4; m++) {
      localB->work[m - 1] = 0.0;
    }

    for (m = 2; m < 4; m++) {
      cartesian_trajectory__xaxpy_cxo(2, localB->e_i[m - 1], localB->A, 3 * (m -
        1) + 2, localB->work, 2);
    }

    for (m = 2; m < 4; m++) {
      memcpy(&localB->A_o[0], &localB->A[0], 9U * sizeof(real_T));
      cartesian_trajectory_p_xaxpy_cx(2, -localB->e_i[m - 1] / localB->e_i[1],
        localB->work, 2, localB->A_o, (m - 1) * 3 + 2, localB->A);
    }
  }

  for (m = 2; m < 4; m++) {
    V[m - 1] = localB->e_i[m - 1];
  }

  apply_transform = false;
  localB->nrm = cartesian_trajectory_plan_xnrm2(2, localB->A, 5);
  if (localB->nrm > 0.0) {
    apply_transform = true;
    if (localB->A[4] < 0.0) {
      localB->s[1] = -localB->nrm;
    } else {
      localB->s[1] = localB->nrm;
    }

    if (fabs(localB->s[1]) >= 1.0020841800044864E-292) {
      localB->nrm = 1.0 / localB->s[1];
      for (qq = 5; qq < 7; qq++) {
        localB->A[qq - 1] *= localB->nrm;
      }
    } else {
      for (qq = 5; qq < 7; qq++) {
        localB->A[qq - 1] /= localB->s[1];
      }
    }

    localB->A[4]++;
    localB->s[1] = -localB->s[1];
  } else {
    localB->s[1] = 0.0;
  }

  if (apply_transform) {
    for (m = 3; m < 4; m++) {
      memcpy(&localB->A_o[0], &localB->A[0], 9U * sizeof(real_T));
      cartesian_trajectory_plan_xaxpy(2, -(cartesian_trajectory_plan_xdotc(2,
        localB->A, 5, localB->A, 8) / localB->A[4]), 5, localB->A_o, 8,
        localB->A);
    }
  }

  for (m = 2; m < 4; m++) {
    U[m + 2] = localB->A[m + 2];
  }

  m = 2;
  localB->s[2] = localB->A[8];
  localB->e_i[1] = localB->A[7];
  localB->e_i[2] = 0.0;
  U[6] = 0.0;
  U[7] = 0.0;
  U[8] = 1.0;
  for (c_q = 1; c_q >= 0; c_q--) {
    qq = 3 * c_q + c_q;
    if (localB->s[c_q] != 0.0) {
      for (kase = c_q + 2; kase < 4; kase++) {
        qjj = ((kase - 1) * 3 + c_q) + 1;
        memcpy(&localB->A[0], &U[0], 9U * sizeof(real_T));
        cartesian_trajectory_plan_xaxpy(3 - c_q,
          -(cartesian_trajectory_plan_xdotc(3 - c_q, U, qq + 1, U, qjj) / U[qq]),
          qq + 1, localB->A, qjj, U);
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
    if ((c_q + 1 <= 1) && (localB->e_i[0] != 0.0)) {
      memcpy(&localB->A[0], &V[0], 9U * sizeof(real_T));
      cartesian_trajectory_plan_xaxpy(2, -(cartesian_trajectory_plan_xdotc(2, V,
        2, V, 5) / V[1]), 2, localB->A, 5, V);
      memcpy(&localB->A[0], &V[0], 9U * sizeof(real_T));
      cartesian_trajectory_plan_xaxpy(2, -(cartesian_trajectory_plan_xdotc(2, V,
        2, V, 8) / V[1]), 2, localB->A, 8, V);
    }

    V[3 * c_q] = 0.0;
    V[3 * c_q + 1] = 0.0;
    V[3 * c_q + 2] = 0.0;
    V[c_q + 3 * c_q] = 1.0;
  }

  for (c_q = 0; c_q < 3; c_q++) {
    localB->ztest = localB->e_i[c_q];
    if (localB->s[c_q] != 0.0) {
      localB->rt = fabs(localB->s[c_q]);
      localB->nrm = localB->s[c_q] / localB->rt;
      localB->s[c_q] = localB->rt;
      if (c_q + 1 < 3) {
        localB->ztest = localB->e_i[c_q] / localB->nrm;
      }

      qjj = 3 * c_q;
      for (qq = qjj + 1; qq <= qjj + 3; qq++) {
        U[qq - 1] *= localB->nrm;
      }
    }

    if ((c_q + 1 < 3) && (localB->ztest != 0.0)) {
      localB->rt = fabs(localB->ztest);
      localB->nrm = localB->rt / localB->ztest;
      localB->ztest = localB->rt;
      localB->s[c_q + 1] *= localB->nrm;
      qjj = (c_q + 1) * 3;
      for (qq = qjj + 1; qq <= qjj + 3; qq++) {
        V[qq - 1] *= localB->nrm;
      }
    }

    localB->e_i[c_q] = localB->ztest;
  }

  qq = 0;
  localB->nrm = 0.0;
  localB->ztest = fabs(localB->s[0]);
  localB->rt = fabs(localB->e_i[0]);
  if ((localB->ztest > localB->rt) || rtIsNaN(localB->rt)) {
    localB->rt = localB->ztest;
  }

  if (!rtIsNaN(localB->rt)) {
    localB->nrm = localB->rt;
  }

  localB->ztest = fabs(localB->s[1]);
  localB->rt = fabs(localB->e_i[1]);
  if ((localB->ztest > localB->rt) || rtIsNaN(localB->rt)) {
    localB->rt = localB->ztest;
  }

  if ((!(localB->nrm > localB->rt)) && (!rtIsNaN(localB->rt))) {
    localB->nrm = localB->rt;
  }

  localB->ztest = fabs(localB->s[2]);
  localB->rt = fabs(localB->e_i[2]);
  if ((localB->ztest > localB->rt) || rtIsNaN(localB->rt)) {
    localB->rt = localB->ztest;
  }

  if ((!(localB->nrm > localB->rt)) && (!rtIsNaN(localB->rt))) {
    localB->nrm = localB->rt;
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
        localB->rt = fabs(localB->e_i[qjj - 1]);
        if ((localB->rt <= (fabs(localB->s[qjj - 1]) + fabs(localB->s[qjj])) *
             2.2204460492503131E-16) || (localB->rt <= 1.0020841800044864E-292) ||
            ((qq > 20) && (localB->rt <= 2.2204460492503131E-16 * localB->nrm)))
        {
          localB->e_i[qjj - 1] = 0.0;
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
          localB->rt = 0.0;
          if (kase < m + 1) {
            localB->rt = fabs(localB->e_i[kase - 1]);
          }

          if (kase > c_q + 1) {
            localB->rt += fabs(localB->e_i[kase - 2]);
          }

          localB->ztest = fabs(localB->s[kase - 1]);
          if ((localB->ztest <= 2.2204460492503131E-16 * localB->rt) ||
              (localB->ztest <= 1.0020841800044864E-292)) {
            localB->s[kase - 1] = 0.0;
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
      localB->rt = localB->e_i[m - 1];
      localB->e_i[m - 1] = 0.0;
      for (qjj = m; qjj >= c_q + 1; qjj--) {
        localB->ztest = localB->e_i[0];
        cartesian_trajectory_plan_xrotg(localB->s[qjj - 1], localB->rt,
          &localB->s[qjj - 1], &localB->rt, &localB->sqds, &localB->b_pr, localB);
        if (qjj > c_q + 1) {
          localB->rt = -localB->b_pr * localB->e_i[0];
          localB->ztest = localB->e_i[0] * localB->sqds;
        }

        memcpy(&localB->A[0], &V[0], 9U * sizeof(real_T));
        cartesian_trajectory_plann_xrot(localB->A, (qjj - 1) * 3 + 1, 3 * m + 1,
          localB->sqds, localB->b_pr, V);
        localB->e_i[0] = localB->ztest;
      }
      break;

     case 2:
      localB->rt = localB->e_i[c_q - 1];
      localB->e_i[c_q - 1] = 0.0;
      for (qjj = c_q + 1; qjj <= m + 1; qjj++) {
        cartesian_trajectory_plan_xrotg(localB->s[qjj - 1], localB->rt,
          &localB->s[qjj - 1], &localB->ztest, &localB->sqds, &localB->b_pr,
          localB);
        localB->ztest = localB->e_i[qjj - 1];
        localB->rt = localB->ztest * -localB->b_pr;
        localB->e_i[qjj - 1] = localB->ztest * localB->sqds;
        memcpy(&localB->A[0], &U[0], 9U * sizeof(real_T));
        cartesian_trajectory_plann_xrot(localB->A, (qjj - 1) * 3 + 1, (c_q - 1) *
          3 + 1, localB->sqds, localB->b_pr, U);
      }
      break;

     case 3:
      localB->ztest = fabs(localB->s[m]);
      localB->sqds = localB->s[m - 1];
      localB->rt = fabs(localB->sqds);
      if ((localB->ztest > localB->rt) || rtIsNaN(localB->rt)) {
        localB->rt = localB->ztest;
      }

      localB->b_pr = localB->e_i[m - 1];
      localB->ztest = fabs(localB->b_pr);
      if ((localB->rt > localB->ztest) || rtIsNaN(localB->ztest)) {
        localB->ztest = localB->rt;
      }

      localB->rt = fabs(localB->s[c_q]);
      if ((localB->ztest > localB->rt) || rtIsNaN(localB->rt)) {
        localB->rt = localB->ztest;
      }

      localB->ztest = fabs(localB->e_i[c_q]);
      if ((localB->rt > localB->ztest) || rtIsNaN(localB->ztest)) {
        localB->ztest = localB->rt;
      }

      localB->rt = localB->s[m] / localB->ztest;
      localB->smm1 = localB->sqds / localB->ztest;
      localB->emm1 = localB->b_pr / localB->ztest;
      localB->sqds = localB->s[c_q] / localB->ztest;
      localB->b_pr = ((localB->smm1 + localB->rt) * (localB->smm1 - localB->rt)
                      + localB->emm1 * localB->emm1) / 2.0;
      localB->smm1 = localB->rt * localB->emm1;
      localB->smm1 *= localB->smm1;
      if ((localB->b_pr != 0.0) || (localB->smm1 != 0.0)) {
        localB->emm1 = sqrt(localB->b_pr * localB->b_pr + localB->smm1);
        if (localB->b_pr < 0.0) {
          localB->emm1 = -localB->emm1;
        }

        localB->emm1 = localB->smm1 / (localB->b_pr + localB->emm1);
      } else {
        localB->emm1 = 0.0;
      }

      localB->rt = (localB->sqds + localB->rt) * (localB->sqds - localB->rt) +
        localB->emm1;
      localB->sqds *= localB->e_i[c_q] / localB->ztest;
      for (d_k = c_q + 1; d_k <= m; d_k++) {
        cartesian_trajectory_plan_xrotg(localB->rt, localB->sqds, &localB->ztest,
          &localB->emm1, &localB->b_pr, &localB->smm1, localB);
        if (d_k > c_q + 1) {
          localB->e_i[0] = localB->ztest;
        }

        localB->ztest = localB->e_i[d_k - 1];
        localB->rt = localB->s[d_k - 1];
        localB->e_i[d_k - 1] = localB->ztest * localB->b_pr - localB->rt *
          localB->smm1;
        localB->sqds = localB->smm1 * localB->s[d_k];
        localB->s[d_k] *= localB->b_pr;
        qjj = (d_k - 1) * 3 + 1;
        kase = 3 * d_k + 1;
        memcpy(&localB->A[0], &V[0], 9U * sizeof(real_T));
        cartesian_trajectory_plann_xrot(localB->A, qjj, kase, localB->b_pr,
          localB->smm1, V);
        cartesian_trajectory_plan_xrotg(localB->rt * localB->b_pr +
          localB->ztest * localB->smm1, localB->sqds, &localB->s[d_k - 1],
          &localB->unusedU2, &localB->emm1, &localB->d_sn, localB);
        localB->rt = localB->e_i[d_k - 1] * localB->emm1 + localB->d_sn *
          localB->s[d_k];
        localB->s[d_k] = localB->e_i[d_k - 1] * -localB->d_sn + localB->emm1 *
          localB->s[d_k];
        localB->sqds = localB->d_sn * localB->e_i[d_k];
        localB->e_i[d_k] *= localB->emm1;
        memcpy(&localB->A[0], &U[0], 9U * sizeof(real_T));
        cartesian_trajectory_plann_xrot(localB->A, qjj, kase, localB->emm1,
          localB->d_sn, U);
      }

      localB->e_i[m - 1] = localB->rt;
      qq++;
      break;

     default:
      if (localB->s[c_q] < 0.0) {
        localB->s[c_q] = -localB->s[c_q];
        qjj = 3 * c_q;
        for (qq = qjj + 1; qq <= qjj + 3; qq++) {
          V[qq - 1] = -V[qq - 1];
        }
      }

      qq = c_q + 1;
      while ((c_q + 1 < 3) && (localB->s[c_q] < localB->s[qq])) {
        localB->rt = localB->s[c_q];
        localB->s[c_q] = localB->s[qq];
        localB->s[qq] = localB->rt;
        qjj = 3 * c_q + 1;
        kase = (c_q + 1) * 3 + 1;
        memcpy(&localB->A[0], &V[0], 9U * sizeof(real_T));
        cartesian_trajectory_plan_xswap(localB->A, qjj, kase, V);
        memcpy(&localB->A[0], &U[0], 9U * sizeof(real_T));
        cartesian_trajectory_plan_xswap(localB->A, qjj, kase, U);
        c_q = qq;
        qq++;
      }

      qq = 0;
      m--;
      break;
    }
  }

  s[0] = localB->s[0];
  s[1] = localB->s[1];
  s[2] = localB->s[2];
}

static void cartesian_trajectory_rotm2axang(const real_T R[9], real_T axang[4],
  B_MATLABSystem_cartesian_traj_T *localB)
{
  boolean_T e;
  boolean_T p;
  boolean_T rEQ0;
  int32_T loop_ub_tmp;
  boolean_T exitg1;
  localB->u.re = (((R[0] + R[4]) + R[8]) - 1.0) * 0.5;
  if (!(fabs(localB->u.re) > 1.0)) {
    localB->v_ox.re = acos(localB->u.re);
  } else {
    localB->u_k.re = localB->u.re + 1.0;
    localB->u_k.im = 0.0;
    localB->dc.re = 1.0 - localB->u.re;
    localB->dc.im = 0.0;
    localB->v_ox.re = 2.0 * rt_atan2d_snf((cartesian_trajectory_plann_sqrt
      (localB->dc)).re, (cartesian_trajectory_plann_sqrt(localB->u_k)).re);
  }

  localB->a_m = 2.0 * sin(localB->v_ox.re);
  localB->v_f[0] = (R[5] - R[7]) / localB->a_m;
  localB->v_f[1] = (R[6] - R[2]) / localB->a_m;
  localB->v_f[2] = (R[1] - R[3]) / localB->a_m;
  if (rtIsNaN(localB->v_ox.re) || rtIsInf(localB->v_ox.re)) {
    localB->a_m = (rtNaN);
  } else if (localB->v_ox.re == 0.0) {
    localB->a_m = 0.0;
  } else {
    localB->a_m = fmod(localB->v_ox.re, 3.1415926535897931);
    rEQ0 = (localB->a_m == 0.0);
    if (!rEQ0) {
      localB->q = fabs(localB->v_ox.re / 3.1415926535897931);
      rEQ0 = !(fabs(localB->q - floor(localB->q + 0.5)) > 2.2204460492503131E-16
               * localB->q);
    }

    if (rEQ0) {
      localB->a_m = 0.0;
    } else {
      if (localB->v_ox.re < 0.0) {
        localB->a_m += 3.1415926535897931;
      }
    }
  }

  rEQ0 = (localB->a_m == 0.0);
  e = true;
  localB->b_k_b = 0;
  exitg1 = false;
  while ((!exitg1) && (localB->b_k_b < 3)) {
    if (!(localB->v_f[localB->b_k_b] == 0.0)) {
      e = false;
      exitg1 = true;
    } else {
      localB->b_k_b++;
    }
  }

  if (rEQ0 || e) {
    loop_ub_tmp = (rEQ0 || e);
    localB->loop_ub_os = loop_ub_tmp * 3 - 1;
    if (0 <= localB->loop_ub_os) {
      memset(&localB->vspecial_data[0], 0, (localB->loop_ub_os + 1) * sizeof
             (real_T));
    }

    loop_ub_tmp--;
    for (localB->loop_ub_os = 0; localB->loop_ub_os <= loop_ub_tmp;
         localB->loop_ub_os++) {
      memset(&localB->b_I[0], 0, 9U * sizeof(real_T));
      localB->b_I[0] = 1.0;
      localB->b_I[4] = 1.0;
      localB->b_I[8] = 1.0;
      p = true;
      for (localB->b_k_b = 0; localB->b_k_b < 9; localB->b_k_b++) {
        localB->a_m = localB->b_I[localB->b_k_b] - R[localB->b_k_b];
        if (p && ((!rtIsInf(localB->a_m)) && (!rtIsNaN(localB->a_m)))) {
        } else {
          p = false;
        }

        localB->b_I[localB->b_k_b] = localB->a_m;
      }

      if (p) {
        cartesian_trajectory_planne_svd(localB->b_I, localB->b_U,
          localB->vspecial_data, localB->V_o, localB);
      } else {
        for (localB->b_k_b = 0; localB->b_k_b < 9; localB->b_k_b++) {
          localB->V_o[localB->b_k_b] = (rtNaN);
        }
      }

      localB->vspecial_data[0] = localB->V_o[6];
      localB->vspecial_data[1] = localB->V_o[7];
      localB->vspecial_data[2] = localB->V_o[8];
    }

    loop_ub_tmp = 0;
    if (rEQ0 || e) {
      for (localB->loop_ub_os = 0; localB->loop_ub_os < 1; localB->loop_ub_os++)
      {
        loop_ub_tmp++;
      }
    }

    for (localB->b_k_b = 0; localB->b_k_b < loop_ub_tmp; localB->b_k_b++) {
      localB->v_f[0] = localB->vspecial_data[3 * localB->b_k_b];
      localB->v_f[1] = localB->vspecial_data[3 * localB->b_k_b + 1];
      localB->v_f[2] = localB->vspecial_data[3 * localB->b_k_b + 2];
    }
  }

  localB->a_m = 1.0 / sqrt((localB->v_f[0] * localB->v_f[0] + localB->v_f[1] *
    localB->v_f[1]) + localB->v_f[2] * localB->v_f[2]);
  localB->v_f[0] *= localB->a_m;
  localB->v_f[1] *= localB->a_m;
  axang[0] = localB->v_f[0];
  axang[1] = localB->v_f[1];
  axang[2] = localB->v_f[2] * localB->a_m;
  axang[3] = localB->v_ox.re;
}

static void cartesian_IKHelpers_computeCost(const real_T x[6],
  f_robotics_manip_internal_IKE_T *args, real_T *cost, real_T W[36],
  emxArray_real_T_cartesian_tra_T *Jac, f_robotics_manip_internal_IKE_T **b_args,
  B_MATLABSystem_cartesian_traj_T *localB)
{
  x_robotics_manip_internal_Rig_T *treeInternal;
  emxArray_char_T_cartesian_tra_T *bodyName;
  emxArray_real_T_cartesian_tra_T *J;
  emxArray_real_T_cartesian_tra_T *y;
  cartesian_trajec_emxInit_char_T(&bodyName, 2, localB);
  *b_args = args;
  treeInternal = args->Robot;
  localB->b_j_a = bodyName->size[0] * bodyName->size[1];
  bodyName->size[0] = 1;
  bodyName->size[1] = args->BodyName->size[1];
  cartes_emxEnsureCapacity_char_T(bodyName, localB->b_j_a, localB);
  localB->loop_ub_d = args->BodyName->size[0] * args->BodyName->size[1] - 1;
  for (localB->b_j_a = 0; localB->b_j_a <= localB->loop_ub_d; localB->b_j_a++) {
    bodyName->data[localB->b_j_a] = args->BodyName->data[localB->b_j_a];
  }

  for (localB->b_j_a = 0; localB->b_j_a < 16; localB->b_j_a++) {
    localB->Td[localB->b_j_a] = args->Tform[localB->b_j_a];
  }

  for (localB->b_j_a = 0; localB->b_j_a < 36; localB->b_j_a++) {
    W[localB->b_j_a] = args->WeightMatrix[localB->b_j_a];
  }

  cartesian_trajec_emxInit_real_T(&J, 2, localB);
  RigidBodyTree_efficientFKAndJac(treeInternal, x, bodyName, localB->T_data,
    localB->T_size, J, localB);
  localB->b_j_a = Jac->size[0] * Jac->size[1];
  Jac->size[0] = 6;
  Jac->size[1] = J->size[1];
  cartes_emxEnsureCapacity_real_T(Jac, localB->b_j_a, localB);
  localB->loop_ub_d = J->size[0] * J->size[1] - 1;
  cartesian_trajec_emxFree_char_T(&bodyName);
  for (localB->b_j_a = 0; localB->b_j_a <= localB->loop_ub_d; localB->b_j_a++) {
    Jac->data[localB->b_j_a] = -J->data[localB->b_j_a];
  }

  cartesian_trajec_emxFree_real_T(&J);
  for (localB->b_j_a = 0; localB->b_j_a < 3; localB->b_j_a++) {
    localB->T[3 * localB->b_j_a] = localB->T_data[localB->b_j_a];
    localB->n_k = 3 * localB->b_j_a + 1;
    localB->T[localB->n_k] = localB->T_data[((localB->b_j_a + 1) +
      localB->T_size[0]) - 1];
    localB->boffset_f = 3 * localB->b_j_a + 2;
    localB->T[localB->boffset_f] = localB->T_data[((localB->b_j_a + 1) +
      (localB->T_size[0] << 1)) - 1];
    for (localB->loop_ub_d = 0; localB->loop_ub_d < 3; localB->loop_ub_d++) {
      localB->Td_tmp = localB->loop_ub_d + 3 * localB->b_j_a;
      localB->Td_p[localB->Td_tmp] = 0.0;
      localB->Td_p[localB->Td_tmp] += localB->T[3 * localB->b_j_a] * localB->
        Td[localB->loop_ub_d];
      localB->Td_p[localB->Td_tmp] += localB->T[localB->n_k] * localB->Td
        [localB->loop_ub_d + 4];
      localB->Td_p[localB->Td_tmp] += localB->T[localB->boffset_f] * localB->
        Td[localB->loop_ub_d + 8];
    }
  }

  cartesian_trajectory_rotm2axang(localB->Td_p, localB->v, localB);
  localB->e[0] = localB->v[3] * localB->v[0];
  localB->e[3] = localB->Td[12] - localB->T_data[localB->T_size[0] * 3];
  localB->e[1] = localB->v[3] * localB->v[1];
  localB->e[4] = localB->Td[13] - localB->T_data[localB->T_size[0] * 3 + 1];
  localB->e[2] = localB->v[3] * localB->v[2];
  localB->e[5] = localB->Td[14] - localB->T_data[localB->T_size[0] * 3 + 2];
  localB->b_j_a = args->ErrTemp->size[0];
  args->ErrTemp->size[0] = 6;
  cartes_emxEnsureCapacity_real_T(args->ErrTemp, localB->b_j_a, localB);
  for (localB->b_j_a = 0; localB->b_j_a < 6; localB->b_j_a++) {
    args->ErrTemp->data[localB->b_j_a] = localB->e[localB->b_j_a];
  }

  for (localB->b_j_a = 0; localB->b_j_a < 6; localB->b_j_a++) {
    localB->y[localB->b_j_a] = 0.0;
    for (localB->loop_ub_d = 0; localB->loop_ub_d < 6; localB->loop_ub_d++) {
      localB->s_g = W[6 * localB->b_j_a + localB->loop_ub_d] * (0.5 * localB->
        e[localB->loop_ub_d]) + localB->y[localB->b_j_a];
      localB->y[localB->b_j_a] = localB->s_g;
    }
  }

  localB->s_g = 0.0;
  for (localB->b_j_a = 0; localB->b_j_a < 6; localB->b_j_a++) {
    localB->s_g += localB->y[localB->b_j_a] * localB->e[localB->b_j_a];
  }

  args->CostTemp = localB->s_g;
  for (localB->b_j_a = 0; localB->b_j_a < 6; localB->b_j_a++) {
    localB->y[localB->b_j_a] = 0.0;
    for (localB->loop_ub_d = 0; localB->loop_ub_d < 6; localB->loop_ub_d++) {
      localB->s_g = W[6 * localB->b_j_a + localB->loop_ub_d] * localB->e
        [localB->loop_ub_d] + localB->y[localB->b_j_a];
      localB->y[localB->b_j_a] = localB->s_g;
    }
  }

  cartesian_trajec_emxInit_real_T(&y, 2, localB);
  localB->n_k = Jac->size[1] - 1;
  localB->b_j_a = y->size[0] * y->size[1];
  y->size[0] = 1;
  y->size[1] = Jac->size[1];
  cartes_emxEnsureCapacity_real_T(y, localB->b_j_a, localB);
  for (localB->b_j_a = 0; localB->b_j_a <= localB->n_k; localB->b_j_a++) {
    localB->boffset_f = localB->b_j_a * 6 - 1;
    localB->s_g = 0.0;
    for (localB->loop_ub_d = 0; localB->loop_ub_d < 6; localB->loop_ub_d++) {
      localB->s_g += Jac->data[(localB->boffset_f + localB->loop_ub_d) + 1] *
        localB->y[localB->loop_ub_d];
    }

    y->data[localB->b_j_a] = localB->s_g;
  }

  localB->b_j_a = args->GradTemp->size[0];
  args->GradTemp->size[0] = y->size[1];
  cartes_emxEnsureCapacity_real_T(args->GradTemp, localB->b_j_a, localB);
  localB->loop_ub_d = y->size[1];
  for (localB->b_j_a = 0; localB->b_j_a < localB->loop_ub_d; localB->b_j_a++) {
    args->GradTemp->data[localB->b_j_a] = y->data[localB->b_j_a];
  }

  cartesian_trajec_emxFree_real_T(&y);
  localB->s_g = args->CostTemp;
  *cost = localB->s_g;
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

static real_T SystemTimeProvider_getElapsedTi(const
  f_robotics_core_internal_Syst_T *obj)
{
  real_T systemTime;
  systemTime = ctimefun();
  return systemTime - obj->StartTime;
}

static real_T cartesian_trajectory_pla_norm_j(const real_T x[6])
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
  if (cartesian_trajectory_pla_norm_j(Hg) < obj->GradientTolerance) {
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

static void cartesian_trajectory_pl_xzgetrf(int32_T m, int32_T n, const
  emxArray_real_T_cartesian_tra_T *A, int32_T lda,
  emxArray_real_T_cartesian_tra_T *b_A, emxArray_int32_T_cartesian_tr_T *ipiv,
  int32_T *info, B_MATLABSystem_cartesian_traj_T *localB)
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
  cartes_emxEnsureCapacity_real_T(b_A, k, localB);
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
  cartesian_trajec_emxInit_real_T(&c_x, 2, localB);
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
          cartes_emxEnsureCapacity_real_T(c_x, ix, localB);
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
          cartes_emxEnsureCapacity_real_T(b_A, k, localB);
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
  emxArray_real_T_cartesian_tra_T *b_B, B_MATLABSystem_cartesian_traj_T *localB)
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
  cartes_emxEnsureCapacity_real_T(b_B, i_0, localB);
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

static real_T cartesian_trajectory_p_xnrm2_ad(int32_T n, const
  emxArray_real_T_cartesian_tra_T *x, int32_T ix0,
  B_MATLABSystem_cartesian_traj_T *localB)
{
  real_T y;
  y = 0.0;
  if (n >= 1) {
    if (n == 1) {
      y = fabs(x->data[ix0 - 1]);
    } else {
      localB->scale_n = 3.3121686421112381E-170;
      localB->kend_c = ix0 + n;
      for (localB->k_k = ix0; localB->k_k < localB->kend_c; localB->k_k++) {
        localB->absxk_p = fabs(x->data[localB->k_k - 1]);
        if (localB->absxk_p > localB->scale_n) {
          localB->t_d = localB->scale_n / localB->absxk_p;
          y = y * localB->t_d * localB->t_d + 1.0;
          localB->scale_n = localB->absxk_p;
        } else {
          localB->t_d = localB->absxk_p / localB->scale_n;
          y += localB->t_d * localB->t_d;
        }
      }

      y = localB->scale_n * sqrt(y);
    }
  }

  return y;
}

static void cartesian_trajectory_pla_qrpf_b(const
  emxArray_real_T_cartesian_tra_T *A, int32_T m, int32_T n,
  emxArray_real_T_cartesian_tra_T *tau, const emxArray_int32_T_cartesian_tr_T
  *jpvt, emxArray_real_T_cartesian_tra_T *b_A, emxArray_int32_T_cartesian_tr_T
  *b_jpvt, B_MATLABSystem_cartesian_traj_T *localB)
{
  emxArray_real_T_cartesian_tra_T *work;
  emxArray_real_T_cartesian_tra_T *vn1;
  emxArray_real_T_cartesian_tra_T *vn2;
  emxArray_real_T_cartesian_tra_T *c_x;
  int32_T exitg1;
  boolean_T exitg2;
  localB->kend = b_jpvt->size[0] * b_jpvt->size[1];
  b_jpvt->size[0] = 1;
  b_jpvt->size[1] = jpvt->size[1];
  carte_emxEnsureCapacity_int32_T(b_jpvt, localB->kend);
  localB->ix_i = jpvt->size[0] * jpvt->size[1] - 1;
  for (localB->kend = 0; localB->kend <= localB->ix_i; localB->kend++) {
    b_jpvt->data[localB->kend] = jpvt->data[localB->kend];
  }

  localB->kend = b_A->size[0] * b_A->size[1];
  b_A->size[0] = A->size[0];
  b_A->size[1] = A->size[1];
  cartes_emxEnsureCapacity_real_T(b_A, localB->kend, localB);
  localB->ix_i = A->size[0] * A->size[1] - 1;
  for (localB->kend = 0; localB->kend <= localB->ix_i; localB->kend++) {
    b_A->data[localB->kend] = A->data[localB->kend];
  }

  cartesian_trajec_emxInit_real_T(&work, 1, localB);
  localB->ma = A->size[0];
  if (m < n) {
    localB->m_gr = m;
  } else {
    localB->m_gr = n;
  }

  localB->minmn_e = localB->m_gr - 1;
  localB->kend = work->size[0];
  work->size[0] = A->size[1];
  cartes_emxEnsureCapacity_real_T(work, localB->kend, localB);
  localB->ix_i = A->size[1];
  for (localB->kend = 0; localB->kend < localB->ix_i; localB->kend++) {
    work->data[localB->kend] = 0.0;
  }

  cartesian_trajec_emxInit_real_T(&vn1, 1, localB);
  localB->kend = vn1->size[0];
  vn1->size[0] = A->size[1];
  cartes_emxEnsureCapacity_real_T(vn1, localB->kend, localB);
  localB->ix_i = A->size[1];
  for (localB->kend = 0; localB->kend < localB->ix_i; localB->kend++) {
    vn1->data[localB->kend] = 0.0;
  }

  cartesian_trajec_emxInit_real_T(&vn2, 1, localB);
  localB->kend = vn2->size[0];
  vn2->size[0] = A->size[1];
  cartes_emxEnsureCapacity_real_T(vn2, localB->kend, localB);
  localB->ix_i = A->size[1];
  for (localB->kend = 0; localB->kend < localB->ix_i; localB->kend++) {
    vn2->data[localB->kend] = 0.0;
  }

  for (localB->m_gr = 0; localB->m_gr < n; localB->m_gr++) {
    localB->pvt = localB->m_gr * localB->ma;
    localB->smax = 0.0;
    if (m >= 1) {
      if (m == 1) {
        localB->smax = fabs(A->data[localB->pvt]);
      } else {
        localB->scale_c = 3.3121686421112381E-170;
        localB->kend = localB->pvt + m;
        for (localB->itemp = localB->pvt + 1; localB->itemp <= localB->kend;
             localB->itemp++) {
          localB->absxk = fabs(A->data[localB->itemp - 1]);
          if (localB->absxk > localB->scale_c) {
            localB->t = localB->scale_c / localB->absxk;
            localB->smax = localB->smax * localB->t * localB->t + 1.0;
            localB->scale_c = localB->absxk;
          } else {
            localB->t = localB->absxk / localB->scale_c;
            localB->smax += localB->t * localB->t;
          }
        }

        localB->smax = localB->scale_c * sqrt(localB->smax);
      }
    }

    vn1->data[localB->m_gr] = localB->smax;
    vn2->data[localB->m_gr] = vn1->data[localB->m_gr];
  }

  cartesian_trajec_emxInit_real_T(&c_x, 2, localB);
  for (localB->m_gr = 0; localB->m_gr <= localB->minmn_e; localB->m_gr++) {
    localB->iy_g = localB->m_gr * localB->ma;
    localB->ii = localB->iy_g + localB->m_gr;
    localB->nmi = n - localB->m_gr;
    localB->mmi = (m - localB->m_gr) - 1;
    if (localB->nmi < 1) {
      localB->kend = 0;
    } else {
      localB->kend = 1;
      if (localB->nmi > 1) {
        localB->ix_i = localB->m_gr;
        localB->smax = fabs(vn1->data[localB->m_gr]);
        for (localB->itemp = 2; localB->itemp <= localB->nmi; localB->itemp++) {
          localB->ix_i++;
          localB->scale_c = fabs(vn1->data[localB->ix_i]);
          if (localB->scale_c > localB->smax) {
            localB->kend = localB->itemp;
            localB->smax = localB->scale_c;
          }
        }
      }
    }

    localB->pvt = (localB->m_gr + localB->kend) - 1;
    if (localB->pvt + 1 != localB->m_gr + 1) {
      localB->kend = c_x->size[0] * c_x->size[1];
      c_x->size[0] = b_A->size[0];
      c_x->size[1] = b_A->size[1];
      cartes_emxEnsureCapacity_real_T(c_x, localB->kend, localB);
      localB->ix_i = b_A->size[0] * b_A->size[1] - 1;
      for (localB->kend = 0; localB->kend <= localB->ix_i; localB->kend++) {
        c_x->data[localB->kend] = b_A->data[localB->kend];
      }

      localB->ix_i = localB->pvt * localB->ma;
      for (localB->itemp = 0; localB->itemp < m; localB->itemp++) {
        localB->scale_c = c_x->data[localB->ix_i];
        c_x->data[localB->ix_i] = c_x->data[localB->iy_g];
        c_x->data[localB->iy_g] = localB->scale_c;
        localB->ix_i++;
        localB->iy_g++;
      }

      localB->kend = b_A->size[0] * b_A->size[1];
      b_A->size[0] = c_x->size[0];
      b_A->size[1] = c_x->size[1];
      cartes_emxEnsureCapacity_real_T(b_A, localB->kend, localB);
      localB->ix_i = c_x->size[0] * c_x->size[1] - 1;
      for (localB->kend = 0; localB->kend <= localB->ix_i; localB->kend++) {
        b_A->data[localB->kend] = c_x->data[localB->kend];
      }

      localB->itemp = b_jpvt->data[localB->pvt];
      b_jpvt->data[localB->pvt] = b_jpvt->data[localB->m_gr];
      b_jpvt->data[localB->m_gr] = localB->itemp;
      vn1->data[localB->pvt] = vn1->data[localB->m_gr];
      vn2->data[localB->pvt] = vn2->data[localB->m_gr];
    }

    if (localB->m_gr + 1 < m) {
      localB->pvt = localB->ii + 2;
      localB->kend = c_x->size[0] * c_x->size[1];
      c_x->size[0] = b_A->size[0];
      c_x->size[1] = b_A->size[1];
      cartes_emxEnsureCapacity_real_T(c_x, localB->kend, localB);
      localB->ix_i = b_A->size[0] * b_A->size[1] - 1;
      for (localB->kend = 0; localB->kend <= localB->ix_i; localB->kend++) {
        c_x->data[localB->kend] = b_A->data[localB->kend];
      }

      localB->smax = b_A->data[localB->ii];
      tau->data[localB->m_gr] = 0.0;
      if (localB->mmi + 1 > 0) {
        localB->scale_c = cartesian_trajectory_p_xnrm2_ad(localB->mmi, b_A,
          localB->ii + 2, localB);
        if (localB->scale_c != 0.0) {
          localB->scale_c = rt_hypotd_snf(b_A->data[localB->ii], localB->scale_c);
          if (b_A->data[localB->ii] >= 0.0) {
            localB->scale_c = -localB->scale_c;
          }

          if (fabs(localB->scale_c) < 1.0020841800044864E-292) {
            localB->kend = -1;
            localB->ix_i = (localB->ii + localB->mmi) + 1;
            do {
              localB->kend++;
              for (localB->itemp = localB->pvt; localB->itemp <= localB->ix_i;
                   localB->itemp++) {
                c_x->data[localB->itemp - 1] *= 9.9792015476736E+291;
              }

              localB->scale_c *= 9.9792015476736E+291;
              localB->smax *= 9.9792015476736E+291;
            } while (!(fabs(localB->scale_c) >= 1.0020841800044864E-292));

            localB->scale_c = rt_hypotd_snf(localB->smax,
              cartesian_trajectory_p_xnrm2_ad(localB->mmi, c_x, localB->ii + 2,
              localB));
            if (localB->smax >= 0.0) {
              localB->scale_c = -localB->scale_c;
            }

            tau->data[localB->m_gr] = (localB->scale_c - localB->smax) /
              localB->scale_c;
            localB->smax = 1.0 / (localB->smax - localB->scale_c);
            for (localB->itemp = localB->pvt; localB->itemp <= localB->ix_i;
                 localB->itemp++) {
              c_x->data[localB->itemp - 1] *= localB->smax;
            }

            for (localB->itemp = 0; localB->itemp <= localB->kend; localB->itemp
                 ++) {
              localB->scale_c *= 1.0020841800044864E-292;
            }

            localB->smax = localB->scale_c;
          } else {
            tau->data[localB->m_gr] = (localB->scale_c - b_A->data[localB->ii]) /
              localB->scale_c;
            localB->smax = 1.0 / (b_A->data[localB->ii] - localB->scale_c);
            localB->kend = c_x->size[0] * c_x->size[1];
            c_x->size[0] = b_A->size[0];
            c_x->size[1] = b_A->size[1];
            cartes_emxEnsureCapacity_real_T(c_x, localB->kend, localB);
            localB->ix_i = b_A->size[0] * b_A->size[1] - 1;
            for (localB->kend = 0; localB->kend <= localB->ix_i; localB->kend++)
            {
              c_x->data[localB->kend] = b_A->data[localB->kend];
            }

            localB->b_g = (localB->ii + localB->mmi) + 1;
            for (localB->itemp = localB->pvt; localB->itemp <= localB->b_g;
                 localB->itemp++) {
              c_x->data[localB->itemp - 1] *= localB->smax;
            }

            localB->smax = localB->scale_c;
          }
        }
      }

      localB->kend = b_A->size[0] * b_A->size[1];
      b_A->size[0] = c_x->size[0];
      b_A->size[1] = c_x->size[1];
      cartes_emxEnsureCapacity_real_T(b_A, localB->kend, localB);
      localB->ix_i = c_x->size[0] * c_x->size[1] - 1;
      for (localB->kend = 0; localB->kend <= localB->ix_i; localB->kend++) {
        b_A->data[localB->kend] = c_x->data[localB->kend];
      }

      b_A->data[localB->ii] = localB->smax;
    } else {
      tau->data[localB->m_gr] = 0.0;
    }

    if (localB->m_gr + 1 < n) {
      localB->smax = b_A->data[localB->ii];
      b_A->data[localB->ii] = 1.0;
      localB->pvt = (localB->ii + localB->ma) + 1;
      localB->kend = c_x->size[0] * c_x->size[1];
      c_x->size[0] = b_A->size[0];
      c_x->size[1] = b_A->size[1];
      cartes_emxEnsureCapacity_real_T(c_x, localB->kend, localB);
      localB->ix_i = b_A->size[0] * b_A->size[1] - 1;
      for (localB->kend = 0; localB->kend <= localB->ix_i; localB->kend++) {
        c_x->data[localB->kend] = b_A->data[localB->kend];
      }

      if (tau->data[localB->m_gr] != 0.0) {
        localB->itemp = localB->mmi;
        localB->kend = localB->ii + localB->mmi;
        while ((localB->itemp + 1 > 0) && (b_A->data[localB->kend] == 0.0)) {
          localB->itemp--;
          localB->kend--;
        }

        localB->nmi--;
        exitg2 = false;
        while ((!exitg2) && (localB->nmi > 0)) {
          localB->ix_i = (localB->nmi - 1) * localB->ma + localB->pvt;
          localB->kend = localB->ix_i;
          do {
            exitg1 = 0;
            if (localB->kend <= localB->ix_i + localB->itemp) {
              if (b_A->data[localB->kend - 1] != 0.0) {
                exitg1 = 1;
              } else {
                localB->kend++;
              }
            } else {
              localB->nmi--;
              exitg1 = 2;
            }
          } while (exitg1 == 0);

          if (exitg1 == 1) {
            exitg2 = true;
          }
        }

        localB->lastc = localB->nmi - 1;
        localB->kend = c_x->size[0] * c_x->size[1];
        c_x->size[0] = b_A->size[0];
        c_x->size[1] = b_A->size[1];
        cartes_emxEnsureCapacity_real_T(c_x, localB->kend, localB);
        localB->ix_i = b_A->size[0] * b_A->size[1] - 1;
        for (localB->kend = 0; localB->kend <= localB->ix_i; localB->kend++) {
          c_x->data[localB->kend] = b_A->data[localB->kend];
        }
      } else {
        localB->itemp = -1;
        localB->lastc = -1;
      }

      if (localB->itemp + 1 > 0) {
        if (localB->lastc + 1 != 0) {
          for (localB->kend = 0; localB->kend <= localB->lastc; localB->kend++)
          {
            work->data[localB->kend] = 0.0;
          }

          localB->iy_g = 0;
          localB->b_g = localB->ma * localB->lastc + localB->pvt;
          for (localB->nmi = localB->pvt; localB->ma < 0 ? localB->nmi >=
               localB->b_g : localB->nmi <= localB->b_g; localB->nmi +=
               localB->ma) {
            localB->ix_i = localB->ii;
            localB->scale_c = 0.0;
            localB->d_g = localB->nmi + localB->itemp;
            for (localB->kend = localB->nmi; localB->kend <= localB->d_g;
                 localB->kend++) {
              localB->scale_c += c_x->data[localB->kend - 1] * c_x->data
                [localB->ix_i];
              localB->ix_i++;
            }

            work->data[localB->iy_g] += localB->scale_c;
            localB->iy_g++;
          }
        }

        if (!(-tau->data[localB->m_gr] == 0.0)) {
          localB->iy_g = 0;
          for (localB->kend = 0; localB->kend <= localB->lastc; localB->kend++)
          {
            if (work->data[localB->iy_g] != 0.0) {
              localB->scale_c = work->data[localB->iy_g] * -tau->data
                [localB->m_gr];
              localB->ix_i = localB->ii;
              localB->b_g = localB->itemp + localB->pvt;
              for (localB->nmi = localB->pvt; localB->nmi <= localB->b_g;
                   localB->nmi++) {
                c_x->data[localB->nmi - 1] += c_x->data[localB->ix_i] *
                  localB->scale_c;
                localB->ix_i++;
              }
            }

            localB->iy_g++;
            localB->pvt += localB->ma;
          }
        }
      }

      localB->kend = b_A->size[0] * b_A->size[1];
      b_A->size[0] = c_x->size[0];
      b_A->size[1] = c_x->size[1];
      cartes_emxEnsureCapacity_real_T(b_A, localB->kend, localB);
      localB->ix_i = c_x->size[0] * c_x->size[1] - 1;
      for (localB->kend = 0; localB->kend <= localB->ix_i; localB->kend++) {
        b_A->data[localB->kend] = c_x->data[localB->kend];
      }

      b_A->data[localB->ii] = localB->smax;
    }

    for (localB->ii = localB->m_gr + 2; localB->ii <= n; localB->ii++) {
      localB->pvt = ((localB->ii - 1) * localB->ma + localB->m_gr) + 1;
      if (vn1->data[localB->ii - 1] != 0.0) {
        localB->smax = fabs(b_A->data[localB->pvt - 1]) / vn1->data[localB->ii -
          1];
        localB->smax = 1.0 - localB->smax * localB->smax;
        if (localB->smax < 0.0) {
          localB->smax = 0.0;
        }

        localB->scale_c = vn1->data[localB->ii - 1] / vn2->data[localB->ii - 1];
        localB->scale_c = localB->scale_c * localB->scale_c * localB->smax;
        if (localB->scale_c <= 1.4901161193847656E-8) {
          if (localB->m_gr + 1 < m) {
            localB->smax = 0.0;
            if (localB->mmi >= 1) {
              if (localB->mmi == 1) {
                localB->smax = fabs(b_A->data[localB->pvt]);
              } else {
                localB->scale_c = 3.3121686421112381E-170;
                localB->kend = localB->pvt + localB->mmi;
                for (localB->itemp = localB->pvt + 1; localB->itemp <=
                     localB->kend; localB->itemp++) {
                  localB->absxk = fabs(b_A->data[localB->itemp - 1]);
                  if (localB->absxk > localB->scale_c) {
                    localB->t = localB->scale_c / localB->absxk;
                    localB->smax = localB->smax * localB->t * localB->t + 1.0;
                    localB->scale_c = localB->absxk;
                  } else {
                    localB->t = localB->absxk / localB->scale_c;
                    localB->smax += localB->t * localB->t;
                  }
                }

                localB->smax = localB->scale_c * sqrt(localB->smax);
              }
            }

            vn1->data[localB->ii - 1] = localB->smax;
            vn2->data[localB->ii - 1] = vn1->data[localB->ii - 1];
          } else {
            vn1->data[localB->ii - 1] = 0.0;
            vn2->data[localB->ii - 1] = 0.0;
          }
        } else {
          vn1->data[localB->ii - 1] *= sqrt(localB->smax);
        }
      }
    }
  }

  cartesian_trajec_emxFree_real_T(&c_x);
  cartesian_trajec_emxFree_real_T(&vn2);
  cartesian_trajec_emxFree_real_T(&vn1);
  cartesian_trajec_emxFree_real_T(&work);
}

static void cartesian_trajectory_p_mldivide(const
  emxArray_real_T_cartesian_tra_T *A, const emxArray_real_T_cartesian_tra_T *B,
  emxArray_real_T_cartesian_tra_T *Y, B_MATLABSystem_cartesian_traj_T *localB)
{
  emxArray_real_T_cartesian_tra_T *c_A;
  emxArray_real_T_cartesian_tra_T *b_tau;
  emxArray_int32_T_cartesian_tr_T *b_jpvt;
  emxArray_real_T_cartesian_tra_T *B_0;
  emxArray_int32_T_cartesian_tr_T *b_jpvt_0;
  boolean_T guard1 = false;
  cartesian_trajec_emxInit_real_T(&c_A, 2, localB);
  cartesian_trajec_emxInit_real_T(&b_tau, 1, localB);
  cartesian_traje_emxInit_int32_T(&b_jpvt, 2);
  cartesian_trajec_emxInit_real_T(&B_0, 2, localB);
  cartesian_traje_emxInit_int32_T(&b_jpvt_0, 2);
  if ((A->size[0] == 0) || (A->size[1] == 0) || ((B->size[0] == 0) || (B->size[1]
        == 0))) {
    localB->minmn = A->size[1];
    localB->minmana = B->size[1];
    localB->b_i_gz = Y->size[0] * Y->size[1];
    Y->size[0] = localB->minmn;
    Y->size[1] = localB->minmana;
    cartes_emxEnsureCapacity_real_T(Y, localB->b_i_gz, localB);
    localB->minmn = localB->minmn * localB->minmana - 1;
    for (localB->b_i_gz = 0; localB->b_i_gz <= localB->minmn; localB->b_i_gz++)
    {
      Y->data[localB->b_i_gz] = 0.0;
    }
  } else if (A->size[0] == A->size[1]) {
    localB->minmn = A->size[0];
    localB->rankR = A->size[1];
    if (localB->minmn < localB->rankR) {
      localB->rankR = localB->minmn;
    }

    localB->minmn = B->size[0];
    if (localB->minmn < localB->rankR) {
      localB->rankR = localB->minmn;
    }

    localB->nb = B->size[1] - 1;
    cartesian_trajectory_pl_xzgetrf(localB->rankR, localB->rankR, A, A->size[0],
      c_A, b_jpvt, &localB->minmn, localB);
    localB->b_i_gz = B_0->size[0] * B_0->size[1];
    B_0->size[0] = B->size[0];
    B_0->size[1] = B->size[1];
    cartes_emxEnsureCapacity_real_T(B_0, localB->b_i_gz, localB);
    localB->minmn = B->size[0] * B->size[1] - 1;
    for (localB->b_i_gz = 0; localB->b_i_gz <= localB->minmn; localB->b_i_gz++)
    {
      B_0->data[localB->b_i_gz] = B->data[localB->b_i_gz];
    }

    localB->minmn = localB->rankR - 2;
    for (localB->b_i_gz = 0; localB->b_i_gz <= localB->minmn; localB->b_i_gz++)
    {
      if (localB->b_i_gz + 1 != b_jpvt->data[localB->b_i_gz]) {
        localB->na = b_jpvt->data[localB->b_i_gz] - 1;
        for (localB->minmana = 0; localB->minmana <= localB->nb; localB->minmana
             ++) {
          localB->tol_l = B_0->data[B_0->size[0] * localB->minmana +
            localB->b_i_gz];
          B_0->data[localB->b_i_gz + B_0->size[0] * localB->minmana] = B_0->
            data[B_0->size[0] * localB->minmana + localB->na];
          B_0->data[localB->na + B_0->size[0] * localB->minmana] = localB->tol_l;
        }
      }
    }

    if ((B->size[1] == 0) || ((B_0->size[0] == 0) || (B_0->size[1] == 0))) {
    } else {
      for (localB->minmana = 0; localB->minmana <= localB->nb; localB->minmana++)
      {
        localB->m_m = B->size[0] * localB->minmana - 1;
        for (localB->minmn = 0; localB->minmn < localB->rankR; localB->minmn++)
        {
          localB->nb_o = c_A->size[0] * localB->minmn - 1;
          localB->b_i_gz = (localB->minmn + localB->m_m) + 1;
          if (B_0->data[localB->b_i_gz] != 0.0) {
            for (localB->na = localB->minmn + 2; localB->na <= localB->rankR;
                 localB->na++) {
              localB->mn = localB->na + localB->m_m;
              B_0->data[localB->mn] -= B_0->data[localB->b_i_gz] * c_A->
                data[localB->na + localB->nb_o];
            }
          }
        }
      }
    }

    cartesian_trajectory_plan_xtrsm(localB->rankR, B->size[1], c_A, c_A->size[0],
      B_0, B->size[0], Y, localB);
  } else {
    localB->na = A->size[1] - 1;
    localB->b_i_gz = c_A->size[0] * c_A->size[1];
    c_A->size[0] = A->size[0];
    c_A->size[1] = A->size[1];
    cartes_emxEnsureCapacity_real_T(c_A, localB->b_i_gz, localB);
    localB->minmn = A->size[0] * A->size[1] - 1;
    for (localB->b_i_gz = 0; localB->b_i_gz <= localB->minmn; localB->b_i_gz++)
    {
      c_A->data[localB->b_i_gz] = A->data[localB->b_i_gz];
    }

    localB->minmn = A->size[0];
    localB->minmana = A->size[1];
    if (localB->minmn < localB->minmana) {
      localB->minmana = localB->minmn;
    }

    localB->b_i_gz = b_tau->size[0];
    b_tau->size[0] = localB->minmana;
    cartes_emxEnsureCapacity_real_T(b_tau, localB->b_i_gz, localB);
    for (localB->b_i_gz = 0; localB->b_i_gz < localB->minmana; localB->b_i_gz++)
    {
      b_tau->data[localB->b_i_gz] = 0.0;
    }

    guard1 = false;
    if ((A->size[0] == 0) || (A->size[1] == 0)) {
      guard1 = true;
    } else {
      localB->minmn = A->size[0];
      localB->minmana = A->size[1];
      if (localB->minmn < localB->minmana) {
        localB->minmana = localB->minmn;
      }

      if (localB->minmana < 1) {
        guard1 = true;
      } else {
        localB->b_i_gz = b_jpvt->size[0] * b_jpvt->size[1];
        b_jpvt->size[0] = 1;
        b_jpvt->size[1] = A->size[1];
        carte_emxEnsureCapacity_int32_T(b_jpvt, localB->b_i_gz);
        localB->minmn = A->size[1] - 1;
        for (localB->b_i_gz = 0; localB->b_i_gz <= localB->minmn; localB->b_i_gz
             ++) {
          b_jpvt->data[localB->b_i_gz] = 0;
        }

        for (localB->minmn = 0; localB->minmn <= localB->na; localB->minmn++) {
          b_jpvt->data[localB->minmn] = localB->minmn + 1;
        }

        localB->b_i_gz = b_jpvt_0->size[0] * b_jpvt_0->size[1];
        b_jpvt_0->size[0] = 1;
        b_jpvt_0->size[1] = b_jpvt->size[1];
        carte_emxEnsureCapacity_int32_T(b_jpvt_0, localB->b_i_gz);
        localB->minmn = b_jpvt->size[0] * b_jpvt->size[1];
        for (localB->b_i_gz = 0; localB->b_i_gz < localB->minmn; localB->b_i_gz
             ++) {
          b_jpvt_0->data[localB->b_i_gz] = b_jpvt->data[localB->b_i_gz];
        }

        cartesian_trajectory_pla_qrpf_b(A, A->size[0], A->size[1], b_tau,
          b_jpvt_0, c_A, b_jpvt, localB);
      }
    }

    if (guard1) {
      localB->b_i_gz = b_jpvt->size[0] * b_jpvt->size[1];
      b_jpvt->size[0] = 1;
      b_jpvt->size[1] = A->size[1];
      carte_emxEnsureCapacity_int32_T(b_jpvt, localB->b_i_gz);
      localB->minmn = A->size[1] - 1;
      for (localB->b_i_gz = 0; localB->b_i_gz <= localB->minmn; localB->b_i_gz++)
      {
        b_jpvt->data[localB->b_i_gz] = 0;
      }

      for (localB->minmana = 0; localB->minmana <= localB->na; localB->minmana++)
      {
        b_jpvt->data[localB->minmana] = localB->minmana + 1;
      }
    }

    localB->rankR = 0;
    if (c_A->size[0] < c_A->size[1]) {
      localB->minmn = c_A->size[0];
      localB->minmana = c_A->size[1];
    } else {
      localB->minmn = c_A->size[1];
      localB->minmana = c_A->size[0];
    }

    if (localB->minmn > 0) {
      localB->tol_l = 2.2204460492503131E-15 * static_cast<real_T>
        (localB->minmana);
      if (1.4901161193847656E-8 < localB->tol_l) {
        localB->tol_l = 1.4901161193847656E-8;
      }

      localB->tol_l *= fabs(c_A->data[0]);
      while ((localB->rankR < localB->minmn) && (!(fabs(c_A->data[c_A->size[0] *
                localB->rankR + localB->rankR]) <= localB->tol_l))) {
        localB->rankR++;
      }
    }

    localB->nb = B->size[1] - 1;
    localB->minmn = c_A->size[1];
    localB->minmana = B->size[1];
    localB->b_i_gz = Y->size[0] * Y->size[1];
    Y->size[0] = localB->minmn;
    Y->size[1] = localB->minmana;
    cartes_emxEnsureCapacity_real_T(Y, localB->b_i_gz, localB);
    localB->minmn = localB->minmn * localB->minmana - 1;
    for (localB->b_i_gz = 0; localB->b_i_gz <= localB->minmn; localB->b_i_gz++)
    {
      Y->data[localB->b_i_gz] = 0.0;
    }

    localB->b_i_gz = B_0->size[0] * B_0->size[1];
    B_0->size[0] = B->size[0];
    B_0->size[1] = B->size[1];
    cartes_emxEnsureCapacity_real_T(B_0, localB->b_i_gz, localB);
    localB->minmn = B->size[0] * B->size[1] - 1;
    for (localB->b_i_gz = 0; localB->b_i_gz <= localB->minmn; localB->b_i_gz++)
    {
      B_0->data[localB->b_i_gz] = B->data[localB->b_i_gz];
    }

    localB->m_m = c_A->size[0];
    localB->nb_o = B->size[1] - 1;
    localB->minmn = c_A->size[0];
    localB->minmana = c_A->size[1];
    if (localB->minmn < localB->minmana) {
      localB->minmana = localB->minmn;
    }

    localB->mn = localB->minmana - 1;
    for (localB->minmana = 0; localB->minmana <= localB->mn; localB->minmana++)
    {
      if (b_tau->data[localB->minmana] != 0.0) {
        for (localB->minmn = 0; localB->minmn <= localB->nb_o; localB->minmn++)
        {
          localB->tol_l = B_0->data[B_0->size[0] * localB->minmn +
            localB->minmana];
          for (localB->na = localB->minmana + 2; localB->na <= localB->m_m;
               localB->na++) {
            localB->tol_l += c_A->data[(c_A->size[0] * localB->minmana +
              localB->na) - 1] * B_0->data[(B_0->size[0] * localB->minmn +
              localB->na) - 1];
          }

          localB->tol_l *= b_tau->data[localB->minmana];
          if (localB->tol_l != 0.0) {
            B_0->data[localB->minmana + B_0->size[0] * localB->minmn] -=
              localB->tol_l;
            for (localB->b_i_gz = localB->minmana + 2; localB->b_i_gz <=
                 localB->m_m; localB->b_i_gz++) {
              B_0->data[(localB->b_i_gz + B_0->size[0] * localB->minmn) - 1] -=
                c_A->data[(c_A->size[0] * localB->minmana + localB->b_i_gz) - 1]
                * localB->tol_l;
            }
          }
        }
      }
    }

    for (localB->minmn = 0; localB->minmn <= localB->nb; localB->minmn++) {
      for (localB->b_i_gz = 0; localB->b_i_gz < localB->rankR; localB->b_i_gz++)
      {
        Y->data[(b_jpvt->data[localB->b_i_gz] + Y->size[0] * localB->minmn) - 1]
          = B_0->data[B_0->size[0] * localB->minmn + localB->b_i_gz];
      }

      for (localB->minmana = localB->rankR; localB->minmana >= 1;
           localB->minmana--) {
        Y->data[(b_jpvt->data[localB->minmana - 1] + Y->size[0] * localB->minmn)
          - 1] /= c_A->data[((localB->minmana - 1) * c_A->size[0] +
                             localB->minmana) - 1];
        localB->na = localB->minmana - 2;
        for (localB->b_i_gz = 0; localB->b_i_gz <= localB->na; localB->b_i_gz++)
        {
          Y->data[(b_jpvt->data[localB->b_i_gz] + Y->size[0] * localB->minmn) -
            1] -= Y->data[(b_jpvt->data[localB->minmana - 1] + Y->size[0] *
                           localB->minmn) - 1] * c_A->data[(localB->minmana - 1)
            * c_A->size[0] + localB->b_i_gz];
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

static void cartesian_trajectory_planne_inv(const
  emxArray_real_T_cartesian_tra_T *x, emxArray_real_T_cartesian_tra_T *y,
  B_MATLABSystem_cartesian_traj_T *localB)
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
    cartes_emxEnsureCapacity_real_T(y, info, localB);
    n_0 = x->size[0] * x->size[1] - 1;
    for (info = 0; info <= n_0; info++) {
      y->data[info] = x->data[info];
    }
  } else {
    n = x->size[0];
    info = y->size[0] * y->size[1];
    y->size[0] = x->size[0];
    y->size[1] = x->size[1];
    cartes_emxEnsureCapacity_real_T(y, info, localB);
    n_0 = x->size[0] * x->size[1] - 1;
    for (info = 0; info <= n_0; info++) {
      y->data[info] = 0.0;
    }

    cartesian_traje_emxInit_int32_T(&p, 2);
    cartesian_trajec_emxInit_real_T(&c_A, 2, localB);
    cartesian_traje_emxInit_int32_T(&b_ipiv, 2);
    cartesian_trajectory_pl_xzgetrf(x->size[0], x->size[0], x, x->size[0], c_A,
      b_ipiv, &info, localB);
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
    cartesian_trajec_emxInit_real_T(&y_0, 2, localB);
    info = y_0->size[0] * y_0->size[1];
    y_0->size[0] = y->size[0];
    y_0->size[1] = y->size[1];
    cartes_emxEnsureCapacity_real_T(y_0, info, localB);
    n_0 = y->size[0] * y->size[1];
    for (info = 0; info < n_0; info++) {
      y_0->data[info] = y->data[info];
    }

    cartesian_trajectory_plan_xtrsm(x->size[0], x->size[0], c_A, x->size[0], y_0,
      x->size[0], y, localB);
    cartesian_trajec_emxFree_real_T(&y_0);
    cartesian_trajec_emxFree_real_T(&c_A);
  }
}

static void cartesian_trajectory_plann_diag(const
  emxArray_real_T_cartesian_tra_T *v, emxArray_real_T_cartesian_tra_T *d,
  B_MATLABSystem_cartesian_traj_T *localB)
{
  int32_T u0;
  int32_T u1;
  if ((v->size[0] == 1) && (v->size[1] == 1)) {
    u0 = d->size[0];
    d->size[0] = 1;
    cartes_emxEnsureCapacity_real_T(d, u0, localB);
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
    cartes_emxEnsureCapacity_real_T(d, u0, localB);
    for (u0 = 0; u0 < u1; u0++) {
      d->data[u0] = v->data[v->size[0] * u0 + u0];
    }
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

static void cartesian_trajectory_pl_sqrt_nh(emxArray_real_T_cartesian_tra_T *x)
{
  int32_T nx;
  int32_T b_k;
  nx = x->size[0] - 1;
  for (b_k = 0; b_k <= nx; b_k++) {
    x->data[b_k] = sqrt(x->data[b_k]);
  }
}

static boolean_T cartesian_tr_isPositiveDefinite(const real_T B[36],
  B_MATLABSystem_cartesian_traj_T *localB)
{
  boolean_T flag;
  emxArray_real_T_cartesian_tra_T *b_x;
  boolean_T exitg1;
  localB->c_A_size_idx_0 = 6;
  localB->c_A_size_idx_1 = 6;
  memcpy(&localB->c_A_data[0], &B[0], 36U * sizeof(real_T));
  localB->b_info = 0;
  localB->b_j_e = 1;
  cartesian_trajec_emxInit_real_T(&b_x, 2, localB);
  exitg1 = false;
  while ((!exitg1) && (localB->b_j_e - 1 < 6)) {
    localB->jm1 = localB->b_j_e - 2;
    localB->idxAjj = ((localB->b_j_e - 1) * 6 + localB->b_j_e) - 1;
    localB->ssq = 0.0;
    if (localB->b_j_e - 1 >= 1) {
      localB->ix_o = localB->b_j_e - 1;
      localB->iy = localB->b_j_e - 1;
      for (localB->k_fr = 0; localB->k_fr <= localB->jm1; localB->k_fr++) {
        localB->ssq += localB->c_A_data[localB->ix_o] * localB->c_A_data
          [localB->iy];
        localB->ix_o += 6;
        localB->iy += 6;
      }
    }

    localB->ssq = localB->c_A_data[localB->idxAjj] - localB->ssq;
    if (localB->ssq > 0.0) {
      localB->ssq = sqrt(localB->ssq);
      localB->c_A_data[localB->idxAjj] = localB->ssq;
      if (localB->b_j_e < 6) {
        if (localB->b_j_e - 1 != 0) {
          localB->ix_o = localB->b_j_e - 1;
          localB->jm1 = (localB->b_j_e - 2) * 6 + localB->b_j_e;
          for (localB->k_fr = localB->b_j_e + 1; localB->k_fr <= localB->jm1 + 1;
               localB->k_fr += 6) {
            localB->c = -localB->c_A_data[localB->ix_o];
            localB->iy = localB->idxAjj + 1;
            localB->d_j = localB->k_fr - localB->b_j_e;
            for (localB->ia = localB->k_fr; localB->ia <= localB->d_j + 5;
                 localB->ia++) {
              localB->c_A_data[localB->iy] += localB->c_A_data[localB->ia - 1] *
                localB->c;
              localB->iy++;
            }

            localB->ix_o += 6;
          }
        }

        localB->ssq = 1.0 / localB->ssq;
        localB->ix_o = b_x->size[0] * b_x->size[1];
        b_x->size[0] = 6;
        b_x->size[1] = 6;
        cartes_emxEnsureCapacity_real_T(b_x, localB->ix_o, localB);
        localB->c_A_size_idx_0 = localB->c_A_size_idx_0 * localB->c_A_size_idx_1
          - 1;
        for (localB->ix_o = 0; localB->ix_o <= localB->c_A_size_idx_0;
             localB->ix_o++) {
          b_x->data[localB->ix_o] = localB->c_A_data[localB->ix_o];
        }

        localB->jm1 = localB->idxAjj - localB->b_j_e;
        for (localB->k_fr = localB->idxAjj + 2; localB->k_fr <= localB->jm1 + 7;
             localB->k_fr++) {
          b_x->data[localB->k_fr - 1] *= localB->ssq;
        }

        localB->c_A_size_idx_0 = b_x->size[0];
        localB->c_A_size_idx_1 = b_x->size[1];
        for (localB->ix_o = 0; localB->ix_o < 36; localB->ix_o++) {
          localB->c_A_data[localB->ix_o] = b_x->data[localB->ix_o];
        }
      }

      localB->b_j_e++;
    } else {
      localB->b_info = localB->b_j_e;
      exitg1 = true;
    }
  }

  cartesian_trajec_emxFree_real_T(&b_x);
  flag = (localB->b_info == 0);
  return flag;
}

static boolean_T DampedBFGSwGradientProjection_k(const
  h_robotics_core_internal_Damp_T *obj, const real_T xNew[6],
  B_MATLABSystem_cartesian_traj_T *localB)
{
  boolean_T flag;
  emxArray_real_T_cartesian_tra_T *b;
  emxArray_real_T_cartesian_tra_T *c;
  int32_T m;
  int32_T inner;
  int32_T b_i;
  int32_T loop_ub;
  emxArray_boolean_T_cartesian__T *c_0;
  cartesian_trajec_emxInit_real_T(&b, 2, localB);
  cartesian_trajec_emxInit_real_T(&c, 1, localB);
  cartesian_tra_emxInit_boolean_T(&c_0, 1);
  if (obj->ConstraintsOn) {
    b_i = b->size[0] * b->size[1];
    b->size[0] = obj->ConstraintMatrix->size[0];
    b->size[1] = obj->ConstraintMatrix->size[1];
    cartes_emxEnsureCapacity_real_T(b, b_i, localB);
    loop_ub = obj->ConstraintMatrix->size[0] * obj->ConstraintMatrix->size[1] -
      1;
    for (b_i = 0; b_i <= loop_ub; b_i++) {
      b->data[b_i] = obj->ConstraintMatrix->data[b_i];
    }

    m = b->size[1] - 1;
    inner = b->size[0] - 1;
    b_i = c->size[0];
    c->size[0] = b->size[1];
    cartes_emxEnsureCapacity_real_T(c, b_i, localB);
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
  *iter, B_MATLABSystem_cartesian_traj_T *localB)
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
  for (localB->i_i = 0; localB->i_i < 6; localB->i_i++) {
    localB->x[localB->i_i] = obj->SeedInternal[localB->i_i];
  }

  cartesian_trajec_emxInit_real_T(&unusedU1, 2, localB);
  cartesian_trajec_emxInit_real_T(&grad, 1, localB);
  obj->TimeObjInternal.StartTime = ctimefun();
  cartesian_IKHelpers_computeCost(localB->x, obj->ExtraArgs, &localB->cost,
    localB->unusedU0, unusedU1, &b, localB);
  obj->ExtraArgs = b;
  args = obj->ExtraArgs;
  localB->b_i_l = grad->size[0];
  grad->size[0] = args->GradTemp->size[0];
  cartes_emxEnsureCapacity_real_T(grad, localB->b_i_l, localB);
  localB->i_i = args->GradTemp->size[0];
  for (localB->b_i_l = 0; localB->b_i_l < localB->i_i; localB->b_i_l++) {
    grad->data[localB->b_i_l] = args->GradTemp->data[localB->b_i_l];
  }

  cartesian_trajectory_planne_eye(localB->unusedU0);
  memcpy(&localB->H[0], &localB->unusedU0[0], 36U * sizeof(real_T));
  cartesian_tra_emxInit_boolean_T(&activeSet, 1);
  cartesian_trajec_emxInit_real_T(&A, 2, localB);
  cartesian_trajec_emxInit_real_T(&alpha, 1, localB);
  cartesian_traje_emxInit_int32_T(&ii, 1);
  if (obj->ConstraintsOn) {
    localB->b_i_l = A->size[0] * A->size[1];
    A->size[0] = obj->ConstraintMatrix->size[0];
    A->size[1] = obj->ConstraintMatrix->size[1];
    cartes_emxEnsureCapacity_real_T(A, localB->b_i_l, localB);
    localB->i_i = obj->ConstraintMatrix->size[0] * obj->ConstraintMatrix->size[1]
      - 1;
    for (localB->b_i_l = 0; localB->b_i_l <= localB->i_i; localB->b_i_l++) {
      A->data[localB->b_i_l] = obj->ConstraintMatrix->data[localB->b_i_l];
    }

    localB->m_i = A->size[1] - 1;
    localB->inner = A->size[0] - 1;
    localB->b_i_l = alpha->size[0];
    alpha->size[0] = A->size[1];
    cartes_emxEnsureCapacity_real_T(alpha, localB->b_i_l, localB);
    for (localB->b_i_l = 0; localB->b_i_l <= localB->m_i; localB->b_i_l++) {
      alpha->data[localB->b_i_l] = 0.0;
    }

    for (localB->nx_n = 0; localB->nx_n <= localB->inner; localB->nx_n++) {
      for (localB->g_idx_0 = 0; localB->g_idx_0 <= localB->m_i; localB->g_idx_0
           ++) {
        alpha->data[localB->g_idx_0] += A->data[localB->g_idx_0 * A->size[0] +
          localB->nx_n] * localB->x[localB->nx_n];
      }
    }

    localB->b_i_l = activeSet->size[0];
    activeSet->size[0] = alpha->size[0];
    car_emxEnsureCapacity_boolean_T(activeSet, localB->b_i_l);
    localB->i_i = alpha->size[0];
    for (localB->b_i_l = 0; localB->b_i_l < localB->i_i; localB->b_i_l++) {
      activeSet->data[localB->b_i_l] = (alpha->data[localB->b_i_l] >=
        obj->ConstraintBound->data[localB->b_i_l]);
    }

    localB->nx_n = activeSet->size[0] - 1;
    localB->idx = 0;
    for (localB->g_idx_0 = 0; localB->g_idx_0 <= localB->nx_n; localB->g_idx_0++)
    {
      if (activeSet->data[localB->g_idx_0]) {
        localB->idx++;
      }
    }

    localB->b_i_l = ii->size[0];
    ii->size[0] = localB->idx;
    carte_emxEnsureCapacity_int32_T(ii, localB->b_i_l);
    localB->b_i_l = 0;
    for (localB->g_idx_0 = 0; localB->g_idx_0 <= localB->nx_n; localB->g_idx_0++)
    {
      if (activeSet->data[localB->g_idx_0]) {
        ii->data[localB->b_i_l] = localB->g_idx_0 + 1;
        localB->b_i_l++;
      }
    }

    localB->i_i = obj->ConstraintMatrix->size[0];
    localB->b_i_l = A->size[0] * A->size[1];
    A->size[0] = localB->i_i;
    A->size[1] = ii->size[0];
    cartes_emxEnsureCapacity_real_T(A, localB->b_i_l, localB);
    localB->n = ii->size[0];
    for (localB->b_i_l = 0; localB->b_i_l < localB->n; localB->b_i_l++) {
      for (localB->idx = 0; localB->idx < localB->i_i; localB->idx++) {
        A->data[localB->idx + A->size[0] * localB->b_i_l] =
          obj->ConstraintMatrix->data[(ii->data[localB->b_i_l] - 1) *
          obj->ConstraintMatrix->size[0] + localB->idx];
      }
    }
  } else {
    localB->g_idx_0 = obj->ConstraintBound->size[0];
    localB->b_i_l = activeSet->size[0];
    activeSet->size[0] = localB->g_idx_0;
    car_emxEnsureCapacity_boolean_T(activeSet, localB->b_i_l);
    for (localB->b_i_l = 0; localB->b_i_l < localB->g_idx_0; localB->b_i_l++) {
      activeSet->data[localB->b_i_l] = false;
    }

    A->size[0] = 6;
    A->size[1] = 0;
  }

  localB->j = A->size[1] - 1;
  cartesian_trajec_emxInit_real_T(&AIn, 2, localB);
  cartesian_trajec_emxInit_real_T(&A_0, 2, localB);
  cartesian_trajec_emxInit_real_T(&A_1, 1, localB);
  for (localB->nx_n = 0; localB->nx_n <= localB->j; localB->nx_n++) {
    localB->i_i = A->size[0];
    localB->b_i_l = A_0->size[0] * A_0->size[1];
    A_0->size[0] = 1;
    A_0->size[1] = localB->i_i;
    cartes_emxEnsureCapacity_real_T(A_0, localB->b_i_l, localB);
    for (localB->b_i_l = 0; localB->b_i_l < localB->i_i; localB->b_i_l++) {
      A_0->data[localB->b_i_l] = A->data[A->size[0] * localB->nx_n +
        localB->b_i_l];
    }

    localB->i_i = A->size[0];
    localB->b_i_l = A_1->size[0];
    A_1->size[0] = localB->i_i;
    cartes_emxEnsureCapacity_real_T(A_1, localB->b_i_l, localB);
    for (localB->b_i_l = 0; localB->b_i_l < localB->i_i; localB->b_i_l++) {
      A_1->data[localB->b_i_l] = A->data[A->size[0] * localB->nx_n +
        localB->b_i_l];
    }

    localB->A_f = 0.0;
    for (localB->b_i_l = 0; localB->b_i_l < 6; localB->b_i_l++) {
      localB->Hg[localB->b_i_l] = 0.0;
      for (localB->idx = 0; localB->idx < 6; localB->idx++) {
        localB->s_j = localB->H[6 * localB->b_i_l + localB->idx] * A_0->
          data[localB->idx] + localB->Hg[localB->b_i_l];
        localB->Hg[localB->b_i_l] = localB->s_j;
      }

      localB->A_f += localB->Hg[localB->b_i_l] * A_1->data[localB->b_i_l];
    }

    localB->s_j = 1.0 / localB->A_f;
    for (localB->b_i_l = 0; localB->b_i_l < 36; localB->b_i_l++) {
      localB->V[localB->b_i_l] = localB->s_j * localB->H[localB->b_i_l];
    }

    localB->i_i = A->size[0];
    localB->n = A->size[0];
    localB->b_i_l = AIn->size[0] * AIn->size[1];
    AIn->size[0] = localB->i_i;
    AIn->size[1] = localB->n;
    cartes_emxEnsureCapacity_real_T(AIn, localB->b_i_l, localB);
    for (localB->b_i_l = 0; localB->b_i_l < localB->n; localB->b_i_l++) {
      for (localB->idx = 0; localB->idx < localB->i_i; localB->idx++) {
        AIn->data[localB->idx + AIn->size[0] * localB->b_i_l] = A->data[A->size
          [0] * localB->nx_n + localB->idx] * A->data[A->size[0] * localB->nx_n
          + localB->b_i_l];
      }
    }

    localB->n = AIn->size[1] - 1;
    localB->b_i_l = unusedU1->size[0] * unusedU1->size[1];
    unusedU1->size[0] = 6;
    unusedU1->size[1] = AIn->size[1];
    cartes_emxEnsureCapacity_real_T(unusedU1, localB->b_i_l, localB);
    for (localB->idx = 0; localB->idx <= localB->n; localB->idx++) {
      localB->coffset = localB->idx * 6 - 1;
      localB->boffset = localB->idx * AIn->size[0] - 1;
      for (localB->b_i_l = 0; localB->b_i_l < 6; localB->b_i_l++) {
        localB->s_j = 0.0;
        for (localB->g_idx_0 = 0; localB->g_idx_0 < 6; localB->g_idx_0++) {
          localB->s_j += localB->V[localB->g_idx_0 * 6 + localB->b_i_l] *
            AIn->data[(localB->boffset + localB->g_idx_0) + 1];
        }

        unusedU1->data[(localB->coffset + localB->b_i_l) + 1] = localB->s_j;
      }
    }

    for (localB->b_i_l = 0; localB->b_i_l < 6; localB->b_i_l++) {
      for (localB->idx = 0; localB->idx < 6; localB->idx++) {
        localB->s_j = 0.0;
        for (localB->i_i = 0; localB->i_i < 6; localB->i_i++) {
          localB->s_j += unusedU1->data[6 * localB->i_i + localB->b_i_l] *
            localB->H[6 * localB->idx + localB->i_i];
        }

        localB->idxl = 6 * localB->idx + localB->b_i_l;
        localB->H_m[localB->idxl] = localB->H[localB->idxl] - localB->s_j;
      }
    }

    memcpy(&localB->H[0], &localB->H_m[0], 36U * sizeof(real_T));
  }

  cartesian_trajec_emxFree_real_T(&A_1);
  cartesian_trajec_emxFree_real_T(&A_0);
  for (localB->i_i = 0; localB->i_i < 6; localB->i_i++) {
    xSol[localB->i_i] = localB->x[localB->i_i];
  }

  localB->A_f = obj->MaxNumIterationInternal;
  localB->g_idx_0 = 0;
  cartesian_trajec_emxInit_real_T(&L, 1, localB);
  cartesian_traje_emxInit_int32_T(&cb, 1);
  cartesian_traje_emxInit_int32_T(&db, 1);
  cartesian_traje_emxInit_int32_T(&eb, 1);
  cartesian_traje_emxInit_int32_T(&fb, 1);
  cartesian_traje_emxInit_int32_T(&gb, 1);
  cartesian_trajec_emxInit_real_T(&a, 2, localB);
  cartesian_trajec_emxInit_real_T(&y, 1, localB);
  cartesian_traje_emxInit_int32_T(&ii_0, 1);
  cartesian_trajec_emxInit_real_T(&y_0, 2, localB);
  cartesian_tra_emxInit_boolean_T(&x, 1);
  cartesian_trajec_emxInit_real_T(&A_2, 2, localB);
  cartesian_trajec_emxInit_real_T(&sigma, 2, localB);
  cartesian_trajec_emxInit_real_T(&tmp, 2, localB);
  cartesian_trajec_emxInit_real_T(&tmp_0, 2, localB);
  cartesian_trajec_emxInit_real_T(&grad_0, 2, localB);
  cartesian_trajec_emxInit_real_T(&sNew, 2, localB);
  cartesian_traje_emxInit_int32_T(&ii_1, 1);
  cartesian_traje_emxInit_int32_T(&ii_2, 1);
  cartesian_trajec_emxInit_real_T(&grad_1, 2, localB);
  cartesian_trajec_emxInit_real_T(&A_3, 2, localB);
  cartesian_trajec_emxInit_real_T(&alpha_0, 2, localB);
  cartesian_traje_emxInit_int32_T(&ii_3, 1);
  do {
    exitg2 = 0;
    if (localB->g_idx_0 <= static_cast<int32_T>(localB->A_f) - 1) {
      localB->s_j = SystemTimeProvider_getElapsedTi(&obj->TimeObjInternal);
      localB->flag = (localB->s_j > obj->MaxTimeInternal);
      if (localB->flag) {
        *exitFlag = TimeLimitExceeded;
        args = obj->ExtraArgs;
        for (localB->b_i_l = 0; localB->b_i_l < 36; localB->b_i_l++) {
          localB->unusedU0[localB->b_i_l] = args->WeightMatrix[localB->b_i_l];
        }

        localB->b_i_l = grad->size[0];
        grad->size[0] = args->ErrTemp->size[0];
        cartes_emxEnsureCapacity_real_T(grad, localB->b_i_l, localB);
        localB->i_i = args->ErrTemp->size[0];
        for (localB->b_i_l = 0; localB->b_i_l < localB->i_i; localB->b_i_l++) {
          grad->data[localB->b_i_l] = args->ErrTemp->data[localB->b_i_l];
        }

        for (localB->b_i_l = 0; localB->b_i_l < 6; localB->b_i_l++) {
          localB->x[localB->b_i_l] = 0.0;
          for (localB->idx = 0; localB->idx < 6; localB->idx++) {
            localB->A_f = localB->unusedU0[6 * localB->idx + localB->b_i_l] *
              grad->data[localB->idx] + localB->x[localB->b_i_l];
            localB->x[localB->b_i_l] = localB->A_f;
          }
        }

        *err = cartesian_trajectory_pla_norm_j(localB->x);
        *iter = static_cast<real_T>(localB->g_idx_0) + 1.0;
        exitg2 = 1;
      } else {
        if ((A->size[0] == 0) || (A->size[1] == 0)) {
          localB->b_i_l = alpha->size[0];
          alpha->size[0] = 1;
          cartes_emxEnsureCapacity_real_T(alpha, localB->b_i_l, localB);
          alpha->data[0] = 0.0;
        } else {
          localB->m_i = A->size[1] - 1;
          localB->inner = A->size[0] - 1;
          localB->n = A->size[1] - 1;
          localB->b_i_l = AIn->size[0] * AIn->size[1];
          AIn->size[0] = A->size[1];
          AIn->size[1] = A->size[1];
          cartes_emxEnsureCapacity_real_T(AIn, localB->b_i_l, localB);
          for (localB->idx = 0; localB->idx <= localB->n; localB->idx++) {
            localB->coffset = (localB->m_i + 1) * localB->idx - 1;
            localB->boffset = localB->idx * A->size[0] - 1;
            for (localB->b_i_l = 0; localB->b_i_l <= localB->m_i; localB->b_i_l
                 ++) {
              AIn->data[(localB->coffset + localB->b_i_l) + 1] = 0.0;
            }

            for (localB->nx_n = 0; localB->nx_n <= localB->inner; localB->nx_n++)
            {
              localB->s_j = A->data[(localB->boffset + localB->nx_n) + 1];
              for (localB->j = 0; localB->j <= localB->m_i; localB->j++) {
                localB->b_i_l = (localB->coffset + localB->j) + 1;
                AIn->data[localB->b_i_l] += A->data[localB->j * A->size[0] +
                  localB->nx_n] * localB->s_j;
              }
            }
          }

          localB->b_i_l = A_2->size[0] * A_2->size[1];
          A_2->size[0] = A->size[1];
          A_2->size[1] = A->size[0];
          cartes_emxEnsureCapacity_real_T(A_2, localB->b_i_l, localB);
          localB->i_i = A->size[0];
          for (localB->b_i_l = 0; localB->b_i_l < localB->i_i; localB->b_i_l++)
          {
            localB->n = A->size[1];
            for (localB->idx = 0; localB->idx < localB->n; localB->idx++) {
              A_2->data[localB->idx + A_2->size[0] * localB->b_i_l] = A->data
                [A->size[0] * localB->idx + localB->b_i_l];
            }
          }

          cartesian_trajectory_p_mldivide(AIn, A_2, a, localB);
          localB->m_i = a->size[0] - 1;
          localB->inner = a->size[1] - 1;
          localB->b_i_l = alpha->size[0];
          alpha->size[0] = a->size[0];
          cartes_emxEnsureCapacity_real_T(alpha, localB->b_i_l, localB);
          for (localB->b_i_l = 0; localB->b_i_l <= localB->m_i; localB->b_i_l++)
          {
            alpha->data[localB->b_i_l] = 0.0;
          }

          for (localB->nx_n = 0; localB->nx_n <= localB->inner; localB->nx_n++)
          {
            localB->aoffset = localB->nx_n * a->size[0] - 1;
            for (localB->j = 0; localB->j <= localB->m_i; localB->j++) {
              alpha->data[localB->j] += a->data[(localB->aoffset + localB->j) +
                1] * grad->data[localB->nx_n];
            }
          }
        }

        for (localB->b_i_l = 0; localB->b_i_l < 6; localB->b_i_l++) {
          localB->Hg[localB->b_i_l] = 0.0;
          for (localB->idx = 0; localB->idx < 6; localB->idx++) {
            localB->b_gamma = localB->H[6 * localB->idx + localB->b_i_l] *
              grad->data[localB->idx] + localB->Hg[localB->b_i_l];
            localB->Hg[localB->b_i_l] = localB->b_gamma;
          }
        }

        if (DampedBFGSwGradientProjection_a(obj, localB->Hg, alpha)) {
          *exitFlag = LocalMinimumFound;
          args = obj->ExtraArgs;
          for (localB->b_i_l = 0; localB->b_i_l < 36; localB->b_i_l++) {
            localB->unusedU0[localB->b_i_l] = args->WeightMatrix[localB->b_i_l];
          }

          localB->b_i_l = grad->size[0];
          grad->size[0] = args->ErrTemp->size[0];
          cartes_emxEnsureCapacity_real_T(grad, localB->b_i_l, localB);
          localB->i_i = args->ErrTemp->size[0];
          for (localB->b_i_l = 0; localB->b_i_l < localB->i_i; localB->b_i_l++)
          {
            grad->data[localB->b_i_l] = args->ErrTemp->data[localB->b_i_l];
          }

          for (localB->b_i_l = 0; localB->b_i_l < 6; localB->b_i_l++) {
            localB->x[localB->b_i_l] = 0.0;
            for (localB->idx = 0; localB->idx < 6; localB->idx++) {
              localB->A_f = localB->unusedU0[6 * localB->idx + localB->b_i_l] *
                grad->data[localB->idx] + localB->x[localB->b_i_l];
              localB->x[localB->b_i_l] = localB->A_f;
            }
          }

          *err = cartesian_trajectory_pla_norm_j(localB->x);
          *iter = static_cast<real_T>(localB->g_idx_0) + 1.0;
          exitg2 = 1;
        } else {
          guard1 = false;
          guard2 = false;
          if (obj->ConstraintsOn && ((A->size[0] != 0) && (A->size[1] != 0))) {
            localB->m_i = A->size[1] - 1;
            localB->inner = A->size[0] - 1;
            localB->n = A->size[1] - 1;
            localB->b_i_l = AIn->size[0] * AIn->size[1];
            AIn->size[0] = A->size[1];
            AIn->size[1] = A->size[1];
            cartes_emxEnsureCapacity_real_T(AIn, localB->b_i_l, localB);
            for (localB->idx = 0; localB->idx <= localB->n; localB->idx++) {
              localB->coffset = (localB->m_i + 1) * localB->idx - 1;
              localB->boffset = localB->idx * A->size[0] - 1;
              for (localB->b_i_l = 0; localB->b_i_l <= localB->m_i;
                   localB->b_i_l++) {
                AIn->data[(localB->coffset + localB->b_i_l) + 1] = 0.0;
              }

              for (localB->nx_n = 0; localB->nx_n <= localB->inner; localB->nx_n
                   ++) {
                localB->s_j = A->data[(localB->boffset + localB->nx_n) + 1];
                for (localB->j = 0; localB->j <= localB->m_i; localB->j++) {
                  localB->b_i_l = (localB->coffset + localB->j) + 1;
                  AIn->data[localB->b_i_l] += A->data[localB->j * A->size[0] +
                    localB->nx_n] * localB->s_j;
                }
              }
            }

            cartesian_trajectory_planne_inv(AIn, a, localB);
            cartesian_trajectory_plann_diag(a, L, localB);
            cartesian_trajectory_pl_sqrt_nh(L);
            localB->b_i_l = alpha->size[0];
            cartes_emxEnsureCapacity_real_T(alpha, localB->b_i_l, localB);
            localB->i_i = alpha->size[0];
            for (localB->b_i_l = 0; localB->b_i_l < localB->i_i; localB->b_i_l++)
            {
              alpha->data[localB->b_i_l] /= L->data[localB->b_i_l];
            }

            localB->n = alpha->size[0];
            if (alpha->size[0] <= 2) {
              if (alpha->size[0] == 1) {
                localB->s_j = alpha->data[0];
                localB->idxl = 0;
              } else if ((alpha->data[0] < alpha->data[1]) || (rtIsNaN
                          (alpha->data[0]) && (!rtIsNaN(alpha->data[1])))) {
                localB->s_j = alpha->data[1];
                localB->idxl = 1;
              } else {
                localB->s_j = alpha->data[0];
                localB->idxl = 0;
              }
            } else {
              if (!rtIsNaN(alpha->data[0])) {
                localB->idxl = 1;
              } else {
                localB->idxl = 0;
                localB->b_i_l = 2;
                exitg3 = false;
                while ((!exitg3) && (localB->b_i_l <= alpha->size[0])) {
                  if (!rtIsNaN(alpha->data[localB->b_i_l - 1])) {
                    localB->idxl = localB->b_i_l;
                    exitg3 = true;
                  } else {
                    localB->b_i_l++;
                  }
                }
              }

              if (localB->idxl == 0) {
                localB->s_j = alpha->data[0];
              } else {
                localB->s_j = alpha->data[localB->idxl - 1];
                localB->nx_n = localB->idxl;
                for (localB->b_i_l = localB->idxl + 1; localB->b_i_l <=
                     localB->n; localB->b_i_l++) {
                  if (localB->s_j < alpha->data[localB->b_i_l - 1]) {
                    localB->s_j = alpha->data[localB->b_i_l - 1];
                    localB->nx_n = localB->b_i_l;
                  }
                }

                localB->idxl = localB->nx_n - 1;
              }
            }

            if (cartesian_trajectory_pla_norm_j(localB->Hg) < 0.5 * localB->s_j)
            {
              localB->nx_n = activeSet->size[0];
              localB->idx = 0;
              localB->b_i_l = ii->size[0];
              ii->size[0] = activeSet->size[0];
              carte_emxEnsureCapacity_int32_T(ii, localB->b_i_l);
              localB->b_i_l = 1;
              exitg3 = false;
              while ((!exitg3) && (localB->b_i_l - 1 <= localB->nx_n - 1)) {
                if (activeSet->data[localB->b_i_l - 1]) {
                  localB->idx++;
                  ii->data[localB->idx - 1] = localB->b_i_l;
                  if (localB->idx >= localB->nx_n) {
                    exitg3 = true;
                  } else {
                    localB->b_i_l++;
                  }
                } else {
                  localB->b_i_l++;
                }
              }

              if (activeSet->size[0] == 1) {
                if (localB->idx == 0) {
                  ii->size[0] = 0;
                }
              } else {
                if (1 > localB->idx) {
                  localB->idx = 0;
                }

                localB->b_i_l = ii_1->size[0];
                ii_1->size[0] = localB->idx;
                carte_emxEnsureCapacity_int32_T(ii_1, localB->b_i_l);
                for (localB->b_i_l = 0; localB->b_i_l < localB->idx;
                     localB->b_i_l++) {
                  ii_1->data[localB->b_i_l] = ii->data[localB->b_i_l];
                }

                localB->b_i_l = ii->size[0];
                ii->size[0] = ii_1->size[0];
                carte_emxEnsureCapacity_int32_T(ii, localB->b_i_l);
                localB->i_i = ii_1->size[0];
                for (localB->b_i_l = 0; localB->b_i_l < localB->i_i;
                     localB->b_i_l++) {
                  ii->data[localB->b_i_l] = ii_1->data[localB->b_i_l];
                }
              }

              localB->b_i_l = alpha->size[0];
              alpha->size[0] = ii->size[0];
              cartes_emxEnsureCapacity_real_T(alpha, localB->b_i_l, localB);
              localB->i_i = ii->size[0];
              for (localB->b_i_l = 0; localB->b_i_l < localB->i_i; localB->b_i_l
                   ++) {
                alpha->data[localB->b_i_l] = ii->data[localB->b_i_l];
              }

              activeSet->data[static_cast<int32_T>(alpha->data[localB->idxl]) -
                1] = false;
              localB->nx_n = activeSet->size[0] - 1;
              localB->idx = 0;
              for (localB->b_i_l = 0; localB->b_i_l <= localB->nx_n;
                   localB->b_i_l++) {
                if (activeSet->data[localB->b_i_l]) {
                  localB->idx++;
                }
              }

              localB->b_i_l = eb->size[0];
              eb->size[0] = localB->idx;
              carte_emxEnsureCapacity_int32_T(eb, localB->b_i_l);
              localB->idx = 0;
              for (localB->b_i_l = 0; localB->b_i_l <= localB->nx_n;
                   localB->b_i_l++) {
                if (activeSet->data[localB->b_i_l]) {
                  eb->data[localB->idx] = localB->b_i_l + 1;
                  localB->idx++;
                }
              }

              localB->i_i = obj->ConstraintMatrix->size[0];
              localB->b_i_l = A->size[0] * A->size[1];
              A->size[0] = localB->i_i;
              A->size[1] = eb->size[0];
              cartes_emxEnsureCapacity_real_T(A, localB->b_i_l, localB);
              localB->n = eb->size[0];
              for (localB->b_i_l = 0; localB->b_i_l < localB->n; localB->b_i_l++)
              {
                for (localB->idx = 0; localB->idx < localB->i_i; localB->idx++)
                {
                  A->data[localB->idx + A->size[0] * localB->b_i_l] =
                    obj->ConstraintMatrix->data[(eb->data[localB->b_i_l] - 1) *
                    obj->ConstraintMatrix->size[0] + localB->idx];
                }
              }

              localB->m_i = A->size[1] - 1;
              localB->inner = A->size[0] - 1;
              localB->n = A->size[1] - 1;
              localB->b_i_l = AIn->size[0] * AIn->size[1];
              AIn->size[0] = A->size[1];
              AIn->size[1] = A->size[1];
              cartes_emxEnsureCapacity_real_T(AIn, localB->b_i_l, localB);
              for (localB->idx = 0; localB->idx <= localB->n; localB->idx++) {
                localB->coffset = (localB->m_i + 1) * localB->idx - 1;
                localB->boffset = localB->idx * A->size[0] - 1;
                for (localB->b_i_l = 0; localB->b_i_l <= localB->m_i;
                     localB->b_i_l++) {
                  AIn->data[(localB->coffset + localB->b_i_l) + 1] = 0.0;
                }

                for (localB->nx_n = 0; localB->nx_n <= localB->inner;
                     localB->nx_n++) {
                  localB->s_j = A->data[(localB->boffset + localB->nx_n) + 1];
                  for (localB->j = 0; localB->j <= localB->m_i; localB->j++) {
                    localB->b_i_l = (localB->coffset + localB->j) + 1;
                    AIn->data[localB->b_i_l] += A->data[localB->j * A->size[0] +
                      localB->nx_n] * localB->s_j;
                  }
                }
              }

              localB->b_i_l = A_3->size[0] * A_3->size[1];
              A_3->size[0] = A->size[1];
              A_3->size[1] = A->size[0];
              cartes_emxEnsureCapacity_real_T(A_3, localB->b_i_l, localB);
              localB->i_i = A->size[0];
              for (localB->b_i_l = 0; localB->b_i_l < localB->i_i; localB->b_i_l
                   ++) {
                localB->n = A->size[1];
                for (localB->idx = 0; localB->idx < localB->n; localB->idx++) {
                  A_3->data[localB->idx + A_3->size[0] * localB->b_i_l] =
                    A->data[A->size[0] * localB->idx + localB->b_i_l];
                }
              }

              cartesian_trajectory_p_mldivide(AIn, A_3, a, localB);
              localB->m_i = A->size[0] - 1;
              localB->inner = A->size[1] - 1;
              localB->n = a->size[1] - 1;
              localB->b_i_l = AIn->size[0] * AIn->size[1];
              AIn->size[0] = A->size[0];
              AIn->size[1] = a->size[1];
              cartes_emxEnsureCapacity_real_T(AIn, localB->b_i_l, localB);
              for (localB->idx = 0; localB->idx <= localB->n; localB->idx++) {
                localB->coffset = (localB->m_i + 1) * localB->idx - 1;
                localB->boffset = localB->idx * a->size[0] - 1;
                for (localB->b_i_l = 0; localB->b_i_l <= localB->m_i;
                     localB->b_i_l++) {
                  AIn->data[(localB->coffset + localB->b_i_l) + 1] = 0.0;
                }

                for (localB->nx_n = 0; localB->nx_n <= localB->inner;
                     localB->nx_n++) {
                  localB->aoffset = localB->nx_n * A->size[0] - 1;
                  localB->s_j = a->data[(localB->boffset + localB->nx_n) + 1];
                  for (localB->j = 0; localB->j <= localB->m_i; localB->j++) {
                    localB->i_i = localB->j + 1;
                    localB->b_i_l = localB->coffset + localB->i_i;
                    AIn->data[localB->b_i_l] += A->data[localB->aoffset +
                      localB->i_i] * localB->s_j;
                  }
                }
              }

              for (localB->b_i_l = 0; localB->b_i_l < 36; localB->b_i_l++) {
                localB->P[localB->b_i_l] = localB->unusedU0[localB->b_i_l] -
                  AIn->data[localB->b_i_l];
              }

              localB->s_j = alpha->data[localB->idxl];
              localB->b_i_l = static_cast<int32_T>(localB->s_j);
              localB->i_i = obj->ConstraintMatrix->size[0];
              localB->idx = alpha->size[0];
              alpha->size[0] = localB->i_i;
              cartes_emxEnsureCapacity_real_T(alpha, localB->idx, localB);
              for (localB->idx = 0; localB->idx < localB->i_i; localB->idx++) {
                alpha->data[localB->idx] = obj->ConstraintMatrix->data
                  [(localB->b_i_l - 1) * obj->ConstraintMatrix->size[0] +
                  localB->idx];
              }

              localB->b_i_l = alpha_0->size[0] * alpha_0->size[1];
              alpha_0->size[0] = 1;
              alpha_0->size[1] = alpha->size[0];
              cartes_emxEnsureCapacity_real_T(alpha_0, localB->b_i_l, localB);
              localB->i_i = alpha->size[0];
              for (localB->b_i_l = 0; localB->b_i_l < localB->i_i; localB->b_i_l
                   ++) {
                alpha_0->data[localB->b_i_l] = alpha->data[localB->b_i_l];
              }

              localB->s_j = 0.0;
              for (localB->b_i_l = 0; localB->b_i_l < 6; localB->b_i_l++) {
                localB->Hg[localB->b_i_l] = 0.0;
                for (localB->idx = 0; localB->idx < 6; localB->idx++) {
                  localB->b_gamma = localB->P[6 * localB->b_i_l + localB->idx] *
                    alpha_0->data[localB->idx] + localB->Hg[localB->b_i_l];
                  localB->Hg[localB->b_i_l] = localB->b_gamma;
                }

                localB->s_j += localB->Hg[localB->b_i_l] * alpha->data
                  [localB->b_i_l];
              }

              localB->s_j = 1.0 / localB->s_j;
              for (localB->b_i_l = 0; localB->b_i_l < 36; localB->b_i_l++) {
                localB->V[localB->b_i_l] = localB->s_j * localB->P[localB->b_i_l];
              }

              localB->b_i_l = AIn->size[0] * AIn->size[1];
              AIn->size[0] = alpha->size[0];
              AIn->size[1] = alpha->size[0];
              cartes_emxEnsureCapacity_real_T(AIn, localB->b_i_l, localB);
              localB->i_i = alpha->size[0];
              for (localB->b_i_l = 0; localB->b_i_l < localB->i_i; localB->b_i_l
                   ++) {
                localB->n = alpha->size[0];
                for (localB->idx = 0; localB->idx < localB->n; localB->idx++) {
                  AIn->data[localB->idx + AIn->size[0] * localB->b_i_l] =
                    alpha->data[localB->idx] * alpha->data[localB->b_i_l];
                }
              }

              localB->n = AIn->size[1] - 1;
              localB->b_i_l = unusedU1->size[0] * unusedU1->size[1];
              unusedU1->size[0] = 6;
              unusedU1->size[1] = AIn->size[1];
              cartes_emxEnsureCapacity_real_T(unusedU1, localB->b_i_l, localB);
              for (localB->idx = 0; localB->idx <= localB->n; localB->idx++) {
                localB->coffset = localB->idx * 6 - 1;
                localB->boffset = localB->idx * AIn->size[0] - 1;
                for (localB->b_i_l = 0; localB->b_i_l < 6; localB->b_i_l++) {
                  localB->s_j = 0.0;
                  for (localB->nx_n = 0; localB->nx_n < 6; localB->nx_n++) {
                    localB->s_j += localB->V[localB->nx_n * 6 + localB->b_i_l] *
                      AIn->data[(localB->boffset + localB->nx_n) + 1];
                  }

                  unusedU1->data[(localB->coffset + localB->b_i_l) + 1] =
                    localB->s_j;
                }
              }

              for (localB->b_i_l = 0; localB->b_i_l < 6; localB->b_i_l++) {
                for (localB->idx = 0; localB->idx < 6; localB->idx++) {
                  localB->s_j = 0.0;
                  for (localB->i_i = 0; localB->i_i < 6; localB->i_i++) {
                    localB->s_j += unusedU1->data[6 * localB->i_i +
                      localB->b_i_l] * localB->P[6 * localB->idx + localB->i_i];
                  }

                  localB->idxl = 6 * localB->idx + localB->b_i_l;
                  localB->H[localB->idxl] += localB->s_j;
                }
              }

              localB->g_idx_0++;
            } else {
              guard2 = true;
            }
          } else {
            guard2 = true;
          }

          if (guard2) {
            for (localB->b_i_l = 0; localB->b_i_l < 6; localB->b_i_l++) {
              localB->Hg[localB->b_i_l] = -localB->Hg[localB->b_i_l];
            }

            localB->idxl = -1;
            if (obj->ConstraintsOn) {
              localB->b_i_l = x->size[0];
              x->size[0] = activeSet->size[0];
              car_emxEnsureCapacity_boolean_T(x, localB->b_i_l);
              localB->i_i = activeSet->size[0];
              for (localB->b_i_l = 0; localB->b_i_l < localB->i_i; localB->b_i_l
                   ++) {
                x->data[localB->b_i_l] = !activeSet->data[localB->b_i_l];
              }

              if (cartesian_trajectory_planne_any(x)) {
                localB->nx_n = activeSet->size[0] - 1;
                localB->idx = 0;
                for (localB->b_i_l = 0; localB->b_i_l <= localB->nx_n;
                     localB->b_i_l++) {
                  if (!activeSet->data[localB->b_i_l]) {
                    localB->idx++;
                  }
                }

                localB->b_i_l = cb->size[0];
                cb->size[0] = localB->idx;
                carte_emxEnsureCapacity_int32_T(cb, localB->b_i_l);
                localB->idx = 0;
                for (localB->b_i_l = 0; localB->b_i_l <= localB->nx_n;
                     localB->b_i_l++) {
                  if (!activeSet->data[localB->b_i_l]) {
                    cb->data[localB->idx] = localB->b_i_l + 1;
                    localB->idx++;
                  }
                }

                localB->b_i_l = alpha->size[0];
                alpha->size[0] = cb->size[0];
                cartes_emxEnsureCapacity_real_T(alpha, localB->b_i_l, localB);
                localB->i_i = cb->size[0];
                for (localB->b_i_l = 0; localB->b_i_l < localB->i_i;
                     localB->b_i_l++) {
                  alpha->data[localB->b_i_l] = obj->ConstraintBound->data
                    [cb->data[localB->b_i_l] - 1];
                }

                localB->nx_n = activeSet->size[0] - 1;
                localB->idx = 0;
                for (localB->b_i_l = 0; localB->b_i_l <= localB->nx_n;
                     localB->b_i_l++) {
                  if (!activeSet->data[localB->b_i_l]) {
                    localB->idx++;
                  }
                }

                localB->b_i_l = db->size[0];
                db->size[0] = localB->idx;
                carte_emxEnsureCapacity_int32_T(db, localB->b_i_l);
                localB->idx = 0;
                for (localB->b_i_l = 0; localB->b_i_l <= localB->nx_n;
                     localB->b_i_l++) {
                  if (!activeSet->data[localB->b_i_l]) {
                    db->data[localB->idx] = localB->b_i_l + 1;
                    localB->idx++;
                  }
                }

                localB->i_i = obj->ConstraintMatrix->size[0];
                localB->b_i_l = AIn->size[0] * AIn->size[1];
                AIn->size[0] = localB->i_i;
                AIn->size[1] = db->size[0];
                cartes_emxEnsureCapacity_real_T(AIn, localB->b_i_l, localB);
                localB->n = db->size[0];
                for (localB->b_i_l = 0; localB->b_i_l < localB->n; localB->b_i_l
                     ++) {
                  for (localB->idx = 0; localB->idx < localB->i_i; localB->idx++)
                  {
                    AIn->data[localB->idx + AIn->size[0] * localB->b_i_l] =
                      obj->ConstraintMatrix->data[(db->data[localB->b_i_l] - 1) *
                      obj->ConstraintMatrix->size[0] + localB->idx];
                  }
                }

                localB->nx_n = x->size[0];
                localB->idx = 0;
                localB->b_i_l = ii->size[0];
                ii->size[0] = x->size[0];
                carte_emxEnsureCapacity_int32_T(ii, localB->b_i_l);
                localB->b_i_l = 1;
                exitg3 = false;
                while ((!exitg3) && (localB->b_i_l - 1 <= localB->nx_n - 1)) {
                  if (x->data[localB->b_i_l - 1]) {
                    localB->idx++;
                    ii->data[localB->idx - 1] = localB->b_i_l;
                    if (localB->idx >= localB->nx_n) {
                      exitg3 = true;
                    } else {
                      localB->b_i_l++;
                    }
                  } else {
                    localB->b_i_l++;
                  }
                }

                if (x->size[0] == 1) {
                  if (localB->idx == 0) {
                    ii->size[0] = 0;
                  }
                } else {
                  if (1 > localB->idx) {
                    localB->idx = 0;
                  }

                  localB->b_i_l = ii_2->size[0];
                  ii_2->size[0] = localB->idx;
                  carte_emxEnsureCapacity_int32_T(ii_2, localB->b_i_l);
                  for (localB->b_i_l = 0; localB->b_i_l < localB->idx;
                       localB->b_i_l++) {
                    ii_2->data[localB->b_i_l] = ii->data[localB->b_i_l];
                  }

                  localB->b_i_l = ii->size[0];
                  ii->size[0] = ii_2->size[0];
                  carte_emxEnsureCapacity_int32_T(ii, localB->b_i_l);
                  localB->i_i = ii_2->size[0];
                  for (localB->b_i_l = 0; localB->b_i_l < localB->i_i;
                       localB->b_i_l++) {
                    ii->data[localB->b_i_l] = ii_2->data[localB->b_i_l];
                  }
                }

                localB->m_i = AIn->size[1] - 1;
                localB->inner = AIn->size[0] - 1;
                localB->b_i_l = L->size[0];
                L->size[0] = AIn->size[1];
                cartes_emxEnsureCapacity_real_T(L, localB->b_i_l, localB);
                for (localB->b_i_l = 0; localB->b_i_l <= localB->m_i;
                     localB->b_i_l++) {
                  L->data[localB->b_i_l] = 0.0;
                }

                for (localB->nx_n = 0; localB->nx_n <= localB->inner;
                     localB->nx_n++) {
                  for (localB->j = 0; localB->j <= localB->m_i; localB->j++) {
                    L->data[localB->j] += AIn->data[localB->j * AIn->size[0] +
                      localB->nx_n] * localB->x[localB->nx_n];
                  }
                }

                localB->m_i = AIn->size[1] - 1;
                localB->inner = AIn->size[0] - 1;
                localB->b_i_l = y->size[0];
                y->size[0] = AIn->size[1];
                cartes_emxEnsureCapacity_real_T(y, localB->b_i_l, localB);
                for (localB->b_i_l = 0; localB->b_i_l <= localB->m_i;
                     localB->b_i_l++) {
                  y->data[localB->b_i_l] = 0.0;
                }

                for (localB->nx_n = 0; localB->nx_n <= localB->inner;
                     localB->nx_n++) {
                  for (localB->j = 0; localB->j <= localB->m_i; localB->j++) {
                    y->data[localB->j] += AIn->data[localB->j * AIn->size[0] +
                      localB->nx_n] * localB->Hg[localB->nx_n];
                  }
                }

                localB->b_i_l = alpha->size[0];
                cartes_emxEnsureCapacity_real_T(alpha, localB->b_i_l, localB);
                localB->i_i = alpha->size[0];
                for (localB->b_i_l = 0; localB->b_i_l < localB->i_i;
                     localB->b_i_l++) {
                  alpha->data[localB->b_i_l] = (alpha->data[localB->b_i_l] -
                    L->data[localB->b_i_l]) / y->data[localB->b_i_l];
                }

                localB->b_i_l = x->size[0];
                x->size[0] = alpha->size[0];
                car_emxEnsureCapacity_boolean_T(x, localB->b_i_l);
                localB->i_i = alpha->size[0];
                for (localB->b_i_l = 0; localB->b_i_l < localB->i_i;
                     localB->b_i_l++) {
                  x->data[localB->b_i_l] = (alpha->data[localB->b_i_l] > 0.0);
                }

                localB->nx_n = x->size[0];
                localB->idx = 0;
                localB->b_i_l = ii_0->size[0];
                ii_0->size[0] = x->size[0];
                carte_emxEnsureCapacity_int32_T(ii_0, localB->b_i_l);
                localB->b_i_l = 1;
                exitg3 = false;
                while ((!exitg3) && (localB->b_i_l - 1 <= localB->nx_n - 1)) {
                  if (x->data[localB->b_i_l - 1]) {
                    localB->idx++;
                    ii_0->data[localB->idx - 1] = localB->b_i_l;
                    if (localB->idx >= localB->nx_n) {
                      exitg3 = true;
                    } else {
                      localB->b_i_l++;
                    }
                  } else {
                    localB->b_i_l++;
                  }
                }

                if (x->size[0] == 1) {
                  if (localB->idx == 0) {
                    ii_0->size[0] = 0;
                  }
                } else {
                  if (1 > localB->idx) {
                    localB->idx = 0;
                  }

                  localB->b_i_l = ii_3->size[0];
                  ii_3->size[0] = localB->idx;
                  carte_emxEnsureCapacity_int32_T(ii_3, localB->b_i_l);
                  for (localB->b_i_l = 0; localB->b_i_l < localB->idx;
                       localB->b_i_l++) {
                    ii_3->data[localB->b_i_l] = ii_0->data[localB->b_i_l];
                  }

                  localB->b_i_l = ii_0->size[0];
                  ii_0->size[0] = ii_3->size[0];
                  carte_emxEnsureCapacity_int32_T(ii_0, localB->b_i_l);
                  localB->i_i = ii_3->size[0];
                  for (localB->b_i_l = 0; localB->b_i_l < localB->i_i;
                       localB->b_i_l++) {
                    ii_0->data[localB->b_i_l] = ii_3->data[localB->b_i_l];
                  }
                }

                localB->b_i_l = L->size[0];
                L->size[0] = ii_0->size[0];
                cartes_emxEnsureCapacity_real_T(L, localB->b_i_l, localB);
                localB->i_i = ii_0->size[0];
                for (localB->b_i_l = 0; localB->b_i_l < localB->i_i;
                     localB->b_i_l++) {
                  L->data[localB->b_i_l] = ii_0->data[localB->b_i_l];
                }

                if (L->size[0] != 0) {
                  localB->nx_n = alpha->size[0] - 1;
                  localB->idx = 0;
                  for (localB->b_i_l = 0; localB->b_i_l <= localB->nx_n;
                       localB->b_i_l++) {
                    if (alpha->data[localB->b_i_l] > 0.0) {
                      localB->idx++;
                    }
                  }

                  localB->b_i_l = fb->size[0];
                  fb->size[0] = localB->idx;
                  carte_emxEnsureCapacity_int32_T(fb, localB->b_i_l);
                  localB->idx = 0;
                  for (localB->b_i_l = 0; localB->b_i_l <= localB->nx_n;
                       localB->b_i_l++) {
                    if (alpha->data[localB->b_i_l] > 0.0) {
                      fb->data[localB->idx] = localB->b_i_l + 1;
                      localB->idx++;
                    }
                  }

                  localB->n = fb->size[0];
                  if (fb->size[0] <= 2) {
                    if (fb->size[0] == 1) {
                      localB->s_j = alpha->data[fb->data[0] - 1];
                      localB->idxl = 0;
                    } else if ((alpha->data[fb->data[0] - 1] > alpha->data
                                [fb->data[1] - 1]) || (rtIsNaN(alpha->data
                                 [fb->data[0] - 1]) && (!rtIsNaN(alpha->data
                                  [fb->data[1] - 1])))) {
                      localB->s_j = alpha->data[fb->data[1] - 1];
                      localB->idxl = 1;
                    } else {
                      localB->s_j = alpha->data[fb->data[0] - 1];
                      localB->idxl = 0;
                    }
                  } else {
                    if (!rtIsNaN(alpha->data[fb->data[0] - 1])) {
                      localB->idxl = 1;
                    } else {
                      localB->idxl = 0;
                      localB->b_i_l = 2;
                      exitg3 = false;
                      while ((!exitg3) && (localB->b_i_l <= fb->size[0])) {
                        if (!rtIsNaN(alpha->data[fb->data[localB->b_i_l - 1] - 1]))
                        {
                          localB->idxl = localB->b_i_l;
                          exitg3 = true;
                        } else {
                          localB->b_i_l++;
                        }
                      }
                    }

                    if (localB->idxl == 0) {
                      localB->s_j = alpha->data[fb->data[0] - 1];
                    } else {
                      localB->s_j = alpha->data[fb->data[localB->idxl - 1] - 1];
                      localB->nx_n = localB->idxl;
                      for (localB->b_i_l = localB->idxl + 1; localB->b_i_l <=
                           localB->n; localB->b_i_l++) {
                        if (localB->s_j > alpha->data[fb->data[localB->b_i_l - 1]
                            - 1]) {
                          localB->s_j = alpha->data[fb->data[localB->b_i_l - 1]
                            - 1];
                          localB->nx_n = localB->b_i_l;
                        }
                      }

                      localB->idxl = localB->nx_n - 1;
                    }
                  }

                  localB->idxl = ii->data[static_cast<int32_T>(L->data
                    [localB->idxl]) - 1];
                } else {
                  localB->s_j = 0.0;
                }
              } else {
                localB->s_j = 0.0;
              }
            } else {
              localB->s_j = 0.0;
            }

            if (localB->s_j > 0.0) {
              if (1.0 < localB->s_j) {
                localB->b_gamma = 1.0;
              } else {
                localB->b_gamma = localB->s_j;
              }
            } else {
              localB->b_gamma = 1.0;
            }

            localB->beta = obj->ArmijoRuleBeta;
            localB->sigma = obj->ArmijoRuleSigma;
            for (localB->b_i_l = 0; localB->b_i_l < 6; localB->b_i_l++) {
              localB->sNew_m[localB->b_i_l] = localB->b_gamma * localB->
                Hg[localB->b_i_l] + localB->x[localB->b_i_l];
            }

            cartesian_IKHelpers_computeCost(localB->sNew_m, obj->ExtraArgs,
              &localB->costNew, localB->V, unusedU1, &c, localB);
            obj->ExtraArgs = c;
            localB->m = 0.0;
            do {
              exitg1 = 0;
              for (localB->i_i = 0; localB->i_i < 6; localB->i_i++) {
                xSol[localB->i_i] = localB->b_gamma * localB->Hg[localB->i_i];
              }

              localB->b_i_l = sigma->size[0] * sigma->size[1];
              sigma->size[0] = 1;
              sigma->size[1] = grad->size[0];
              cartes_emxEnsureCapacity_real_T(sigma, localB->b_i_l, localB);
              localB->i_i = grad->size[0];
              for (localB->b_i_l = 0; localB->b_i_l < localB->i_i; localB->b_i_l
                   ++) {
                sigma->data[localB->b_i_l] = -localB->sigma * grad->data
                  [localB->b_i_l];
              }

              localB->sigma_a = 0.0;
              for (localB->b_i_l = 0; localB->b_i_l < 6; localB->b_i_l++) {
                localB->sigma_a += sigma->data[localB->b_i_l] * xSol
                  [localB->b_i_l];
              }

              if (localB->cost - localB->costNew < localB->sigma_a) {
                localB->flag = (localB->b_gamma < obj->StepTolerance);
                if (localB->flag) {
                  for (localB->i_i = 0; localB->i_i < 6; localB->i_i++) {
                    xSol[localB->i_i] = localB->x[localB->i_i];
                  }

                  *exitFlag = StepSizeBelowMinimum;
                  args = obj->ExtraArgs;
                  for (localB->b_i_l = 0; localB->b_i_l < 36; localB->b_i_l++) {
                    localB->unusedU0[localB->b_i_l] = args->WeightMatrix
                      [localB->b_i_l];
                  }

                  localB->b_i_l = grad->size[0];
                  grad->size[0] = args->ErrTemp->size[0];
                  cartes_emxEnsureCapacity_real_T(grad, localB->b_i_l, localB);
                  localB->i_i = args->ErrTemp->size[0];
                  for (localB->b_i_l = 0; localB->b_i_l < localB->i_i;
                       localB->b_i_l++) {
                    grad->data[localB->b_i_l] = args->ErrTemp->data
                      [localB->b_i_l];
                  }

                  for (localB->b_i_l = 0; localB->b_i_l < 6; localB->b_i_l++) {
                    localB->x[localB->b_i_l] = 0.0;
                    for (localB->idx = 0; localB->idx < 6; localB->idx++) {
                      localB->A_f = localB->unusedU0[6 * localB->idx +
                        localB->b_i_l] * grad->data[localB->idx] + localB->
                        x[localB->b_i_l];
                      localB->x[localB->b_i_l] = localB->A_f;
                    }
                  }

                  *err = cartesian_trajectory_pla_norm_j(localB->x);
                  *iter = static_cast<real_T>(localB->g_idx_0) + 1.0;
                  exitg1 = 1;
                } else {
                  localB->b_gamma *= localB->beta;
                  localB->m++;
                  for (localB->b_i_l = 0; localB->b_i_l < 6; localB->b_i_l++) {
                    localB->sNew_m[localB->b_i_l] = localB->b_gamma * localB->
                      Hg[localB->b_i_l] + localB->x[localB->b_i_l];
                  }

                  cartesian_IKHelpers_computeCost(localB->sNew_m, obj->ExtraArgs,
                    &localB->costNew, localB->V, unusedU1, &d, localB);
                  obj->ExtraArgs = d;
                }
              } else {
                for (localB->b_i_l = 0; localB->b_i_l < 6; localB->b_i_l++) {
                  xSol[localB->b_i_l] += localB->x[localB->b_i_l];
                }

                args = obj->ExtraArgs;
                localB->b_i_l = alpha->size[0];
                alpha->size[0] = args->GradTemp->size[0];
                cartes_emxEnsureCapacity_real_T(alpha, localB->b_i_l, localB);
                localB->i_i = args->GradTemp->size[0];
                for (localB->b_i_l = 0; localB->b_i_l < localB->i_i;
                     localB->b_i_l++) {
                  alpha->data[localB->b_i_l] = args->GradTemp->data
                    [localB->b_i_l];
                }

                exitg1 = 2;
              }
            } while (exitg1 == 0);

            if (exitg1 == 1) {
              exitg2 = 1;
            } else if ((localB->m == 0.0) && (fabs(localB->b_gamma - localB->s_j)
                        < 1.4901161193847656E-8)) {
              localB->i_i = obj->ConstraintMatrix->size[0];
              localB->b_i_l = grad->size[0];
              grad->size[0] = localB->i_i;
              cartes_emxEnsureCapacity_real_T(grad, localB->b_i_l, localB);
              for (localB->b_i_l = 0; localB->b_i_l < localB->i_i; localB->b_i_l
                   ++) {
                grad->data[localB->b_i_l] = obj->ConstraintMatrix->data
                  [(localB->idxl - 1) * obj->ConstraintMatrix->size[0] +
                  localB->b_i_l];
              }

              activeSet->data[localB->idxl - 1] = true;
              localB->nx_n = activeSet->size[0] - 1;
              localB->idx = 0;
              for (localB->b_i_l = 0; localB->b_i_l <= localB->nx_n;
                   localB->b_i_l++) {
                if (activeSet->data[localB->b_i_l]) {
                  localB->idx++;
                }
              }

              localB->b_i_l = gb->size[0];
              gb->size[0] = localB->idx;
              carte_emxEnsureCapacity_int32_T(gb, localB->b_i_l);
              localB->idx = 0;
              for (localB->b_i_l = 0; localB->b_i_l <= localB->nx_n;
                   localB->b_i_l++) {
                if (activeSet->data[localB->b_i_l]) {
                  gb->data[localB->idx] = localB->b_i_l + 1;
                  localB->idx++;
                }
              }

              localB->i_i = obj->ConstraintMatrix->size[0];
              localB->b_i_l = A->size[0] * A->size[1];
              A->size[0] = localB->i_i;
              A->size[1] = gb->size[0];
              cartes_emxEnsureCapacity_real_T(A, localB->b_i_l, localB);
              localB->n = gb->size[0];
              for (localB->b_i_l = 0; localB->b_i_l < localB->n; localB->b_i_l++)
              {
                for (localB->idx = 0; localB->idx < localB->i_i; localB->idx++)
                {
                  A->data[localB->idx + A->size[0] * localB->b_i_l] =
                    obj->ConstraintMatrix->data[(gb->data[localB->b_i_l] - 1) *
                    obj->ConstraintMatrix->size[0] + localB->idx];
                }
              }

              localB->b_i_l = AIn->size[0] * AIn->size[1];
              AIn->size[0] = grad->size[0];
              AIn->size[1] = grad->size[0];
              cartes_emxEnsureCapacity_real_T(AIn, localB->b_i_l, localB);
              localB->i_i = grad->size[0];
              for (localB->b_i_l = 0; localB->b_i_l < localB->i_i; localB->b_i_l
                   ++) {
                localB->n = grad->size[0];
                for (localB->idx = 0; localB->idx < localB->n; localB->idx++) {
                  AIn->data[localB->idx + AIn->size[0] * localB->b_i_l] =
                    grad->data[localB->idx] * grad->data[localB->b_i_l];
                }
              }

              localB->m_i = AIn->size[0] - 1;
              localB->inner = AIn->size[1] - 1;
              localB->b_i_l = y_0->size[0] * y_0->size[1];
              y_0->size[0] = AIn->size[0];
              y_0->size[1] = 6;
              cartes_emxEnsureCapacity_real_T(y_0, localB->b_i_l, localB);
              for (localB->idx = 0; localB->idx < 6; localB->idx++) {
                localB->coffset = (localB->m_i + 1) * localB->idx - 1;
                localB->boffset = localB->idx * 6 - 1;
                for (localB->b_i_l = 0; localB->b_i_l <= localB->m_i;
                     localB->b_i_l++) {
                  y_0->data[(localB->coffset + localB->b_i_l) + 1] = 0.0;
                }

                for (localB->nx_n = 0; localB->nx_n <= localB->inner;
                     localB->nx_n++) {
                  localB->aoffset = localB->nx_n * AIn->size[0] - 1;
                  localB->s_j = localB->H[(localB->boffset + localB->nx_n) + 1];
                  for (localB->j = 0; localB->j <= localB->m_i; localB->j++) {
                    localB->i_i = localB->j + 1;
                    localB->b_i_l = localB->coffset + localB->i_i;
                    y_0->data[localB->b_i_l] += AIn->data[localB->aoffset +
                      localB->i_i] * localB->s_j;
                  }
                }
              }

              localB->b_i_l = grad_1->size[0] * grad_1->size[1];
              grad_1->size[0] = 1;
              grad_1->size[1] = grad->size[0];
              cartes_emxEnsureCapacity_real_T(grad_1, localB->b_i_l, localB);
              localB->i_i = grad->size[0];
              for (localB->b_i_l = 0; localB->b_i_l < localB->i_i; localB->b_i_l
                   ++) {
                grad_1->data[localB->b_i_l] = grad->data[localB->b_i_l];
              }

              localB->beta = 0.0;
              for (localB->b_i_l = 0; localB->b_i_l < 6; localB->b_i_l++) {
                localB->sNew_m[localB->b_i_l] = 0.0;
                for (localB->idx = 0; localB->idx < 6; localB->idx++) {
                  localB->sigma = localB->H[6 * localB->b_i_l + localB->idx] *
                    grad_1->data[localB->idx] + localB->sNew_m[localB->b_i_l];
                  localB->sNew_m[localB->b_i_l] = localB->sigma;
                }

                localB->beta += localB->sNew_m[localB->b_i_l] * grad->
                  data[localB->b_i_l];
              }

              localB->s_j = 1.0 / localB->beta;
              for (localB->b_i_l = 0; localB->b_i_l < 6; localB->b_i_l++) {
                for (localB->idx = 0; localB->idx < 6; localB->idx++) {
                  localB->idxl = localB->b_i_l + 6 * localB->idx;
                  localB->H_m[localB->idxl] = 0.0;
                  for (localB->i_i = 0; localB->i_i < 6; localB->i_i++) {
                    localB->H_m[localB->idxl] += localB->H[6 * localB->i_i +
                      localB->b_i_l] * y_0->data[6 * localB->idx + localB->i_i];
                  }
                }
              }

              for (localB->b_i_l = 0; localB->b_i_l < 36; localB->b_i_l++) {
                localB->H[localB->b_i_l] -= localB->s_j * localB->H_m
                  [localB->b_i_l];
              }

              guard1 = true;
            } else {
              localB->b_i_l = grad->size[0];
              grad->size[0] = alpha->size[0];
              cartes_emxEnsureCapacity_real_T(grad, localB->b_i_l, localB);
              localB->i_i = alpha->size[0];
              for (localB->b_i_l = 0; localB->b_i_l < localB->i_i; localB->b_i_l
                   ++) {
                grad->data[localB->b_i_l] = alpha->data[localB->b_i_l] -
                  grad->data[localB->b_i_l];
              }

              localB->b_gamma = 0.0;
              for (localB->b_i_l = 0; localB->b_i_l < 6; localB->b_i_l++) {
                localB->b_gamma += localB->Hg[localB->b_i_l] * grad->data
                  [localB->b_i_l];
              }

              localB->b_i_l = tmp->size[0] * tmp->size[1];
              tmp->size[0] = 1;
              tmp->size[1] = grad->size[0];
              cartes_emxEnsureCapacity_real_T(tmp, localB->b_i_l, localB);
              localB->i_i = grad->size[0];
              for (localB->b_i_l = 0; localB->b_i_l < localB->i_i; localB->b_i_l
                   ++) {
                tmp->data[localB->b_i_l] = 0.2 * grad->data[localB->b_i_l];
              }

              localB->s_j = 0.0;
              for (localB->b_i_l = 0; localB->b_i_l < 6; localB->b_i_l++) {
                localB->sNew_m[localB->b_i_l] = 0.0;
                for (localB->idx = 0; localB->idx < 6; localB->idx++) {
                  localB->beta = localB->H[6 * localB->b_i_l + localB->idx] *
                    tmp->data[localB->idx] + localB->sNew_m[localB->b_i_l];
                  localB->sNew_m[localB->b_i_l] = localB->beta;
                }

                localB->s_j += localB->sNew_m[localB->b_i_l] * grad->data
                  [localB->b_i_l];
              }

              if (localB->b_gamma < localB->s_j) {
                localB->b_i_l = tmp_0->size[0] * tmp_0->size[1];
                tmp_0->size[0] = 1;
                tmp_0->size[1] = grad->size[0];
                cartes_emxEnsureCapacity_real_T(tmp_0, localB->b_i_l, localB);
                localB->i_i = grad->size[0];
                for (localB->b_i_l = 0; localB->b_i_l < localB->i_i;
                     localB->b_i_l++) {
                  tmp_0->data[localB->b_i_l] = 0.8 * grad->data[localB->b_i_l];
                }

                localB->s_j = 0.0;
                for (localB->b_i_l = 0; localB->b_i_l < 6; localB->b_i_l++) {
                  localB->sNew_m[localB->b_i_l] = 0.0;
                  for (localB->idx = 0; localB->idx < 6; localB->idx++) {
                    localB->beta = localB->H[6 * localB->b_i_l + localB->idx] *
                      tmp_0->data[localB->idx] + localB->sNew_m[localB->b_i_l];
                    localB->sNew_m[localB->b_i_l] = localB->beta;
                  }

                  localB->s_j += localB->sNew_m[localB->b_i_l] * grad->
                    data[localB->b_i_l];
                }

                localB->b_i_l = grad_0->size[0] * grad_0->size[1];
                grad_0->size[0] = 1;
                grad_0->size[1] = grad->size[0];
                cartes_emxEnsureCapacity_real_T(grad_0, localB->b_i_l, localB);
                localB->i_i = grad->size[0];
                for (localB->b_i_l = 0; localB->b_i_l < localB->i_i;
                     localB->b_i_l++) {
                  grad_0->data[localB->b_i_l] = grad->data[localB->b_i_l];
                }

                localB->beta = 0.0;
                localB->b_gamma = 0.0;
                for (localB->b_i_l = 0; localB->b_i_l < 6; localB->b_i_l++) {
                  localB->sNew_m[localB->b_i_l] = 0.0;
                  for (localB->idx = 0; localB->idx < 6; localB->idx++) {
                    localB->sigma = localB->H[6 * localB->b_i_l + localB->idx] *
                      grad_0->data[localB->idx] + localB->sNew_m[localB->b_i_l];
                    localB->sNew_m[localB->b_i_l] = localB->sigma;
                  }

                  localB->beta += localB->sNew_m[localB->b_i_l] * grad->
                    data[localB->b_i_l];
                  localB->b_gamma += localB->Hg[localB->b_i_l] * grad->
                    data[localB->b_i_l];
                }

                localB->b_gamma = localB->s_j / (localB->beta - localB->b_gamma);
              } else {
                localB->b_gamma = 1.0;
              }

              localB->beta = 0.0;
              for (localB->b_i_l = 0; localB->b_i_l < 6; localB->b_i_l++) {
                localB->s_j = 0.0;
                for (localB->idx = 0; localB->idx < 6; localB->idx++) {
                  localB->s_j += localB->H[6 * localB->idx + localB->b_i_l] *
                    (1.0 - localB->b_gamma) * grad->data[localB->idx];
                }

                localB->s_j += localB->b_gamma * localB->Hg[localB->b_i_l];
                localB->beta += localB->s_j * grad->data[localB->b_i_l];
                localB->sNew_m[localB->b_i_l] = localB->s_j;
              }

              localB->b_i_l = sNew->size[0] * sNew->size[1];
              sNew->size[0] = 6;
              sNew->size[1] = grad->size[0];
              cartes_emxEnsureCapacity_real_T(sNew, localB->b_i_l, localB);
              localB->i_i = grad->size[0];
              for (localB->b_i_l = 0; localB->b_i_l < localB->i_i; localB->b_i_l
                   ++) {
                for (localB->idx = 0; localB->idx < 6; localB->idx++) {
                  localB->s_j = localB->sNew_m[localB->idx] * grad->data
                    [localB->b_i_l];
                  sNew->data[localB->idx + 6 * localB->b_i_l] = localB->s_j /
                    localB->beta;
                }
              }

              for (localB->b_i_l = 0; localB->b_i_l < 36; localB->b_i_l++) {
                localB->V[localB->b_i_l] = localB->unusedU0[localB->b_i_l] -
                  sNew->data[localB->b_i_l];
              }

              for (localB->b_i_l = 0; localB->b_i_l < 6; localB->b_i_l++) {
                for (localB->idx = 0; localB->idx < 6; localB->idx++) {
                  localB->nx_n = localB->b_i_l + 6 * localB->idx;
                  localB->H_m[localB->nx_n] = 0.0;
                  for (localB->i_i = 0; localB->i_i < 6; localB->i_i++) {
                    localB->H_m[localB->nx_n] += localB->V[6 * localB->i_i +
                      localB->b_i_l] * localB->H[6 * localB->idx + localB->i_i];
                  }
                }
              }

              for (localB->b_i_l = 0; localB->b_i_l < 6; localB->b_i_l++) {
                for (localB->idx = 0; localB->idx < 6; localB->idx++) {
                  localB->nx_n = localB->b_i_l + 6 * localB->idx;
                  localB->P[localB->nx_n] = 0.0;
                  for (localB->i_i = 0; localB->i_i < 6; localB->i_i++) {
                    localB->P[localB->nx_n] += localB->H_m[6 * localB->i_i +
                      localB->b_i_l] * localB->V[6 * localB->i_i + localB->idx];
                  }
                }
              }

              for (localB->b_i_l = 0; localB->b_i_l < 6; localB->b_i_l++) {
                for (localB->idx = 0; localB->idx < 6; localB->idx++) {
                  localB->sNew[localB->idx + 6 * localB->b_i_l] = localB->
                    sNew_m[localB->idx] * localB->sNew_m[localB->b_i_l] /
                    localB->beta;
                }
              }

              for (localB->b_i_l = 0; localB->b_i_l < 36; localB->b_i_l++) {
                localB->s_j = localB->P[localB->b_i_l] + localB->sNew
                  [localB->b_i_l];
                localB->H_m[localB->b_i_l] = 1.4901161193847656E-8 *
                  static_cast<real_T>(tmp_1[localB->b_i_l]) + localB->s_j;
                localB->H[localB->b_i_l] = localB->s_j;
              }

              if (!cartesian_tr_isPositiveDefinite(localB->H_m, localB)) {
                *exitFlag = HessianNotPositiveSemidefinite;
                args = obj->ExtraArgs;
                for (localB->b_i_l = 0; localB->b_i_l < 36; localB->b_i_l++) {
                  localB->unusedU0[localB->b_i_l] = args->WeightMatrix
                    [localB->b_i_l];
                }

                localB->b_i_l = grad->size[0];
                grad->size[0] = args->ErrTemp->size[0];
                cartes_emxEnsureCapacity_real_T(grad, localB->b_i_l, localB);
                localB->i_i = args->ErrTemp->size[0];
                for (localB->b_i_l = 0; localB->b_i_l < localB->i_i;
                     localB->b_i_l++) {
                  grad->data[localB->b_i_l] = args->ErrTemp->data[localB->b_i_l];
                }

                for (localB->b_i_l = 0; localB->b_i_l < 6; localB->b_i_l++) {
                  localB->x[localB->b_i_l] = 0.0;
                  for (localB->idx = 0; localB->idx < 6; localB->idx++) {
                    localB->A_f = localB->unusedU0[6 * localB->idx +
                      localB->b_i_l] * grad->data[localB->idx] + localB->
                      x[localB->b_i_l];
                    localB->x[localB->b_i_l] = localB->A_f;
                  }
                }

                *err = cartesian_trajectory_pla_norm_j(localB->x);
                *iter = static_cast<real_T>(localB->g_idx_0) + 1.0;
                exitg2 = 1;
              } else {
                guard1 = true;
              }
            }
          }

          if (guard1) {
            if (DampedBFGSwGradientProjection_k(obj, xSol, localB)) {
              for (localB->i_i = 0; localB->i_i < 6; localB->i_i++) {
                xSol[localB->i_i] = localB->x[localB->i_i];
              }

              *exitFlag = SearchDirectionInvalid;
              args = obj->ExtraArgs;
              for (localB->b_i_l = 0; localB->b_i_l < 36; localB->b_i_l++) {
                localB->unusedU0[localB->b_i_l] = args->WeightMatrix
                  [localB->b_i_l];
              }

              localB->b_i_l = grad->size[0];
              grad->size[0] = args->ErrTemp->size[0];
              cartes_emxEnsureCapacity_real_T(grad, localB->b_i_l, localB);
              localB->i_i = args->ErrTemp->size[0];
              for (localB->b_i_l = 0; localB->b_i_l < localB->i_i; localB->b_i_l
                   ++) {
                grad->data[localB->b_i_l] = args->ErrTemp->data[localB->b_i_l];
              }

              for (localB->b_i_l = 0; localB->b_i_l < 6; localB->b_i_l++) {
                localB->x[localB->b_i_l] = 0.0;
                for (localB->idx = 0; localB->idx < 6; localB->idx++) {
                  localB->A_f = localB->unusedU0[6 * localB->idx + localB->b_i_l]
                    * grad->data[localB->idx] + localB->x[localB->b_i_l];
                  localB->x[localB->b_i_l] = localB->A_f;
                }
              }

              *err = cartesian_trajectory_pla_norm_j(localB->x);
              *iter = static_cast<real_T>(localB->g_idx_0) + 1.0;
              exitg2 = 1;
            } else {
              for (localB->i_i = 0; localB->i_i < 6; localB->i_i++) {
                localB->x[localB->i_i] = xSol[localB->i_i];
              }

              localB->b_i_l = grad->size[0];
              grad->size[0] = alpha->size[0];
              cartes_emxEnsureCapacity_real_T(grad, localB->b_i_l, localB);
              localB->i_i = alpha->size[0];
              for (localB->b_i_l = 0; localB->b_i_l < localB->i_i; localB->b_i_l
                   ++) {
                grad->data[localB->b_i_l] = alpha->data[localB->b_i_l];
              }

              localB->cost = localB->costNew;
              localB->g_idx_0++;
            }
          }
        }
      }
    } else {
      *exitFlag = IterationLimitExceeded;
      args = obj->ExtraArgs;
      for (localB->b_i_l = 0; localB->b_i_l < 36; localB->b_i_l++) {
        localB->unusedU0[localB->b_i_l] = args->WeightMatrix[localB->b_i_l];
      }

      localB->b_i_l = grad->size[0];
      grad->size[0] = args->ErrTemp->size[0];
      cartes_emxEnsureCapacity_real_T(grad, localB->b_i_l, localB);
      localB->i_i = args->ErrTemp->size[0];
      for (localB->b_i_l = 0; localB->b_i_l < localB->i_i; localB->b_i_l++) {
        grad->data[localB->b_i_l] = args->ErrTemp->data[localB->b_i_l];
      }

      for (localB->b_i_l = 0; localB->b_i_l < 6; localB->b_i_l++) {
        localB->x[localB->b_i_l] = 0.0;
        for (localB->idx = 0; localB->idx < 6; localB->idx++) {
          localB->A_f = localB->unusedU0[6 * localB->idx + localB->b_i_l] *
            grad->data[localB->idx] + localB->x[localB->b_i_l];
          localB->x[localB->b_i_l] = localB->A_f;
        }
      }

      *err = cartesian_trajectory_pla_norm_j(localB->x);
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

static void cartesian_trajectory_pla_rand_m(real_T varargin_1,
  emxArray_real_T_cartesian_tra_T *r, B_MATLABSystem_cartesian_traj_T *localB,
  DW_MATLABSystem_cartesian_tra_T *localDW)
{
  localB->b_k_f = r->size[0];
  localB->d_p = static_cast<int32_T>(varargin_1);
  r->size[0] = localB->d_p;
  cartes_emxEnsureCapacity_real_T(r, localB->b_k_f, localB);
  localB->d_p--;
  for (localB->b_k_f = 0; localB->b_k_f <= localB->d_p; localB->b_k_f++) {
    memcpy(&localB->uv1[0], &localDW->state_e[0], 625U * sizeof(uint32_T));
    cartesian__eml_rand_mt19937ar_a(localB->uv1, localDW->state_e, &r->
      data[localB->b_k_f], localB);
  }
}

static real_T cartesian_trajectory_genrandu_n(uint32_T mt[625],
  B_MATLABSystem_cartesian_traj_T *localB)
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
    cartesi_genrand_uint32_vector_d(mt, localB->b_u_o, localB);
    r = (static_cast<real_T>(localB->b_u_o[0] >> 5U) * 6.7108864E+7 +
         static_cast<real_T>(localB->b_u_o[1] >> 6U)) * 1.1102230246251565E-16;
    if (r == 0.0) {
      if (!cartesian_trajec_is_valid_state(mt, localB)) {
        localB->r_k = 5489U;
        mt[0] = 5489U;
        for (localB->b_mti_i = 0; localB->b_mti_i < 623; localB->b_mti_i++) {
          localB->r_k = ((localB->r_k >> 30U ^ localB->r_k) * 1812433253U +
                         localB->b_mti_i) + 1U;
          mt[localB->b_mti_i + 1] = localB->r_k;
        }

        mt[624] = 624U;
      }
    } else {
      exitg1 = 1;
    }
  } while (exitg1 == 0);

  return r;
}

static real_T cartesia_eml_rand_mt19937ar_aoi(uint32_T state[625],
  B_MATLABSystem_cartesian_traj_T *localB)
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
  localB->xi[0] = 0.0;
  localB->xi[1] = 0.215241895984875;
  localB->xi[2] = 0.286174591792068;
  localB->xi[3] = 0.335737519214422;
  localB->xi[4] = 0.375121332878378;
  localB->xi[5] = 0.408389134611989;
  localB->xi[6] = 0.43751840220787;
  localB->xi[7] = 0.46363433679088;
  localB->xi[8] = 0.487443966139235;
  localB->xi[9] = 0.50942332960209;
  localB->xi[10] = 0.529909720661557;
  localB->xi[11] = 0.549151702327164;
  localB->xi[12] = 0.567338257053817;
  localB->xi[13] = 0.584616766106378;
  localB->xi[14] = 0.601104617755991;
  localB->xi[15] = 0.61689699000775;
  localB->xi[16] = 0.63207223638606;
  localB->xi[17] = 0.646695714894993;
  localB->xi[18] = 0.660822574244419;
  localB->xi[19] = 0.674499822837293;
  localB->xi[20] = 0.687767892795788;
  localB->xi[21] = 0.700661841106814;
  localB->xi[22] = 0.713212285190975;
  localB->xi[23] = 0.725446140909999;
  localB->xi[24] = 0.737387211434295;
  localB->xi[25] = 0.749056662017815;
  localB->xi[26] = 0.760473406430107;
  localB->xi[27] = 0.771654424224568;
  localB->xi[28] = 0.782615023307232;
  localB->xi[29] = 0.793369058840623;
  localB->xi[30] = 0.80392911698997;
  localB->xi[31] = 0.814306670135215;
  localB->xi[32] = 0.824512208752291;
  localB->xi[33] = 0.834555354086381;
  localB->xi[34] = 0.844444954909153;
  localB->xi[35] = 0.854189171008163;
  localB->xi[36] = 0.863795545553308;
  localB->xi[37] = 0.87327106808886;
  localB->xi[38] = 0.882622229585165;
  localB->xi[39] = 0.891855070732941;
  localB->xi[40] = 0.900975224461221;
  localB->xi[41] = 0.909987953496718;
  localB->xi[42] = 0.91889818364959;
  localB->xi[43] = 0.927710533401999;
  localB->xi[44] = 0.936429340286575;
  localB->xi[45] = 0.945058684468165;
  localB->xi[46] = 0.953602409881086;
  localB->xi[47] = 0.96206414322304;
  localB->xi[48] = 0.970447311064224;
  localB->xi[49] = 0.978755155294224;
  localB->xi[50] = 0.986990747099062;
  localB->xi[51] = 0.99515699963509;
  localB->xi[52] = 1.00325667954467;
  localB->xi[53] = 1.01129241744;
  localB->xi[54] = 1.01926671746548;
  localB->xi[55] = 1.02718196603564;
  localB->xi[56] = 1.03504043983344;
  localB->xi[57] = 1.04284431314415;
  localB->xi[58] = 1.05059566459093;
  localB->xi[59] = 1.05829648333067;
  localB->xi[60] = 1.06594867476212;
  localB->xi[61] = 1.07355406579244;
  localB->xi[62] = 1.0811144097034;
  localB->xi[63] = 1.08863139065398;
  localB->xi[64] = 1.09610662785202;
  localB->xi[65] = 1.10354167942464;
  localB->xi[66] = 1.11093804601357;
  localB->xi[67] = 1.11829717411934;
  localB->xi[68] = 1.12562045921553;
  localB->xi[69] = 1.13290924865253;
  localB->xi[70] = 1.14016484436815;
  localB->xi[71] = 1.14738850542085;
  localB->xi[72] = 1.15458145035993;
  localB->xi[73] = 1.16174485944561;
  localB->xi[74] = 1.16887987673083;
  localB->xi[75] = 1.17598761201545;
  localB->xi[76] = 1.18306914268269;
  localB->xi[77] = 1.19012551542669;
  localB->xi[78] = 1.19715774787944;
  localB->xi[79] = 1.20416683014438;
  localB->xi[80] = 1.2111537262437;
  localB->xi[81] = 1.21811937548548;
  localB->xi[82] = 1.22506469375653;
  localB->xi[83] = 1.23199057474614;
  localB->xi[84] = 1.23889789110569;
  localB->xi[85] = 1.24578749554863;
  localB->xi[86] = 1.2526602218949;
  localB->xi[87] = 1.25951688606371;
  localB->xi[88] = 1.26635828701823;
  localB->xi[89] = 1.27318520766536;
  localB->xi[90] = 1.27999841571382;
  localB->xi[91] = 1.28679866449324;
  localB->xi[92] = 1.29358669373695;
  localB->xi[93] = 1.30036323033084;
  localB->xi[94] = 1.30712898903073;
  localB->xi[95] = 1.31388467315022;
  localB->xi[96] = 1.32063097522106;
  localB->xi[97] = 1.32736857762793;
  localB->xi[98] = 1.33409815321936;
  localB->xi[99] = 1.3408203658964;
  localB->xi[100] = 1.34753587118059;
  localB->xi[101] = 1.35424531676263;
  localB->xi[102] = 1.36094934303328;
  localB->xi[103] = 1.36764858359748;
  localB->xi[104] = 1.37434366577317;
  localB->xi[105] = 1.38103521107586;
  localB->xi[106] = 1.38772383568998;
  localB->xi[107] = 1.39441015092814;
  localB->xi[108] = 1.40109476367925;
  localB->xi[109] = 1.4077782768464;
  localB->xi[110] = 1.41446128977547;
  localB->xi[111] = 1.42114439867531;
  localB->xi[112] = 1.42782819703026;
  localB->xi[113] = 1.43451327600589;
  localB->xi[114] = 1.44120022484872;
  localB->xi[115] = 1.44788963128058;
  localB->xi[116] = 1.45458208188841;
  localB->xi[117] = 1.46127816251028;
  localB->xi[118] = 1.46797845861808;
  localB->xi[119] = 1.47468355569786;
  localB->xi[120] = 1.48139403962819;
  localB->xi[121] = 1.48811049705745;
  localB->xi[122] = 1.49483351578049;
  localB->xi[123] = 1.50156368511546;
  localB->xi[124] = 1.50830159628131;
  localB->xi[125] = 1.51504784277671;
  localB->xi[126] = 1.521803020761;
  localB->xi[127] = 1.52856772943771;
  localB->xi[128] = 1.53534257144151;
  localB->xi[129] = 1.542128153229;
  localB->xi[130] = 1.54892508547417;
  localB->xi[131] = 1.55573398346918;
  localB->xi[132] = 1.56255546753104;
  localB->xi[133] = 1.56939016341512;
  localB->xi[134] = 1.57623870273591;
  localB->xi[135] = 1.58310172339603;
  localB->xi[136] = 1.58997987002419;
  localB->xi[137] = 1.59687379442279;
  localB->xi[138] = 1.60378415602609;
  localB->xi[139] = 1.61071162236983;
  localB->xi[140] = 1.61765686957301;
  localB->xi[141] = 1.62462058283303;
  localB->xi[142] = 1.63160345693487;
  localB->xi[143] = 1.63860619677555;
  localB->xi[144] = 1.64562951790478;
  localB->xi[145] = 1.65267414708306;
  localB->xi[146] = 1.65974082285818;
  localB->xi[147] = 1.66683029616166;
  localB->xi[148] = 1.67394333092612;
  localB->xi[149] = 1.68108070472517;
  localB->xi[150] = 1.68824320943719;
  localB->xi[151] = 1.69543165193456;
  localB->xi[152] = 1.70264685479992;
  localB->xi[153] = 1.7098896570713;
  localB->xi[154] = 1.71716091501782;
  localB->xi[155] = 1.72446150294804;
  localB->xi[156] = 1.73179231405296;
  localB->xi[157] = 1.73915426128591;
  localB->xi[158] = 1.74654827828172;
  localB->xi[159] = 1.75397532031767;
  localB->xi[160] = 1.76143636531891;
  localB->xi[161] = 1.76893241491127;
  localB->xi[162] = 1.77646449552452;
  localB->xi[163] = 1.78403365954944;
  localB->xi[164] = 1.79164098655216;
  localB->xi[165] = 1.79928758454972;
  localB->xi[166] = 1.80697459135082;
  localB->xi[167] = 1.81470317596628;
  localB->xi[168] = 1.82247454009388;
  localB->xi[169] = 1.83028991968276;
  localB->xi[170] = 1.83815058658281;
  localB->xi[171] = 1.84605785028518;
  localB->xi[172] = 1.8540130597602;
  localB->xi[173] = 1.86201760539967;
  localB->xi[174] = 1.87007292107127;
  localB->xi[175] = 1.878180486293;
  localB->xi[176] = 1.88634182853678;
  localB->xi[177] = 1.8945585256707;
  localB->xi[178] = 1.90283220855043;
  localB->xi[179] = 1.91116456377125;
  localB->xi[180] = 1.91955733659319;
  localB->xi[181] = 1.92801233405266;
  localB->xi[182] = 1.93653142827569;
  localB->xi[183] = 1.94511656000868;
  localB->xi[184] = 1.95376974238465;
  localB->xi[185] = 1.96249306494436;
  localB->xi[186] = 1.97128869793366;
  localB->xi[187] = 1.98015889690048;
  localB->xi[188] = 1.98910600761744;
  localB->xi[189] = 1.99813247135842;
  localB->xi[190] = 2.00724083056053;
  localB->xi[191] = 2.0164337349062;
  localB->xi[192] = 2.02571394786385;
  localB->xi[193] = 2.03508435372962;
  localB->xi[194] = 2.04454796521753;
  localB->xi[195] = 2.05410793165065;
  localB->xi[196] = 2.06376754781173;
  localB->xi[197] = 2.07353026351874;
  localB->xi[198] = 2.0833996939983;
  localB->xi[199] = 2.09337963113879;
  localB->xi[200] = 2.10347405571488;
  localB->xi[201] = 2.11368715068665;
  localB->xi[202] = 2.12402331568952;
  localB->xi[203] = 2.13448718284602;
  localB->xi[204] = 2.14508363404789;
  localB->xi[205] = 2.15581781987674;
  localB->xi[206] = 2.16669518035431;
  localB->xi[207] = 2.17772146774029;
  localB->xi[208] = 2.18890277162636;
  localB->xi[209] = 2.20024554661128;
  localB->xi[210] = 2.21175664288416;
  localB->xi[211] = 2.22344334009251;
  localB->xi[212] = 2.23531338492992;
  localB->xi[213] = 2.24737503294739;
  localB->xi[214] = 2.25963709517379;
  localB->xi[215] = 2.27210899022838;
  localB->xi[216] = 2.28480080272449;
  localB->xi[217] = 2.29772334890286;
  localB->xi[218] = 2.31088825060137;
  localB->xi[219] = 2.32430801887113;
  localB->xi[220] = 2.33799614879653;
  localB->xi[221] = 2.35196722737914;
  localB->xi[222] = 2.36623705671729;
  localB->xi[223] = 2.38082279517208;
  localB->xi[224] = 2.39574311978193;
  localB->xi[225] = 2.41101841390112;
  localB->xi[226] = 2.42667098493715;
  localB->xi[227] = 2.44272531820036;
  localB->xi[228] = 2.4592083743347;
  localB->xi[229] = 2.47614993967052;
  localB->xi[230] = 2.49358304127105;
  localB->xi[231] = 2.51154444162669;
  localB->xi[232] = 2.53007523215985;
  localB->xi[233] = 2.54922155032478;
  localB->xi[234] = 2.56903545268184;
  localB->xi[235] = 2.58957598670829;
  localB->xi[236] = 2.61091051848882;
  localB->xi[237] = 2.63311639363158;
  localB->xi[238] = 2.65628303757674;
  localB->xi[239] = 2.68051464328574;
  localB->xi[240] = 2.70593365612306;
  localB->xi[241] = 2.73268535904401;
  localB->xi[242] = 2.76094400527999;
  localB->xi[243] = 2.79092117400193;
  localB->xi[244] = 2.82287739682644;
  localB->xi[245] = 2.85713873087322;
  localB->xi[246] = 2.89412105361341;
  localB->xi[247] = 2.93436686720889;
  localB->xi[248] = 2.97860327988184;
  localB->xi[249] = 3.02783779176959;
  localB->xi[250] = 3.08352613200214;
  localB->xi[251] = 3.147889289518;
  localB->xi[252] = 3.2245750520478;
  localB->xi[253] = 3.32024473383983;
  localB->xi[254] = 3.44927829856143;
  localB->xi[255] = 3.65415288536101;
  localB->xi[256] = 3.91075795952492;
  fitab = &tmp[0];
  do {
    exitg1 = 0;
    cartesi_genrand_uint32_vector_d(state, localB->u32, localB);
    localB->i_fn = static_cast<int32_T>((localB->u32[1] >> 24U) + 1U);
    r = ((static_cast<real_T>(localB->u32[0] >> 3U) * 1.6777216E+7 +
          static_cast<real_T>(static_cast<int32_T>(localB->u32[1]) & 16777215)) *
         2.2204460492503131E-16 - 1.0) * localB->xi[localB->i_fn];
    if (fabs(r) <= localB->xi[localB->i_fn - 1]) {
      exitg1 = 1;
    } else if (localB->i_fn < 256) {
      localB->x_j = cartesian_trajectory_genrandu_n(state, localB);
      if ((fitab[localB->i_fn - 1] - fitab[localB->i_fn]) * localB->x_j +
          fitab[localB->i_fn] < exp(-0.5 * r * r)) {
        exitg1 = 1;
      }
    } else {
      do {
        localB->x_j = cartesian_trajectory_genrandu_n(state, localB);
        localB->x_j = log(localB->x_j) * 0.273661237329758;
        localB->d_u = cartesian_trajectory_genrandu_n(state, localB);
      } while (!(-2.0 * log(localB->d_u) > localB->x_j * localB->x_j));

      if (r < 0.0) {
        r = localB->x_j - 3.65415288536101;
      } else {
        r = 3.65415288536101 - localB->x_j;
      }

      exitg1 = 1;
    }
  } while (exitg1 == 0);

  return r;
}

static void cartesian_trajectory_plan_randn(const real_T varargin_1[2],
  emxArray_real_T_cartesian_tra_T *r, B_MATLABSystem_cartesian_traj_T *localB,
  DW_MATLABSystem_cartesian_tra_T *localDW)
{
  localB->b_k_hp = r->size[0] * r->size[1];
  r->size[0] = static_cast<int32_T>(varargin_1[0]);
  r->size[1] = 1;
  cartes_emxEnsureCapacity_real_T(r, localB->b_k_hp, localB);
  localB->d_h = r->size[0] - 1;
  for (localB->b_k_hp = 0; localB->b_k_hp <= localB->d_h; localB->b_k_hp++) {
    r->data[localB->b_k_hp] = cartesia_eml_rand_mt19937ar_aoi(localDW->state_e,
      localB);
  }
}

static void cartes_NLPSolverInterface_solve(h_robotics_core_internal_Damp_T *obj,
  const real_T seed[6], real_T xSol[6], real_T *solutionInfo_Iterations, real_T *
  solutionInfo_RRAttempts, real_T *solutionInfo_Error, real_T
  *solutionInfo_ExitFlag, char_T solutionInfo_Status_data[], int32_T
  solutionInfo_Status_size[2], B_MATLABSystem_cartesian_traj_T *localB,
  DW_MATLABSystem_cartesian_tra_T *localDW)
{
  emxArray_real_T_cartesian_tra_T *newseed;
  f_robotics_manip_internal_IKE_T *args;
  x_robotics_manip_internal_Rig_T *obj_0;
  emxArray_real_T_cartesian_tra_T *qi;
  c_rigidBodyJoint_cartesian_tr_T *obj_1;
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
  for (localB->i_fq = 0; localB->i_fq < 6; localB->i_fq++) {
    obj->SeedInternal[localB->i_fq] = seed[localB->i_fq];
  }

  localB->tol = obj->SolutionTolerance;
  obj->TimeObj.StartTime = ctimefun();
  DampedBFGSwGradientProjection_s(obj, xSol, &localB->exitFlag, &localB->err,
    &localB->iter, localB);
  *solutionInfo_RRAttempts = 0.0;
  *solutionInfo_Iterations = localB->iter;
  *solutionInfo_Error = localB->err;
  localB->exitFlagPrev = localB->exitFlag;
  cartesian_trajec_emxInit_real_T(&newseed, 1, localB);
  cartesian_trajec_emxInit_real_T(&qi, 2, localB);
  cartesian_trajec_emxInit_real_T(&ub, 1, localB);
  cartesian_trajec_emxInit_real_T(&lb, 1, localB);
  cartesian_trajec_emxInit_real_T(&rn, 1, localB);
  cartesian_trajec_emxInit_real_T(&e, 2, localB);
  cartesian_tra_emxInit_boolean_T(&x, 1);
  cartesian_tra_emxInit_boolean_T(&x_tmp, 1);
  cartesian_tra_emxInit_boolean_T(&x_tmp_0, 1);
  cartesian_tra_emxInit_boolean_T(&x_0, 1);
  exitg1 = false;
  while ((!exitg1) && (obj->RandomRestart && (localB->err > localB->tol))) {
    obj->MaxNumIterationInternal -= localB->iter;
    localB->err = ctimefun();
    localB->err -= obj->TimeObj.StartTime;
    obj->MaxTimeInternal = obj->MaxTime - localB->err;
    if (obj->MaxNumIterationInternal <= 0.0) {
      localB->exitFlag = IterationLimitExceeded;
    }

    if ((localB->exitFlag == IterationLimitExceeded) || (localB->exitFlag ==
         TimeLimitExceeded)) {
      localB->exitFlagPrev = localB->exitFlag;
      exitg1 = true;
    } else {
      args = obj->ExtraArgs;
      obj_0 = args->Robot;
      localB->ix = newseed->size[0];
      newseed->size[0] = static_cast<int32_T>(obj_0->PositionNumber);
      cartes_emxEnsureCapacity_real_T(newseed, localB->ix, localB);
      localB->nx = static_cast<int32_T>(obj_0->PositionNumber);
      for (localB->ix = 0; localB->ix < localB->nx; localB->ix++) {
        newseed->data[localB->ix] = 0.0;
      }

      localB->err = obj_0->NumBodies;
      localB->c_c = static_cast<int32_T>(localB->err) - 1;
      for (localB->i_fq = 0; localB->i_fq <= localB->c_c; localB->i_fq++) {
        localB->err = obj_0->PositionDoFMap[localB->i_fq];
        localB->iter = obj_0->PositionDoFMap[localB->i_fq + 8];
        if (localB->err <= localB->iter) {
          obj_1 = obj_0->Bodies[localB->i_fq]->JointInternal;
          if (static_cast<int32_T>(obj_1->PositionNumber) == 0) {
            localB->ix = qi->size[0] * qi->size[1];
            qi->size[0] = 1;
            qi->size[1] = 1;
            cartes_emxEnsureCapacity_real_T(qi, localB->ix, localB);
            qi->data[0] = (rtNaN);
          } else {
            localB->nx = obj_1->PositionLimitsInternal->size[0];
            localB->ix = ub->size[0];
            ub->size[0] = localB->nx;
            cartes_emxEnsureCapacity_real_T(ub, localB->ix, localB);
            for (localB->ix = 0; localB->ix < localB->nx; localB->ix++) {
              ub->data[localB->ix] = obj_1->PositionLimitsInternal->data
                [localB->ix + obj_1->PositionLimitsInternal->size[0]];
            }

            localB->nx = obj_1->PositionLimitsInternal->size[0];
            localB->ix = lb->size[0];
            lb->size[0] = localB->nx;
            cartes_emxEnsureCapacity_real_T(lb, localB->ix, localB);
            for (localB->ix = 0; localB->ix < localB->nx; localB->ix++) {
              lb->data[localB->ix] = obj_1->PositionLimitsInternal->data
                [localB->ix];
            }

            cartesian_trajectory_p_isfinite(lb, x_tmp);
            localB->y_pd = true;
            localB->ix = 0;
            exitg2 = false;
            while ((!exitg2) && (localB->ix + 1 <= x_tmp->size[0])) {
              if (!x_tmp->data[localB->ix]) {
                localB->y_pd = false;
                exitg2 = true;
              } else {
                localB->ix++;
              }
            }

            guard1 = false;
            guard2 = false;
            guard3 = false;
            if (localB->y_pd) {
              cartesian_trajectory_p_isfinite(ub, x);
              localB->y_pd = true;
              localB->ix = 0;
              exitg2 = false;
              while ((!exitg2) && (localB->ix + 1 <= x->size[0])) {
                if (!x->data[localB->ix]) {
                  localB->y_pd = false;
                  exitg2 = true;
                } else {
                  localB->ix++;
                }
              }

              if (localB->y_pd) {
                cartesian_trajectory_pla_rand_m(obj_1->PositionNumber, rn,
                  localB, localDW);
                localB->ix = qi->size[0] * qi->size[1];
                qi->size[0] = lb->size[0];
                qi->size[1] = 1;
                cartes_emxEnsureCapacity_real_T(qi, localB->ix, localB);
                localB->nx = lb->size[0] - 1;
                for (localB->ix = 0; localB->ix <= localB->nx; localB->ix++) {
                  qi->data[localB->ix] = (ub->data[localB->ix] - lb->data
                    [localB->ix]) * rn->data[localB->ix] + lb->data[localB->ix];
                }
              } else {
                guard3 = true;
              }
            } else {
              guard3 = true;
            }

            if (guard3) {
              localB->y_pd = true;
              localB->ix = 0;
              exitg2 = false;
              while ((!exitg2) && (localB->ix + 1 <= x_tmp->size[0])) {
                if (!x_tmp->data[localB->ix]) {
                  localB->y_pd = false;
                  exitg2 = true;
                } else {
                  localB->ix++;
                }
              }

              if (localB->y_pd) {
                cartesian_trajectory_p_isfinite(ub, x);
                localB->ix = x_0->size[0];
                x_0->size[0] = x->size[0];
                car_emxEnsureCapacity_boolean_T(x_0, localB->ix);
                localB->nx = x->size[0];
                for (localB->ix = 0; localB->ix < localB->nx; localB->ix++) {
                  x_0->data[localB->ix] = !x->data[localB->ix];
                }

                if (cartesian_trajectory_planne_any(x_0)) {
                  localB->ub[0] = lb->size[0];
                  localB->ub[1] = 1.0;
                  cartesian_trajectory_plan_randn(localB->ub, qi, localB,
                    localDW);
                  localB->nx = qi->size[0] - 1;
                  localB->ix = e->size[0] * e->size[1];
                  e->size[0] = qi->size[0];
                  e->size[1] = 1;
                  cartes_emxEnsureCapacity_real_T(e, localB->ix, localB);
                  for (localB->ix = 0; localB->ix <= localB->nx; localB->ix++) {
                    e->data[localB->ix] = fabs(qi->data[localB->ix]);
                  }

                  localB->ix = qi->size[0] * qi->size[1];
                  qi->size[0] = lb->size[0];
                  qi->size[1] = 1;
                  cartes_emxEnsureCapacity_real_T(qi, localB->ix, localB);
                  localB->nx = lb->size[0] - 1;
                  for (localB->ix = 0; localB->ix <= localB->nx; localB->ix++) {
                    qi->data[localB->ix] = lb->data[localB->ix] + e->data
                      [localB->ix];
                  }
                } else {
                  guard2 = true;
                }
              } else {
                guard2 = true;
              }
            }

            if (guard2) {
              localB->ix = x_tmp_0->size[0];
              x_tmp_0->size[0] = x_tmp->size[0];
              car_emxEnsureCapacity_boolean_T(x_tmp_0, localB->ix);
              localB->nx = x_tmp->size[0];
              for (localB->ix = 0; localB->ix < localB->nx; localB->ix++) {
                x_tmp_0->data[localB->ix] = !x_tmp->data[localB->ix];
              }

              if (cartesian_trajectory_planne_any(x_tmp_0)) {
                cartesian_trajectory_p_isfinite(ub, x);
                localB->y_pd = true;
                localB->ix = 0;
                exitg2 = false;
                while ((!exitg2) && (localB->ix + 1 <= x->size[0])) {
                  if (!x->data[localB->ix]) {
                    localB->y_pd = false;
                    exitg2 = true;
                  } else {
                    localB->ix++;
                  }
                }

                if (localB->y_pd) {
                  localB->ub[0] = ub->size[0];
                  localB->ub[1] = 1.0;
                  cartesian_trajectory_plan_randn(localB->ub, qi, localB,
                    localDW);
                  localB->nx = qi->size[0] - 1;
                  localB->ix = e->size[0] * e->size[1];
                  e->size[0] = qi->size[0];
                  e->size[1] = 1;
                  cartes_emxEnsureCapacity_real_T(e, localB->ix, localB);
                  for (localB->ix = 0; localB->ix <= localB->nx; localB->ix++) {
                    e->data[localB->ix] = fabs(qi->data[localB->ix]);
                  }

                  localB->ix = qi->size[0] * qi->size[1];
                  qi->size[0] = ub->size[0];
                  qi->size[1] = 1;
                  cartes_emxEnsureCapacity_real_T(qi, localB->ix, localB);
                  localB->nx = ub->size[0] - 1;
                  for (localB->ix = 0; localB->ix <= localB->nx; localB->ix++) {
                    qi->data[localB->ix] = ub->data[localB->ix] - e->data
                      [localB->ix];
                  }
                } else {
                  guard1 = true;
                }
              } else {
                guard1 = true;
              }
            }

            if (guard1) {
              localB->ub[0] = ub->size[0];
              localB->ub[1] = 1.0;
              cartesian_trajectory_plan_randn(localB->ub, qi, localB, localDW);
            }
          }

          if (localB->err > localB->iter) {
            localB->nx = 0;
            localB->ix = 0;
          } else {
            localB->nx = static_cast<int32_T>(localB->err) - 1;
            localB->ix = static_cast<int32_T>(localB->iter);
          }

          localB->unnamed_idx_1 = localB->ix - localB->nx;
          for (localB->ix = 0; localB->ix < localB->unnamed_idx_1; localB->ix++)
          {
            newseed->data[localB->nx + localB->ix] = qi->data[localB->ix];
          }
        }
      }

      for (localB->ix = 0; localB->ix < 6; localB->ix++) {
        obj->SeedInternal[localB->ix] = newseed->data[localB->ix];
      }

      DampedBFGSwGradientProjection_s(obj, localB->c_xSol, &localB->exitFlag,
        &localB->err, &localB->iter, localB);
      if (localB->err < *solutionInfo_Error) {
        for (localB->i_fq = 0; localB->i_fq < 6; localB->i_fq++) {
          xSol[localB->i_fq] = localB->c_xSol[localB->i_fq];
        }

        *solutionInfo_Error = localB->err;
        localB->exitFlagPrev = localB->exitFlag;
      }

      (*solutionInfo_RRAttempts)++;
      *solutionInfo_Iterations += localB->iter;
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
  *solutionInfo_ExitFlag = localB->exitFlagPrev;
  if (*solutionInfo_Error < localB->tol) {
    solutionInfo_Status_size[0] = 1;
    solutionInfo_Status_size[1] = 7;
    for (localB->ix = 0; localB->ix < 7; localB->ix++) {
      solutionInfo_Status_data[localB->ix] = tmp_0[localB->ix];
    }
  } else {
    solutionInfo_Status_size[0] = 1;
    solutionInfo_Status_size[1] = 14;
    for (localB->ix = 0; localB->ix < 14; localB->ix++) {
      solutionInfo_Status_data[localB->ix] = tmp[localB->ix];
    }
  }
}

static void cart_inverseKinematics_stepImpl(b_inverseKinematics_cartesian_T *obj,
  const real_T tform[16], const real_T weights[6], const real_T initialGuess[6],
  real_T QSol[6], real_T *solutionInfo_Iterations, real_T
  *solutionInfo_PoseErrorNorm, real_T *solutionInfo_ExitFlag, char_T
  solutionInfo_Status_data[], int32_T solutionInfo_Status_size[2],
  B_MATLABSystem_cartesian_traj_T *localB, DW_MATLABSystem_cartesian_tra_T
  *localDW)
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
  c_inverseKinematics_setPoseGoal(obj, tform, weights, localB);
  for (localB->i_f = 0; localB->i_f < 6; localB->i_f++) {
    QSol[localB->i_f] = initialGuess[localB->i_f];
  }

  cartesian_trajec_emxInit_char_T(&endEffectorName, 2, localB);
  RigidBodyTree_validateConfigu_j(obj->RigidBodyTreeInternal, QSol, localB);
  cartes_NLPSolverInterface_solve(obj->Solver, QSol, localB->qvSolRaw,
    &localB->d, &localB->bid, &localB->d1, &localB->d2, solutionInfo_Status_data,
    solutionInfo_Status_size, localB, localDW);
  *solutionInfo_ExitFlag = localB->d2;
  *solutionInfo_PoseErrorNorm = localB->d1;
  *solutionInfo_Iterations = localB->d;
  obj_0 = obj->RigidBodyTreeInternal;
  localB->partialTrueCount = endEffectorName->size[0] * endEffectorName->size[1];
  endEffectorName->size[0] = 1;
  endEffectorName->size[1] = obj->Solver->ExtraArgs->BodyName->size[1];
  cartes_emxEnsureCapacity_char_T(endEffectorName, localB->partialTrueCount,
    localB);
  localB->loop_ub = obj->Solver->ExtraArgs->BodyName->size[0] * obj->
    Solver->ExtraArgs->BodyName->size[1] - 1;
  for (localB->partialTrueCount = 0; localB->partialTrueCount <= localB->loop_ub;
       localB->partialTrueCount++) {
    endEffectorName->data[localB->partialTrueCount] = obj->Solver->
      ExtraArgs->BodyName->data[localB->partialTrueCount];
  }

  cartesian_trajec_emxInit_real_T(&bodyIndices, 1, localB);
  localB->partialTrueCount = bodyIndices->size[0];
  bodyIndices->size[0] = static_cast<int32_T>(obj_0->NumBodies);
  cartes_emxEnsureCapacity_real_T(bodyIndices, localB->partialTrueCount, localB);
  localB->loop_ub = static_cast<int32_T>(obj_0->NumBodies);
  for (localB->partialTrueCount = 0; localB->partialTrueCount < localB->loop_ub;
       localB->partialTrueCount++) {
    bodyIndices->data[localB->partialTrueCount] = 0.0;
  }

  cartesian_trajec_emxInit_char_T(&bname, 2, localB);
  localB->bid = -1.0;
  localB->partialTrueCount = bname->size[0] * bname->size[1];
  bname->size[0] = 1;
  bname->size[1] = obj_0->Base.NameInternal->size[1];
  cartes_emxEnsureCapacity_char_T(bname, localB->partialTrueCount, localB);
  localB->loop_ub = obj_0->Base.NameInternal->size[0] * obj_0->
    Base.NameInternal->size[1] - 1;
  for (localB->partialTrueCount = 0; localB->partialTrueCount <= localB->loop_ub;
       localB->partialTrueCount++) {
    bname->data[localB->partialTrueCount] = obj_0->Base.NameInternal->
      data[localB->partialTrueCount];
  }

  if (cartesian_trajectory_pla_strcmp(bname, endEffectorName)) {
    localB->bid = 0.0;
  } else {
    localB->numPositions = obj_0->NumBodies;
    localB->i_f = 0;
    exitg1 = false;
    while ((!exitg1) && (localB->i_f <= static_cast<int32_T>
                         (localB->numPositions) - 1)) {
      body = obj_0->Bodies[localB->i_f];
      localB->partialTrueCount = bname->size[0] * bname->size[1];
      bname->size[0] = 1;
      bname->size[1] = body->NameInternal->size[1];
      cartes_emxEnsureCapacity_char_T(bname, localB->partialTrueCount, localB);
      localB->loop_ub = body->NameInternal->size[0] * body->NameInternal->size[1]
        - 1;
      for (localB->partialTrueCount = 0; localB->partialTrueCount <=
           localB->loop_ub; localB->partialTrueCount++) {
        bname->data[localB->partialTrueCount] = body->NameInternal->data
          [localB->partialTrueCount];
      }

      if (cartesian_trajectory_pla_strcmp(bname, endEffectorName)) {
        localB->bid = static_cast<real_T>(localB->i_f) + 1.0;
        exitg1 = true;
      } else {
        localB->i_f++;
      }
    }
  }

  cartesian_trajec_emxFree_char_T(&bname);
  cartesian_trajec_emxFree_char_T(&endEffectorName);
  if (localB->bid == 0.0) {
    localB->partialTrueCount = bodyIndices->size[0];
    bodyIndices->size[0] = 1;
    cartes_emxEnsureCapacity_real_T(bodyIndices, localB->partialTrueCount,
      localB);
    bodyIndices->data[0] = 0.0;
  } else {
    body = obj_0->Bodies[static_cast<int32_T>(localB->bid) - 1];
    localB->bid = 1.0;
    while (body->ParentIndex != 0.0) {
      bodyIndices->data[static_cast<int32_T>(localB->bid) - 1] = body->Index;
      body = obj_0->Bodies[static_cast<int32_T>(body->ParentIndex) - 1];
      localB->bid++;
    }

    if (1.0 > localB->bid - 1.0) {
      localB->c_k = -1;
    } else {
      localB->c_k = static_cast<int32_T>(localB->bid - 1.0) - 1;
    }

    cartesian_trajec_emxInit_real_T(&bodyIndices_0, 1, localB);
    localB->partialTrueCount = bodyIndices_0->size[0];
    bodyIndices_0->size[0] = localB->c_k + 3;
    cartes_emxEnsureCapacity_real_T(bodyIndices_0, localB->partialTrueCount,
      localB);
    for (localB->partialTrueCount = 0; localB->partialTrueCount <= localB->c_k;
         localB->partialTrueCount++) {
      bodyIndices_0->data[localB->partialTrueCount] = bodyIndices->data
        [localB->partialTrueCount];
    }

    bodyIndices_0->data[localB->c_k + 1] = body->Index;
    bodyIndices_0->data[localB->c_k + 2] = 0.0;
    localB->partialTrueCount = bodyIndices->size[0];
    bodyIndices->size[0] = bodyIndices_0->size[0];
    cartes_emxEnsureCapacity_real_T(bodyIndices, localB->partialTrueCount,
      localB);
    localB->loop_ub = bodyIndices_0->size[0];
    for (localB->partialTrueCount = 0; localB->partialTrueCount <
         localB->loop_ub; localB->partialTrueCount++) {
      bodyIndices->data[localB->partialTrueCount] = bodyIndices_0->data
        [localB->partialTrueCount];
    }

    cartesian_trajec_emxFree_real_T(&bodyIndices_0);
  }

  obj_0 = obj->RigidBodyTreeInternal;
  localB->c_k = bodyIndices->size[0] - 1;
  localB->loop_ub = 0;
  for (localB->i_f = 0; localB->i_f <= localB->c_k; localB->i_f++) {
    if (bodyIndices->data[localB->i_f] != 0.0) {
      localB->loop_ub++;
    }
  }

  cartesian_traje_emxInit_int32_T(&h, 1);
  localB->partialTrueCount = h->size[0];
  h->size[0] = localB->loop_ub;
  carte_emxEnsureCapacity_int32_T(h, localB->partialTrueCount);
  localB->partialTrueCount = 0;
  for (localB->i_f = 0; localB->i_f <= localB->c_k; localB->i_f++) {
    if (bodyIndices->data[localB->i_f] != 0.0) {
      h->data[localB->partialTrueCount] = localB->i_f + 1;
      localB->partialTrueCount++;
    }
  }

  cartesian_trajec_emxInit_real_T(&positionMap, 2, localB);
  localB->partialTrueCount = positionMap->size[0] * positionMap->size[1];
  positionMap->size[0] = h->size[0];
  positionMap->size[1] = 2;
  cartes_emxEnsureCapacity_real_T(positionMap, localB->partialTrueCount, localB);
  localB->loop_ub = h->size[0];
  for (localB->partialTrueCount = 0; localB->partialTrueCount < localB->loop_ub;
       localB->partialTrueCount++) {
    positionMap->data[localB->partialTrueCount] = obj_0->PositionDoFMap[
      static_cast<int32_T>(bodyIndices->data[h->data[localB->partialTrueCount] -
      1]) - 1];
  }

  localB->loop_ub = h->size[0];
  for (localB->partialTrueCount = 0; localB->partialTrueCount < localB->loop_ub;
       localB->partialTrueCount++) {
    positionMap->data[localB->partialTrueCount + positionMap->size[0]] =
      obj_0->PositionDoFMap[static_cast<int32_T>(bodyIndices->data[h->
      data[localB->partialTrueCount] - 1]) + 7];
  }

  cartesian_traje_emxFree_int32_T(&h);
  cartesian_trajec_emxFree_real_T(&bodyIndices);
  cartesian_trajec_emxInit_real_T(&positionIndices, 2, localB);
  localB->partialTrueCount = positionIndices->size[0] * positionIndices->size[1];
  positionIndices->size[0] = 1;
  positionIndices->size[1] = static_cast<int32_T>(obj_0->PositionNumber);
  cartes_emxEnsureCapacity_real_T(positionIndices, localB->partialTrueCount,
    localB);
  localB->loop_ub = static_cast<int32_T>(obj_0->PositionNumber) - 1;
  for (localB->partialTrueCount = 0; localB->partialTrueCount <= localB->loop_ub;
       localB->partialTrueCount++) {
    positionIndices->data[localB->partialTrueCount] = 0.0;
  }

  localB->bid = 0.0;
  localB->c_k = positionMap->size[0] - 1;
  cartesian_trajec_emxInit_real_T(&e, 2, localB);
  cartesian_trajec_emxInit_real_T(&y, 2, localB);
  for (localB->i_f = 0; localB->i_f <= localB->c_k; localB->i_f++) {
    localB->numPositions = (positionMap->data[localB->i_f + positionMap->size[0]]
      - positionMap->data[localB->i_f]) + 1.0;
    if (localB->numPositions > 0.0) {
      if (localB->numPositions < 1.0) {
        y->size[0] = 1;
        y->size[1] = 0;
      } else if (rtIsInf(localB->numPositions) && (1.0 == localB->numPositions))
      {
        localB->partialTrueCount = y->size[0] * y->size[1];
        y->size[0] = 1;
        y->size[1] = 1;
        cartes_emxEnsureCapacity_real_T(y, localB->partialTrueCount, localB);
        y->data[0] = (rtNaN);
      } else {
        localB->partialTrueCount = y->size[0] * y->size[1];
        y->size[0] = 1;
        localB->loop_ub = static_cast<int32_T>(floor(localB->numPositions - 1.0));
        y->size[1] = localB->loop_ub + 1;
        cartes_emxEnsureCapacity_real_T(y, localB->partialTrueCount, localB);
        for (localB->partialTrueCount = 0; localB->partialTrueCount <=
             localB->loop_ub; localB->partialTrueCount++) {
          y->data[localB->partialTrueCount] = static_cast<real_T>
            (localB->partialTrueCount) + 1.0;
        }
      }

      if (rtIsNaN(positionMap->data[localB->i_f]) || rtIsNaN(positionMap->
           data[localB->i_f + positionMap->size[0]])) {
        localB->partialTrueCount = e->size[0] * e->size[1];
        e->size[0] = 1;
        e->size[1] = 1;
        cartes_emxEnsureCapacity_real_T(e, localB->partialTrueCount, localB);
        e->data[0] = (rtNaN);
      } else if (positionMap->data[localB->i_f + positionMap->size[0]] <
                 positionMap->data[localB->i_f]) {
        e->size[0] = 1;
        e->size[1] = 0;
      } else if ((rtIsInf(positionMap->data[localB->i_f]) || rtIsInf
                  (positionMap->data[localB->i_f + positionMap->size[0]])) &&
                 (positionMap->data[localB->i_f + positionMap->size[0]] ==
                  positionMap->data[localB->i_f])) {
        localB->partialTrueCount = e->size[0] * e->size[1];
        e->size[0] = 1;
        e->size[1] = 1;
        cartes_emxEnsureCapacity_real_T(e, localB->partialTrueCount, localB);
        e->data[0] = (rtNaN);
      } else if (floor(positionMap->data[localB->i_f]) == positionMap->
                 data[localB->i_f]) {
        localB->partialTrueCount = e->size[0] * e->size[1];
        e->size[0] = 1;
        e->size[1] = static_cast<int32_T>(floor(positionMap->data[localB->i_f +
          positionMap->size[0]] - positionMap->data[localB->i_f])) + 1;
        cartes_emxEnsureCapacity_real_T(e, localB->partialTrueCount, localB);
        localB->loop_ub = static_cast<int32_T>(floor(positionMap->data
          [localB->i_f + positionMap->size[0]] - positionMap->data[localB->i_f]));
        for (localB->partialTrueCount = 0; localB->partialTrueCount <=
             localB->loop_ub; localB->partialTrueCount++) {
          e->data[localB->partialTrueCount] = positionMap->data[localB->i_f] +
            static_cast<real_T>(localB->partialTrueCount);
        }
      } else {
        localB->ndbl = floor((positionMap->data[localB->i_f + positionMap->size
                              [0]] - positionMap->data[localB->i_f]) + 0.5);
        localB->apnd = positionMap->data[localB->i_f] + localB->ndbl;
        localB->cdiff = localB->apnd - positionMap->data[localB->i_f +
          positionMap->size[0]];
        localB->u0_m = fabs(positionMap->data[localB->i_f]);
        localB->u1_m = fabs(positionMap->data[localB->i_f + positionMap->size[0]]);
        if ((localB->u0_m > localB->u1_m) || rtIsNaN(localB->u1_m)) {
          localB->u1_m = localB->u0_m;
        }

        if (fabs(localB->cdiff) < 4.4408920985006262E-16 * localB->u1_m) {
          localB->ndbl++;
          localB->apnd = positionMap->data[localB->i_f + positionMap->size[0]];
        } else if (localB->cdiff > 0.0) {
          localB->apnd = (localB->ndbl - 1.0) + positionMap->data[localB->i_f];
        } else {
          localB->ndbl++;
        }

        if (localB->ndbl >= 0.0) {
          localB->partialTrueCount = static_cast<int32_T>(localB->ndbl);
        } else {
          localB->partialTrueCount = 0;
        }

        localB->loop_ub = localB->partialTrueCount - 1;
        localB->partialTrueCount = e->size[0] * e->size[1];
        e->size[0] = 1;
        e->size[1] = localB->loop_ub + 1;
        cartes_emxEnsureCapacity_real_T(e, localB->partialTrueCount, localB);
        if (localB->loop_ub + 1 > 0) {
          e->data[0] = positionMap->data[localB->i_f];
          if (localB->loop_ub + 1 > 1) {
            e->data[localB->loop_ub] = localB->apnd;
            localB->nm1d2 = ((localB->loop_ub < 0) + localB->loop_ub) >> 1;
            localB->c_j = localB->nm1d2 - 2;
            for (localB->partialTrueCount = 0; localB->partialTrueCount <=
                 localB->c_j; localB->partialTrueCount++) {
              localB->k = localB->partialTrueCount + 1;
              e->data[localB->k] = positionMap->data[localB->i_f] + static_cast<
                real_T>(localB->k);
              e->data[localB->loop_ub - localB->k] = localB->apnd - static_cast<
                real_T>(localB->k);
            }

            if (localB->nm1d2 << 1 == localB->loop_ub) {
              e->data[localB->nm1d2] = (positionMap->data[localB->i_f] +
                localB->apnd) / 2.0;
            } else {
              e->data[localB->nm1d2] = positionMap->data[localB->i_f] +
                static_cast<real_T>(localB->nm1d2);
              e->data[localB->nm1d2 + 1] = localB->apnd - static_cast<real_T>
                (localB->nm1d2);
            }
          }
        }
      }

      localB->partialTrueCount = e->size[0] * e->size[1];
      localB->loop_ub = localB->partialTrueCount - 1;
      for (localB->partialTrueCount = 0; localB->partialTrueCount <=
           localB->loop_ub; localB->partialTrueCount++) {
        positionIndices->data[static_cast<int32_T>(localB->bid + y->data
          [localB->partialTrueCount]) - 1] = e->data[localB->partialTrueCount];
      }

      localB->bid += localB->numPositions;
    }
  }

  cartesian_trajec_emxFree_real_T(&y);
  cartesian_trajec_emxFree_real_T(&e);
  cartesian_trajec_emxFree_real_T(&positionMap);
  if (1.0 > localB->bid) {
    positionIndices->size[1] = 0;
  } else {
    localB->partialTrueCount = positionIndices->size[0] * positionIndices->size
      [1];
    positionIndices->size[1] = static_cast<int32_T>(localB->bid);
    cartes_emxEnsureCapacity_real_T(positionIndices, localB->partialTrueCount,
      localB);
  }

  localB->loop_ub = positionIndices->size[0] * positionIndices->size[1];
  for (localB->partialTrueCount = 0; localB->partialTrueCount < localB->loop_ub;
       localB->partialTrueCount++) {
    QSol[static_cast<int32_T>(positionIndices->data[localB->partialTrueCount]) -
      1] = localB->qvSolRaw[static_cast<int32_T>(positionIndices->data
      [localB->partialTrueCount]) - 1];
  }

  cartesian_trajec_emxFree_real_T(&positionIndices);
}

real_T rt_roundd_snf(real_T u)
{
  real_T y;
  if (fabs(u) < 4.503599627370496E+15) {
    if (u >= 0.5) {
      y = floor(u + 0.5);
    } else if (u > -0.5) {
      y = u * 0.0;
    } else {
      y = ceil(u - 0.5);
    }
  } else {
    y = u;
  }

  return y;
}

static void matlabCodegenHandle_matlabCod_o(robotics_slmanip_internal_blo_T *obj)
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

static void cartesian_tra_SystemCore_delete(b_inverseKinematics_cartesian_T *obj)
{
  cartesian_tr_SystemCore_release(obj);
}

static void matlabCodegenHandle_matlabCodeg(b_inverseKinematics_cartesian_T *obj)
{
  if (!obj->matlabCodegenIsDeleted) {
    obj->matlabCodegenIsDeleted = true;
    cartesian_tra_SystemCore_delete(obj);
  }
}

static void emxFreeStruct_c_rigidBodyJoint(c_rigidBodyJoint_cartesian_tr_T
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

static void emxFreeStruct_robotics_slmanip_(robotics_slmanip_internal_blo_T
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

//
// System initialize for atomic system:
//    synthesized block
//    synthesized block
//
void cartesian_tra_MATLABSystem_Init(B_MATLABSystem_cartesian_traj_T *localB,
  DW_MATLABSystem_cartesian_tra_T *localDW)
{
  robotics_slmanip_internal_blo_T *obj;
  b_inverseKinematics_cartesian_T *obj_0;
  h_robotics_core_internal_Damp_T *obj_1;
  emxInitStruct_robotics_slmanip_(&localDW->obj, localB);
  emxInitStruct_c_rigidBodyJoint(&localDW->gobj_1, localB);
  emxInitStruct_c_rigidBodyJoint(&localDW->gobj_50, localB);
  emxInitStruct_c_rigidBodyJoint(&localDW->gobj_49, localB);
  emxInitStruct_c_rigidBodyJoint(&localDW->gobj_48, localB);
  emxInitStruct_c_rigidBodyJoint(&localDW->gobj_47, localB);
  emxInitStruct_c_rigidBodyJoint(&localDW->gobj_46, localB);
  emxInitStruct_c_rigidBodyJoint(&localDW->gobj_45, localB);
  emxInitStruct_c_rigidBodyJoint(&localDW->gobj_44, localB);
  emxInitStruct_c_rigidBodyJoint(&localDW->gobj_43, localB);
  emxInitStruct_c_rigidBodyJoint(&localDW->gobj_42, localB);
  emxInitStruct_c_rigidBodyJoint(&localDW->gobj_41, localB);
  emxInitStruct_c_rigidBodyJoint(&localDW->gobj_40, localB);
  emxInitStruct_c_rigidBodyJoint(&localDW->gobj_39, localB);
  emxInitStruct_c_rigidBodyJoint(&localDW->gobj_38, localB);
  emxInitStruct_c_rigidBodyJoint(&localDW->gobj_37, localB);
  emxInitStruct_c_rigidBodyJoint(&localDW->gobj_36, localB);
  emxInitStruct_c_rigidBodyJoint(&localDW->gobj_35, localB);
  emxInitStruct_c_rigidBodyJoint(&localDW->gobj_34, localB);
  emxInitStruct_c_rigidBodyJoint(&localDW->gobj_33, localB);
  emxInitStruct_c_rigidBodyJoint(&localDW->gobj_32, localB);
  emxInitStruct_c_rigidBodyJoint(&localDW->gobj_31, localB);
  emxInitStruct_c_rigidBodyJoint(&localDW->gobj_30, localB);
  emxInitStruct_c_rigidBodyJoint(&localDW->gobj_29, localB);
  emxInitStruct_c_rigidBodyJoint(&localDW->gobj_28, localB);
  emxInitStruct_c_rigidBodyJoint(&localDW->gobj_27, localB);
  emxInitStruct_c_rigidBodyJoint(&localDW->gobj_26, localB);
  emxInitStruct_c_rigidBodyJoint(&localDW->gobj_25, localB);
  emxInitStruct_c_rigidBodyJoint(&localDW->gobj_24, localB);
  emxInitStruct_c_rigidBodyJoint(&localDW->gobj_23, localB);
  emxInitStruct_c_rigidBodyJoint(&localDW->gobj_22, localB);
  emxInitStruct_c_rigidBodyJoint(&localDW->gobj_21, localB);
  emxInitStruct_c_rigidBodyJoint(&localDW->gobj_20, localB);
  emxInitStruct_c_rigidBodyJoint(&localDW->gobj_19, localB);
  emxInitStruct_c_rigidBodyJoint(&localDW->gobj_18, localB);
  emxInitStruct_c_rigidBodyJoint(&localDW->gobj_17, localB);
  emxInitStruct_c_rigidBodyJoint(&localDW->gobj_16, localB);
  emxInitStruct_c_rigidBodyJoint(&localDW->gobj_15, localB);
  emxInitStruct_c_rigidBodyJoint(&localDW->gobj_14, localB);
  emxInitStruct_c_rigidBodyJoint(&localDW->gobj_13, localB);
  emxInitStruct_c_rigidBodyJoint(&localDW->gobj_12, localB);
  emxInitStruct_c_rigidBodyJoint(&localDW->gobj_11, localB);
  emxInitStruct_c_rigidBodyJoint(&localDW->gobj_10, localB);
  emxInitStruct_c_rigidBodyJoint(&localDW->gobj_9, localB);
  emxInitStruct_c_rigidBodyJoint(&localDW->gobj_8, localB);
  emxInitStruct_c_rigidBodyJoint(&localDW->gobj_7, localB);
  emxInitStruct_c_rigidBodyJoint(&localDW->gobj_6, localB);
  emxInitStruct_c_rigidBodyJoint(&localDW->gobj_5, localB);
  emxInitStruct_c_rigidBodyJoint(&localDW->gobj_4, localB);
  emxInitStruct_c_rigidBodyJoint(&localDW->gobj_3, localB);
  emxInitStruct_c_rigidBodyJoint(&localDW->gobj_2, localB);
  emxInitStruct_w_robotics_manip_(&localDW->gobj_51, localB);
  emxInitStruct_w_robotics_manip_(&localDW->gobj_82, localB);
  emxInitStruct_w_robotics_manip_(&localDW->gobj_81, localB);
  emxInitStruct_w_robotics_manip_(&localDW->gobj_80, localB);
  emxInitStruct_w_robotics_manip_(&localDW->gobj_79, localB);
  emxInitStruct_w_robotics_manip_(&localDW->gobj_78, localB);
  emxInitStruct_w_robotics_manip_(&localDW->gobj_77, localB);
  emxInitStruct_w_robotics_manip_(&localDW->gobj_76, localB);
  emxInitStruct_w_robotics_manip_(&localDW->gobj_75, localB);
  emxInitStruct_w_robotics_manip_(&localDW->gobj_74, localB);
  emxInitStruct_w_robotics_manip_(&localDW->gobj_73, localB);
  emxInitStruct_w_robotics_manip_(&localDW->gobj_72, localB);
  emxInitStruct_w_robotics_manip_(&localDW->gobj_71, localB);
  emxInitStruct_w_robotics_manip_(&localDW->gobj_70, localB);
  emxInitStruct_w_robotics_manip_(&localDW->gobj_69, localB);
  emxInitStruct_w_robotics_manip_(&localDW->gobj_68, localB);
  emxInitStruct_w_robotics_manip_(&localDW->gobj_67, localB);
  emxInitStruct_w_robotics_manip_(&localDW->gobj_66, localB);
  emxInitStruct_w_robotics_manip_(&localDW->gobj_65, localB);
  emxInitStruct_w_robotics_manip_(&localDW->gobj_64, localB);
  emxInitStruct_w_robotics_manip_(&localDW->gobj_63, localB);
  emxInitStruct_w_robotics_manip_(&localDW->gobj_62, localB);
  emxInitStruct_w_robotics_manip_(&localDW->gobj_61, localB);
  emxInitStruct_w_robotics_manip_(&localDW->gobj_60, localB);
  emxInitStruct_w_robotics_manip_(&localDW->gobj_59, localB);
  emxInitStruct_w_robotics_manip_(&localDW->gobj_58, localB);
  emxInitStruct_w_robotics_manip_(&localDW->gobj_57, localB);
  emxInitStruct_w_robotics_manip_(&localDW->gobj_56, localB);
  emxInitStruct_w_robotics_manip_(&localDW->gobj_55, localB);
  emxInitStruct_w_robotics_manip_(&localDW->gobj_54, localB);
  emxInitStruct_w_robotics_manip_(&localDW->gobj_53, localB);
  emxInitStruct_w_robotics_manip_(&localDW->gobj_52, localB);
  emxInitStruct_x_robotics_manip_(&localDW->gobj_83, localB);
  emxInitStruct_x_robotics_manip_(&localDW->gobj_84, localB);
  emxInitStruct_f_robotics_manip_(&localDW->gobj_85, localB);
  emxInitStruct_h_robotics_core_i(&localDW->gobj_86, localB);
  emxInitStruct_h_robotics_core_i(&localDW->gobj_87, localB);
  emxInitStruct_v_robotics_manip_(&localDW->gobj_88, localB);
  emxInitStruct_v_robotics_manip_(&localDW->gobj_103, localB);
  emxInitStruct_v_robotics_manip_(&localDW->gobj_102, localB);
  emxInitStruct_v_robotics_manip_(&localDW->gobj_101, localB);
  emxInitStruct_v_robotics_manip_(&localDW->gobj_100, localB);
  emxInitStruct_v_robotics_manip_(&localDW->gobj_99, localB);
  emxInitStruct_v_robotics_manip_(&localDW->gobj_98, localB);
  emxInitStruct_v_robotics_manip_(&localDW->gobj_97, localB);
  emxInitStruct_v_robotics_manip_(&localDW->gobj_96, localB);
  emxInitStruct_v_robotics_manip_(&localDW->gobj_95, localB);
  emxInitStruct_v_robotics_manip_(&localDW->gobj_94, localB);
  emxInitStruct_v_robotics_manip_(&localDW->gobj_93, localB);
  emxInitStruct_v_robotics_manip_(&localDW->gobj_92, localB);
  emxInitStruct_v_robotics_manip_(&localDW->gobj_91, localB);
  emxInitStruct_v_robotics_manip_(&localDW->gobj_90, localB);
  emxInitStruct_v_robotics_manip_(&localDW->gobj_89, localB);

  // Start for MATLABSystem: '<S2>/MATLAB System'
  localDW->obj.IKInternal.matlabCodegenIsDeleted = true;
  localDW->obj.matlabCodegenIsDeleted = true;
  localDW->method_m = 7U;
  localDW->method_not_empty = true;
  localDW->state = 1144108930U;
  localDW->state_not_empty = true;
  localDW->state_a[0] = 362436069U;
  localDW->state_a[1] = 521288629U;
  localDW->state_not_empty_k = true;
  cartesian_tr_eml_rand_mt19937ar(localDW->state_e);
  localDW->method_not_empty_k = true;
  localDW->state_not_empty_j = true;
  cartesian_tra_eml_rand_shr3cong(localDW->state_ar);
  localDW->state_not_empty_i = true;
  localDW->obj.isInitialized = 0;
  localDW->obj.matlabCodegenIsDeleted = false;
  localDW->objisempty = true;
  obj = &localDW->obj;
  localDW->obj.isInitialized = 1;
  car_RigidBodyTree_RigidBodyTree(&localDW->obj.TreeInternal, &localDW->gobj_90,
    &localDW->gobj_91, &localDW->gobj_92, &localDW->gobj_93, &localDW->gobj_94,
    &localDW->gobj_95, &localDW->gobj_96, &localDW->gobj_89, localB);
  obj_0 = &localDW->obj.IKInternal;
  localDW->obj.IKInternal.isInitialized = 0;
  inverseKinematics_set_RigidBody(&localDW->obj.IKInternal, &obj->TreeInternal,
    &localDW->gobj_69, &localDW->gobj_70, &localDW->gobj_71, &localDW->gobj_72,
    &localDW->gobj_73, &localDW->gobj_74, &localDW->gobj_75, &localDW->gobj_76,
    &localDW->gobj_77, &localDW->gobj_78, &localDW->gobj_79, &localDW->gobj_80,
    &localDW->gobj_81, &localDW->gobj_82, &localDW->gobj_51, &localDW->gobj_28,
    &localDW->gobj_29, &localDW->gobj_30, &localDW->gobj_31, &localDW->gobj_32,
    &localDW->gobj_33, &localDW->gobj_34, &localDW->gobj_35, &localDW->gobj_36,
    &localDW->gobj_37, &localDW->gobj_38, &localDW->gobj_39, &localDW->gobj_40,
    &localDW->gobj_41, &localDW->gobj_42, &localDW->gobj_43, &localDW->gobj_44,
    &localDW->gobj_45, &localDW->gobj_46, &localDW->gobj_47, &localDW->gobj_48,
    &localDW->gobj_49, &localDW->gobj_50, &localDW->gobj_1, &localDW->gobj_27,
    &localDW->gobj_68, &localDW->gobj_83, localB, localDW);
  obj_0->Solver = DampedBFGSwGradientProjection_D(&localDW->gobj_87);
  obj_1 = obj_0->Solver;
  obj_1->MaxNumIteration = 1500.0;
  obj_1->MaxTime = 10.0;
  obj_1->GradientTolerance = 1.0E-7;
  obj_1->SolutionTolerance = 1.0E-6;
  obj_1->ConstraintsOn = true;
  obj_1->RandomRestart = false;
  obj_1->StepTolerance = 1.0E-14;
  obj_0->matlabCodegenIsDeleted = false;
}

//
// Output and update for atomic system:
//    synthesized block
//    synthesized block
//
void cartesian_trajecto_MATLABSystem(const real_T rtu_0[16], const real_T rtu_1
  [6], const real_T rtu_2[6], B_MATLABSystem_cartesian_traj_T *localB,
  DW_MATLABSystem_cartesian_tra_T *localDW)
{
  robotics_slmanip_internal_blo_T *obj;
  static const char_T tmp[7] = { 's', 'u', 'c', 'c', 'e', 's', 's' };

  int32_T exitg1;

  // MATLABSystem: '<S2>/MATLAB System'
  for (localB->i = 0; localB->i < 16; localB->i++) {
    localB->u0[localB->i] = rtu_0[localB->i];
  }

  for (localB->i = 0; localB->i < 6; localB->i++) {
    localB->u1[localB->i] = rtu_1[localB->i];
  }

  for (localB->i = 0; localB->i < 6; localB->i++) {
    localB->u2[localB->i] = rtu_2[localB->i];
  }

  obj = &localDW->obj;
  if (localDW->obj.IKInternal.isInitialized != 1) {
    localDW->obj.IKInternal.isSetupComplete = false;
    localDW->obj.IKInternal.isInitialized = 1;
    car_inverseKinematics_setupImpl(&localDW->obj.IKInternal, &localDW->gobj_85,
      localB);
    obj->IKInternal.isSetupComplete = true;
  }

  cart_inverseKinematics_stepImpl(&obj->IKInternal, localB->u0, localB->u1,
    localB->u2, localB->MATLABSystem_o1, &localB->MATLABSystem_o2.Iterations,
    &localB->MATLABSystem_o2.PoseErrorNorm, &localB->b_varargout_2_ExitFlag,
    localB->b_varargout_2_Status_data, localB->b_varargout_2_Status_size, localB,
    localDW);
  for (localB->i = 0; localB->i < 7; localB->i++) {
    localB->b_j[localB->i] = tmp[localB->i];
  }

  localB->b_bool = false;
  if (localB->b_varargout_2_Status_size[1] == 7) {
    localB->i = 1;
    do {
      exitg1 = 0;
      if (localB->i - 1 < 7) {
        localB->kstr = localB->i - 1;
        if (localB->b_varargout_2_Status_data[localB->kstr] != localB->
            b_j[localB->kstr]) {
          exitg1 = 1;
        } else {
          localB->i++;
        }
      } else {
        localB->b_bool = true;
        exitg1 = 1;
      }
    } while (exitg1 == 0);
  }

  localB->b_varargout_2_ExitFlag = rt_roundd_snf(localB->b_varargout_2_ExitFlag);
  if (localB->b_varargout_2_ExitFlag < 65536.0) {
    if (localB->b_varargout_2_ExitFlag >= 0.0) {
      localB->MATLABSystem_o2.ExitFlag = static_cast<uint16_T>
        (localB->b_varargout_2_ExitFlag);
    } else {
      localB->MATLABSystem_o2.ExitFlag = 0U;
    }
  } else {
    localB->MATLABSystem_o2.ExitFlag = MAX_uint16_T;
  }

  if (localB->b_bool) {
    localB->MATLABSystem_o2.Status = 1U;
  } else {
    localB->MATLABSystem_o2.Status = 2U;
  }

  // End of MATLABSystem: '<S2>/MATLAB System'
}

//
// Termination for atomic system:
//    synthesized block
//    synthesized block
//
void cartesian_tra_MATLABSystem_Term(DW_MATLABSystem_cartesian_tra_T *localDW)
{
  // Terminate for MATLABSystem: '<S2>/MATLAB System'
  matlabCodegenHandle_matlabCod_o(&localDW->obj);
  matlabCodegenHandle_matlabCodeg(&localDW->obj.IKInternal);
  emxFreeStruct_robotics_slmanip_(&localDW->obj);
  emxFreeStruct_c_rigidBodyJoint(&localDW->gobj_1);
  emxFreeStruct_c_rigidBodyJoint(&localDW->gobj_50);
  emxFreeStruct_c_rigidBodyJoint(&localDW->gobj_49);
  emxFreeStruct_c_rigidBodyJoint(&localDW->gobj_48);
  emxFreeStruct_c_rigidBodyJoint(&localDW->gobj_47);
  emxFreeStruct_c_rigidBodyJoint(&localDW->gobj_46);
  emxFreeStruct_c_rigidBodyJoint(&localDW->gobj_45);
  emxFreeStruct_c_rigidBodyJoint(&localDW->gobj_44);
  emxFreeStruct_c_rigidBodyJoint(&localDW->gobj_43);
  emxFreeStruct_c_rigidBodyJoint(&localDW->gobj_42);
  emxFreeStruct_c_rigidBodyJoint(&localDW->gobj_41);
  emxFreeStruct_c_rigidBodyJoint(&localDW->gobj_40);
  emxFreeStruct_c_rigidBodyJoint(&localDW->gobj_39);
  emxFreeStruct_c_rigidBodyJoint(&localDW->gobj_38);
  emxFreeStruct_c_rigidBodyJoint(&localDW->gobj_37);
  emxFreeStruct_c_rigidBodyJoint(&localDW->gobj_36);
  emxFreeStruct_c_rigidBodyJoint(&localDW->gobj_35);
  emxFreeStruct_c_rigidBodyJoint(&localDW->gobj_34);
  emxFreeStruct_c_rigidBodyJoint(&localDW->gobj_33);
  emxFreeStruct_c_rigidBodyJoint(&localDW->gobj_32);
  emxFreeStruct_c_rigidBodyJoint(&localDW->gobj_31);
  emxFreeStruct_c_rigidBodyJoint(&localDW->gobj_30);
  emxFreeStruct_c_rigidBodyJoint(&localDW->gobj_29);
  emxFreeStruct_c_rigidBodyJoint(&localDW->gobj_28);
  emxFreeStruct_c_rigidBodyJoint(&localDW->gobj_27);
  emxFreeStruct_c_rigidBodyJoint(&localDW->gobj_26);
  emxFreeStruct_c_rigidBodyJoint(&localDW->gobj_25);
  emxFreeStruct_c_rigidBodyJoint(&localDW->gobj_24);
  emxFreeStruct_c_rigidBodyJoint(&localDW->gobj_23);
  emxFreeStruct_c_rigidBodyJoint(&localDW->gobj_22);
  emxFreeStruct_c_rigidBodyJoint(&localDW->gobj_21);
  emxFreeStruct_c_rigidBodyJoint(&localDW->gobj_20);
  emxFreeStruct_c_rigidBodyJoint(&localDW->gobj_19);
  emxFreeStruct_c_rigidBodyJoint(&localDW->gobj_18);
  emxFreeStruct_c_rigidBodyJoint(&localDW->gobj_17);
  emxFreeStruct_c_rigidBodyJoint(&localDW->gobj_16);
  emxFreeStruct_c_rigidBodyJoint(&localDW->gobj_15);
  emxFreeStruct_c_rigidBodyJoint(&localDW->gobj_14);
  emxFreeStruct_c_rigidBodyJoint(&localDW->gobj_13);
  emxFreeStruct_c_rigidBodyJoint(&localDW->gobj_12);
  emxFreeStruct_c_rigidBodyJoint(&localDW->gobj_11);
  emxFreeStruct_c_rigidBodyJoint(&localDW->gobj_10);
  emxFreeStruct_c_rigidBodyJoint(&localDW->gobj_9);
  emxFreeStruct_c_rigidBodyJoint(&localDW->gobj_8);
  emxFreeStruct_c_rigidBodyJoint(&localDW->gobj_7);
  emxFreeStruct_c_rigidBodyJoint(&localDW->gobj_6);
  emxFreeStruct_c_rigidBodyJoint(&localDW->gobj_5);
  emxFreeStruct_c_rigidBodyJoint(&localDW->gobj_4);
  emxFreeStruct_c_rigidBodyJoint(&localDW->gobj_3);
  emxFreeStruct_c_rigidBodyJoint(&localDW->gobj_2);
  emxFreeStruct_w_robotics_manip_(&localDW->gobj_51);
  emxFreeStruct_w_robotics_manip_(&localDW->gobj_82);
  emxFreeStruct_w_robotics_manip_(&localDW->gobj_81);
  emxFreeStruct_w_robotics_manip_(&localDW->gobj_80);
  emxFreeStruct_w_robotics_manip_(&localDW->gobj_79);
  emxFreeStruct_w_robotics_manip_(&localDW->gobj_78);
  emxFreeStruct_w_robotics_manip_(&localDW->gobj_77);
  emxFreeStruct_w_robotics_manip_(&localDW->gobj_76);
  emxFreeStruct_w_robotics_manip_(&localDW->gobj_75);
  emxFreeStruct_w_robotics_manip_(&localDW->gobj_74);
  emxFreeStruct_w_robotics_manip_(&localDW->gobj_73);
  emxFreeStruct_w_robotics_manip_(&localDW->gobj_72);
  emxFreeStruct_w_robotics_manip_(&localDW->gobj_71);
  emxFreeStruct_w_robotics_manip_(&localDW->gobj_70);
  emxFreeStruct_w_robotics_manip_(&localDW->gobj_69);
  emxFreeStruct_w_robotics_manip_(&localDW->gobj_68);
  emxFreeStruct_w_robotics_manip_(&localDW->gobj_67);
  emxFreeStruct_w_robotics_manip_(&localDW->gobj_66);
  emxFreeStruct_w_robotics_manip_(&localDW->gobj_65);
  emxFreeStruct_w_robotics_manip_(&localDW->gobj_64);
  emxFreeStruct_w_robotics_manip_(&localDW->gobj_63);
  emxFreeStruct_w_robotics_manip_(&localDW->gobj_62);
  emxFreeStruct_w_robotics_manip_(&localDW->gobj_61);
  emxFreeStruct_w_robotics_manip_(&localDW->gobj_60);
  emxFreeStruct_w_robotics_manip_(&localDW->gobj_59);
  emxFreeStruct_w_robotics_manip_(&localDW->gobj_58);
  emxFreeStruct_w_robotics_manip_(&localDW->gobj_57);
  emxFreeStruct_w_robotics_manip_(&localDW->gobj_56);
  emxFreeStruct_w_robotics_manip_(&localDW->gobj_55);
  emxFreeStruct_w_robotics_manip_(&localDW->gobj_54);
  emxFreeStruct_w_robotics_manip_(&localDW->gobj_53);
  emxFreeStruct_w_robotics_manip_(&localDW->gobj_52);
  emxFreeStruct_x_robotics_manip_(&localDW->gobj_83);
  emxFreeStruct_x_robotics_manip_(&localDW->gobj_84);
  emxFreeStruct_f_robotics_manip_(&localDW->gobj_85);
  emxFreeStruct_h_robotics_core_i(&localDW->gobj_86);
  emxFreeStruct_h_robotics_core_i(&localDW->gobj_87);
  emxFreeStruct_v_robotics_manip_(&localDW->gobj_88);
  emxFreeStruct_v_robotics_manip_(&localDW->gobj_103);
  emxFreeStruct_v_robotics_manip_(&localDW->gobj_102);
  emxFreeStruct_v_robotics_manip_(&localDW->gobj_101);
  emxFreeStruct_v_robotics_manip_(&localDW->gobj_100);
  emxFreeStruct_v_robotics_manip_(&localDW->gobj_99);
  emxFreeStruct_v_robotics_manip_(&localDW->gobj_98);
  emxFreeStruct_v_robotics_manip_(&localDW->gobj_97);
  emxFreeStruct_v_robotics_manip_(&localDW->gobj_96);
  emxFreeStruct_v_robotics_manip_(&localDW->gobj_95);
  emxFreeStruct_v_robotics_manip_(&localDW->gobj_94);
  emxFreeStruct_v_robotics_manip_(&localDW->gobj_93);
  emxFreeStruct_v_robotics_manip_(&localDW->gobj_92);
  emxFreeStruct_v_robotics_manip_(&localDW->gobj_91);
  emxFreeStruct_v_robotics_manip_(&localDW->gobj_90);
  emxFreeStruct_v_robotics_manip_(&localDW->gobj_89);
}

//
// System initialize for atomic system:
//    synthesized block
//    synthesized block
//
void CoordinateTransformationCo_Init(DW_CoordinateTransformationCo_T *localDW)
{
  // Start for MATLABSystem: '<S13>/Coordinate Transformation Conversion'
  localDW->objisempty = true;
  localDW->obj.isInitialized = 1;
}

//
// Output and update for atomic system:
//    synthesized block
//    synthesized block
//
void CoordinateTransformationConvers(const real_T rtu_0[4], const real_T rtu_1[3],
  B_CoordinateTransformationCon_T *localB)
{
  int32_T b_k;
  real_T u0_idx_2;
  real_T u0_idx_3;
  int32_T subsa_idx_1;
  real_T tempR_tmp;
  real_T tempR_tmp_0;
  real_T tempR_tmp_1;
  real_T tempR_tmp_2;
  real_T tempR_tmp_3;

  // MATLABSystem: '<S13>/Coordinate Transformation Conversion'
  localB->u0_idx_0 = rtu_0[0];
  localB->u0_idx_1 = rtu_0[1];
  u0_idx_2 = rtu_0[2];
  u0_idx_3 = rtu_0[3];
  localB->u1[0] = rtu_1[0];
  localB->u1[1] = rtu_1[1];
  localB->u1[2] = rtu_1[2];
  localB->b = 1.0 / sqrt(((localB->u0_idx_0 * localB->u0_idx_0 +
    localB->u0_idx_1 * localB->u0_idx_1) + u0_idx_2 * u0_idx_2) + u0_idx_3 *
    u0_idx_3);
  localB->u0_idx_0 *= localB->b;
  localB->u0_idx_1 *= localB->b;
  u0_idx_2 *= localB->b;
  u0_idx_3 *= localB->b;
  localB->b = u0_idx_3 * u0_idx_3;
  tempR_tmp_3 = u0_idx_2 * u0_idx_2;
  localB->tempR[0] = 1.0 - (tempR_tmp_3 + localB->b) * 2.0;
  tempR_tmp = localB->u0_idx_1 * u0_idx_2;
  tempR_tmp_0 = localB->u0_idx_0 * u0_idx_3;
  localB->tempR[1] = (tempR_tmp - tempR_tmp_0) * 2.0;
  tempR_tmp_1 = localB->u0_idx_1 * u0_idx_3;
  tempR_tmp_2 = localB->u0_idx_0 * u0_idx_2;
  localB->tempR[2] = (tempR_tmp_1 + tempR_tmp_2) * 2.0;
  localB->tempR[3] = (tempR_tmp + tempR_tmp_0) * 2.0;
  tempR_tmp = localB->u0_idx_1 * localB->u0_idx_1;
  localB->tempR[4] = 1.0 - (tempR_tmp + localB->b) * 2.0;
  localB->b = u0_idx_2 * u0_idx_3;
  tempR_tmp_0 = localB->u0_idx_0 * localB->u0_idx_1;
  localB->tempR[5] = (localB->b - tempR_tmp_0) * 2.0;
  localB->tempR[6] = (tempR_tmp_1 - tempR_tmp_2) * 2.0;
  localB->tempR[7] = (localB->b + tempR_tmp_0) * 2.0;
  localB->tempR[8] = 1.0 - (tempR_tmp + tempR_tmp_3) * 2.0;
  for (b_k = 0; b_k < 3; b_k++) {
    subsa_idx_1 = b_k + 1;
    localB->R[subsa_idx_1 - 1] = localB->tempR[(subsa_idx_1 - 1) * 3];
    subsa_idx_1 = b_k + 1;
    localB->R[subsa_idx_1 + 2] = localB->tempR[(subsa_idx_1 - 1) * 3 + 1];
    subsa_idx_1 = b_k + 1;
    localB->R[subsa_idx_1 + 5] = localB->tempR[(subsa_idx_1 - 1) * 3 + 2];
  }

  memset(&localB->CoordinateTransformationConve_g[0], 0, sizeof(real_T) << 4U);
  localB->CoordinateTransformationConve_g[15] = 1.0;
  for (b_k = 0; b_k < 3; b_k++) {
    subsa_idx_1 = b_k << 2;
    localB->CoordinateTransformationConve_g[subsa_idx_1] = localB->R[3 * b_k];
    localB->CoordinateTransformationConve_g[subsa_idx_1 + 1] = localB->R[3 * b_k
      + 1];
    localB->CoordinateTransformationConve_g[subsa_idx_1 + 2] = localB->R[3 * b_k
      + 2];
    localB->CoordinateTransformationConve_g[b_k + 12] = localB->u1[b_k];
  }

  // End of MATLABSystem: '<S13>/Coordinate Transformation Conversion'
}

static void matlabCodegenHandle_matlabC_jbz(ros_slros_internal_block_GetP_T *obj)
{
  if (!obj->matlabCodegenIsDeleted) {
    obj->matlabCodegenIsDeleted = true;
  }
}

static void cartesian_tr_matlabCodegenHa_ku(ros_slros_internal_block_SetP_T *obj)
{
  if (!obj->matlabCodegenIsDeleted) {
    obj->matlabCodegenIsDeleted = true;
  }
}

static void matlabCodegenHandle_matlabCo_jb(ros_slros_internal_block_Subs_T *obj)
{
  if (!obj->matlabCodegenIsDeleted) {
    obj->matlabCodegenIsDeleted = true;
  }
}

static void matlabCodegenHandle_matlabCod_j(ros_slros_internal_block_Publ_T *obj)
{
  if (!obj->matlabCodegenIsDeleted) {
    obj->matlabCodegenIsDeleted = true;
  }
}

// Model step function
void cartesian_trajectory_planner_2_step(void)
{
  boolean_T exitg1;

  // SignalConversion generated from: '<S13>/Coordinate Transformation Conversion' incorporates:
  //   MATLABSystem: '<S13>/Get Parameter3'
  //   MATLABSystem: '<S13>/Get Parameter4'
  //   MATLABSystem: '<S13>/Get Parameter5'
  //   MATLABSystem: '<S13>/Get Parameter6'

  ParamGet_cartesian_trajectory_planner_2_278.get_parameter
    (&cartesian_trajectory_planner__B.TmpSignalConversionAtCoordi[0]);
  ParamGet_cartesian_trajectory_planner_2_279.get_parameter
    (&cartesian_trajectory_planner__B.TmpSignalConversionAtCoordi[1]);
  ParamGet_cartesian_trajectory_planner_2_280.get_parameter
    (&cartesian_trajectory_planner__B.TmpSignalConversionAtCoordi[2]);
  ParamGet_cartesian_trajectory_planner_2_281.get_parameter
    (&cartesian_trajectory_planner__B.TmpSignalConversionAtCoordi[3]);

  // SignalConversion generated from: '<S13>/Coordinate Transformation Conversion' incorporates:
  //   MATLABSystem: '<S13>/Get Parameter'
  //   MATLABSystem: '<S13>/Get Parameter1'
  //   MATLABSystem: '<S13>/Get Parameter2'

  ParamGet_cartesian_trajectory_planner_2_275.get_parameter
    (&cartesian_trajectory_planner__B.TmpSignalConversionAtCoor_p[0]);
  ParamGet_cartesian_trajectory_planner_2_276.get_parameter
    (&cartesian_trajectory_planner__B.TmpSignalConversionAtCoor_p[1]);
  ParamGet_cartesian_trajectory_planner_2_277.get_parameter
    (&cartesian_trajectory_planner__B.TmpSignalConversionAtCoor_p[2]);
  CoordinateTransformationConvers
    (cartesian_trajectory_planner__B.TmpSignalConversionAtCoordi,
     cartesian_trajectory_planner__B.TmpSignalConversionAtCoor_p,
     &cartesian_trajectory_planner__B.CoordinateTransformationConve_p);

  // SignalConversion generated from: '<S14>/Coordinate Transformation Conversion' incorporates:
  //   MATLABSystem: '<S14>/Get Parameter3'
  //   MATLABSystem: '<S14>/Get Parameter4'
  //   MATLABSystem: '<S14>/Get Parameter5'
  //   MATLABSystem: '<S14>/Get Parameter6'

  ParamGet_cartesian_trajectory_planner_2_290.get_parameter
    (&cartesian_trajectory_planner__B.TmpSignalConversionAtCoor_l[0]);
  ParamGet_cartesian_trajectory_planner_2_291.get_parameter
    (&cartesian_trajectory_planner__B.TmpSignalConversionAtCoor_l[1]);
  ParamGet_cartesian_trajectory_planner_2_292.get_parameter
    (&cartesian_trajectory_planner__B.TmpSignalConversionAtCoor_l[2]);
  ParamGet_cartesian_trajectory_planner_2_293.get_parameter
    (&cartesian_trajectory_planner__B.TmpSignalConversionAtCoor_l[3]);

  // SignalConversion generated from: '<S14>/Coordinate Transformation Conversion' incorporates:
  //   MATLABSystem: '<S14>/Get Parameter'
  //   MATLABSystem: '<S14>/Get Parameter1'
  //   MATLABSystem: '<S14>/Get Parameter2'

  ParamGet_cartesian_trajectory_planner_2_287.get_parameter
    (&cartesian_trajectory_planner__B.TmpSignalConversionAtCoo_pn[0]);
  ParamGet_cartesian_trajectory_planner_2_288.get_parameter
    (&cartesian_trajectory_planner__B.TmpSignalConversionAtCoo_pn[1]);
  ParamGet_cartesian_trajectory_planner_2_289.get_parameter
    (&cartesian_trajectory_planner__B.TmpSignalConversionAtCoo_pn[2]);
  CoordinateTransformationConvers
    (cartesian_trajectory_planner__B.TmpSignalConversionAtCoor_l,
     cartesian_trajectory_planner__B.TmpSignalConversionAtCoo_pn,
     &cartesian_trajectory_planner__B.CoordinateTransformationConv_pn);

  // SignalConversion generated from: '<S2>/MATLAB System' incorporates:
  //   MATLABSystem: '<S11>/Get Parameter'
  //   MATLABSystem: '<S11>/Get Parameter1'
  //   MATLABSystem: '<S11>/Get Parameter2'
  //   MATLABSystem: '<S11>/Get Parameter3'
  //   MATLABSystem: '<S11>/Get Parameter4'
  //   MATLABSystem: '<S11>/Get Parameter5'

  ParamGet_cartesian_trajectory_planner_2_299.get_parameter
    (&cartesian_trajectory_planner__B.TmpSignalConversionAtMATLAB[0]);
  ParamGet_cartesian_trajectory_planner_2_300.get_parameter
    (&cartesian_trajectory_planner__B.TmpSignalConversionAtMATLAB[1]);
  ParamGet_cartesian_trajectory_planner_2_301.get_parameter
    (&cartesian_trajectory_planner__B.TmpSignalConversionAtMATLAB[2]);
  ParamGet_cartesian_trajectory_planner_2_302.get_parameter
    (&cartesian_trajectory_planner__B.TmpSignalConversionAtMATLAB[3]);
  ParamGet_cartesian_trajectory_planner_2_303.get_parameter
    (&cartesian_trajectory_planner__B.TmpSignalConversionAtMATLAB[4]);
  ParamGet_cartesian_trajectory_planner_2_304.get_parameter
    (&cartesian_trajectory_planner__B.TmpSignalConversionAtMATLAB[5]);

  // Delay: '<Root>/Delay'
  for (cartesian_trajectory_planner__B.i = 0; cartesian_trajectory_planner__B.i <
       6; cartesian_trajectory_planner__B.i++) {
    cartesian_trajectory_planner__B.Delay[cartesian_trajectory_planner__B.i] =
      cartesian_trajectory_planner_DW.Delay_DSTATE[cartesian_trajectory_planner__B.i];
  }

  // End of Delay: '<Root>/Delay'
  cartesian_trajecto_MATLABSystem
    (cartesian_trajectory_planner__B.CoordinateTransformationConv_pn.CoordinateTransformationConve_g,
     cartesian_trajectory_planner__B.TmpSignalConversionAtMATLAB,
     cartesian_trajectory_planner__B.Delay,
     &cartesian_trajectory_planner__B.MATLABSystem_p,
     &cartesian_trajectory_planner_DW.MATLABSystem_p);

  // SignalConversion generated from: '<Root>/Matrix Concatenate' incorporates:
  //   SignalConversion generated from: '<S3>/MATLAB System'

  for (cartesian_trajectory_planner__B.i = 0; cartesian_trajectory_planner__B.i <
       6; cartesian_trajectory_planner__B.i++) {
    cartesian_trajectory_planner__B.MatrixConcatenate[cartesian_trajectory_planner__B.i
      + 6] =
      cartesian_trajectory_planner__B.MATLABSystem_p.MATLABSystem_o1[cartesian_trajectory_planner__B.i];
  }

  // End of SignalConversion generated from: '<Root>/Matrix Concatenate'

  // Delay: '<Root>/Delay1'
  for (cartesian_trajectory_planner__B.i = 0; cartesian_trajectory_planner__B.i <
       6; cartesian_trajectory_planner__B.i++) {
    cartesian_trajectory_planner__B.Delay1[cartesian_trajectory_planner__B.i] =
      cartesian_trajectory_planner_DW.Delay1_DSTATE[cartesian_trajectory_planner__B.i];
  }

  // End of Delay: '<Root>/Delay1'
  cartesian_trajecto_MATLABSystem
    (cartesian_trajectory_planner__B.CoordinateTransformationConve_p.CoordinateTransformationConve_g,
     cartesian_trajectory_planner__B.TmpSignalConversionAtMATLAB,
     cartesian_trajectory_planner__B.Delay1,
     &cartesian_trajectory_planner__B.MATLABSystem,
     &cartesian_trajectory_planner_DW.MATLABSystem);

  // SignalConversion generated from: '<Root>/Matrix Concatenate' incorporates:
  //   SignalConversion generated from: '<S2>/MATLAB System'

  for (cartesian_trajectory_planner__B.i = 0; cartesian_trajectory_planner__B.i <
       6; cartesian_trajectory_planner__B.i++) {
    cartesian_trajectory_planner__B.MatrixConcatenate[cartesian_trajectory_planner__B.i]
      =
      cartesian_trajectory_planner__B.MATLABSystem.MATLABSystem_o1[cartesian_trajectory_planner__B.i];
  }

  // End of SignalConversion generated from: '<Root>/Matrix Concatenate'

  // MATLABSystem: '<S10>/Set Parameter' incorporates:
  //   SignalConversion generated from: '<S2>/MATLAB System'

  ParamSet_cartesian_trajectory_planner_2_325.set_parameter
    (cartesian_trajectory_planner__B.MATLABSystem.MATLABSystem_o1[0]);

  // MATLABSystem: '<S10>/Set Parameter1' incorporates:
  //   SignalConversion generated from: '<S2>/MATLAB System'

  ParamSet_cartesian_trajectory_planner_2_326.set_parameter
    (cartesian_trajectory_planner__B.MATLABSystem.MATLABSystem_o1[1]);

  // MATLABSystem: '<S10>/Set Parameter2' incorporates:
  //   SignalConversion generated from: '<S2>/MATLAB System'

  ParamSet_cartesian_trajectory_planner_2_327.set_parameter
    (cartesian_trajectory_planner__B.MATLABSystem.MATLABSystem_o1[2]);

  // MATLABSystem: '<S10>/Set Parameter3' incorporates:
  //   SignalConversion generated from: '<S2>/MATLAB System'

  ParamSet_cartesian_trajectory_planner_2_328.set_parameter
    (cartesian_trajectory_planner__B.MATLABSystem.MATLABSystem_o1[3]);

  // MATLABSystem: '<S10>/Set Parameter4' incorporates:
  //   SignalConversion generated from: '<S2>/MATLAB System'

  ParamSet_cartesian_trajectory_planner_2_329.set_parameter
    (cartesian_trajectory_planner__B.MATLABSystem.MATLABSystem_o1[5]);

  // MATLABSystem: '<S10>/Set Parameter5' incorporates:
  //   SignalConversion generated from: '<S2>/MATLAB System'

  ParamSet_cartesian_trajectory_planner_2_330.set_parameter
    (cartesian_trajectory_planner__B.MATLABSystem.MATLABSystem_o1[4]);

  // Outputs for Atomic SubSystem: '<Root>/Subscribe'
  // MATLABSystem: '<S8>/SourceBlock' incorporates:
  //   Inport: '<S12>/In1'

  cartesian_trajectory_planner__B.b_varargout_1 =
    Sub_cartesian_trajectory_planner_2_267.getLatestMessage
    (&cartesian_trajectory_planner__B.b_varargout_2);

  // Outputs for Enabled SubSystem: '<S8>/Enabled Subsystem' incorporates:
  //   EnablePort: '<S12>/Enable'

  if (cartesian_trajectory_planner__B.b_varargout_1) {
    cartesian_trajectory_planner__B.In1 =
      cartesian_trajectory_planner__B.b_varargout_2;
  }

  // End of MATLABSystem: '<S8>/SourceBlock'
  // End of Outputs for SubSystem: '<S8>/Enabled Subsystem'
  // End of Outputs for SubSystem: '<Root>/Subscribe'

  // MATLABSystem: '<S9>/Get Parameter'
  ParamGet_cartesian_trajectory_planner_2_270.get_parameter
    (&cartesian_trajectory_planner__B.t_up);

  // MATLAB Function: '<Root>/MATLAB Function1' incorporates:
  //   MATLABSystem: '<S9>/Get Parameter'

  cartesian_trajectory_planner__B.delayed_time =
    (cartesian_trajectory_planner__B.In1.Clock_.Nsec / 1.0E+9 +
     cartesian_trajectory_planner__B.In1.Clock_.Sec) -
    cartesian_trajectory_planner__B.t_up;
  if (cartesian_trajectory_planner__B.delayed_time < 0.0) {
    cartesian_trajectory_planner__B.delayed_time = 0.0;
  }

  // End of MATLAB Function: '<Root>/MATLAB Function1'

  // MATLABSystem: '<S9>/Get Parameter2'
  ParamGet_cartesian_trajectory_planner_2_271.get_parameter
    (&cartesian_trajectory_planner__B.t_up);

  // MATLABSystem: '<S9>/Get Parameter3'
  ParamGet_cartesian_trajectory_planner_2_272.get_parameter
    (&cartesian_trajectory_planner__B.dist);

  // MATLAB Function: '<Root>/MATLAB Function3' incorporates:
  //   MATLABSystem: '<S9>/Get Parameter2'
  //   MATLABSystem: '<S9>/Get Parameter3'

  for (cartesian_trajectory_planner__B.i = 0; cartesian_trajectory_planner__B.i <
       6; cartesian_trajectory_planner__B.i++) {
    cartesian_trajectory_planner__B.q[cartesian_trajectory_planner__B.i] =
      cartesian_trajectory_planner__B.MatrixConcatenate[cartesian_trajectory_planner__B.i];
    cartesian_trajectory_planner__B.qd[cartesian_trajectory_planner__B.i] = 0.0;
    cartesian_trajectory_planner__B.qdd[cartesian_trajectory_planner__B.i] = 0.0;
    cartesian_trajectory_planner__B.max_vel[cartesian_trajectory_planner__B.i] =
      cartesian_trajectory_planner__B.t_up;
    cartesian_trajectory_planner__B.acc[cartesian_trajectory_planner__B.i] =
      cartesian_trajectory_planner__B.dist;
  }

  memset(&cartesian_trajectory_planner__B.t[0], 0, 18U * sizeof(real_T));
  for (cartesian_trajectory_planner__B.i = 0; cartesian_trajectory_planner__B.i <
       6; cartesian_trajectory_planner__B.i++) {
    cartesian_trajectory_planner__B.t_up =
      cartesian_trajectory_planner__B.max_vel[cartesian_trajectory_planner__B.i]
      / cartesian_trajectory_planner__B.acc[cartesian_trajectory_planner__B.i];
    cartesian_trajectory_planner__B.dist =
      cartesian_trajectory_planner__B.MatrixConcatenate[cartesian_trajectory_planner__B.i
      + 6] -
      cartesian_trajectory_planner__B.MatrixConcatenate[cartesian_trajectory_planner__B.i];
    if (cartesian_trajectory_planner__B.dist < 0.0) {
      cartesian_trajectory_planner__B.signes[cartesian_trajectory_planner__B.i] =
        -1.0;
    } else if (cartesian_trajectory_planner__B.dist > 0.0) {
      cartesian_trajectory_planner__B.signes[cartesian_trajectory_planner__B.i] =
        1.0;
    } else if (cartesian_trajectory_planner__B.dist == 0.0) {
      cartesian_trajectory_planner__B.signes[cartesian_trajectory_planner__B.i] =
        0.0;
    } else {
      cartesian_trajectory_planner__B.signes[cartesian_trajectory_planner__B.i] =
        (rtNaN);
    }

    cartesian_trajectory_planner__B.dist = fabs
      (cartesian_trajectory_planner__B.dist);
    cartesian_trajectory_planner__B.d = cartesian_trajectory_planner__B.t_up *
      cartesian_trajectory_planner__B.max_vel[cartesian_trajectory_planner__B.i];
    if (cartesian_trajectory_planner__B.dist < cartesian_trajectory_planner__B.d)
    {
      cartesian_trajectory_planner__B.t_up = sqrt
        (cartesian_trajectory_planner__B.dist /
         cartesian_trajectory_planner__B.acc[cartesian_trajectory_planner__B.i]);
      cartesian_trajectory_planner__B.t[cartesian_trajectory_planner__B.i + 6] =
        0.0;
      cartesian_trajectory_planner__B.t[cartesian_trajectory_planner__B.i] =
        cartesian_trajectory_planner__B.t_up;
      cartesian_trajectory_planner__B.t[cartesian_trajectory_planner__B.i + 12] =
        cartesian_trajectory_planner__B.t_up;
      cartesian_trajectory_planner__B.act_max_vel[cartesian_trajectory_planner__B.i]
        = cartesian_trajectory_planner__B.t_up *
        cartesian_trajectory_planner__B.acc[cartesian_trajectory_planner__B.i];
    } else {
      cartesian_trajectory_planner__B.t[cartesian_trajectory_planner__B.i] =
        cartesian_trajectory_planner__B.t_up;
      cartesian_trajectory_planner__B.t[cartesian_trajectory_planner__B.i + 12] =
        cartesian_trajectory_planner__B.t_up;
      cartesian_trajectory_planner__B.t[cartesian_trajectory_planner__B.i + 6] =
        (cartesian_trajectory_planner__B.dist -
         cartesian_trajectory_planner__B.d) /
        cartesian_trajectory_planner__B.max_vel[cartesian_trajectory_planner__B.i];
      cartesian_trajectory_planner__B.act_max_vel[cartesian_trajectory_planner__B.i]
        =
        cartesian_trajectory_planner__B.max_vel[cartesian_trajectory_planner__B.i];
    }
  }

  // MATLAB Function: '<Root>/MATLAB Function2' incorporates:
  //   Constant: '<S1>/Constant'

  cartesian_trajectory_planner__B.msg =
    cartesian_trajectory_planner__P.Constant_Value;
  cartesian_trajectory_planner__B.msg.Velocities_SL_Info.CurrentLength = 6U;
  cartesian_trajectory_planner__B.msg.Positions_SL_Info.CurrentLength = 6U;
  cartesian_trajectory_planner__B.msg.Accelerations_SL_Info.CurrentLength = 6U;
  for (cartesian_trajectory_planner__B.i = 0; cartesian_trajectory_planner__B.i <
       6; cartesian_trajectory_planner__B.i++) {
    // MATLAB Function: '<Root>/MATLAB Function3'
    cartesian_trajectory_planner__B.t_up =
      cartesian_trajectory_planner__B.delayed_time;
    cartesian_trajectory_planner__B.k = 0;
    exitg1 = false;
    while ((!exitg1) && (cartesian_trajectory_planner__B.k < 3)) {
      cartesian_trajectory_planner__B.i_m = 6 *
        cartesian_trajectory_planner__B.k + cartesian_trajectory_planner__B.i;
      cartesian_trajectory_planner__B.dist =
        cartesian_trajectory_planner__B.t_up -
        cartesian_trajectory_planner__B.t[cartesian_trajectory_planner__B.i_m];
      if (cartesian_trajectory_planner__B.dist < 0.0) {
        switch (cartesian_trajectory_planner__B.k + 1) {
         case 1:
          cartesian_trajectory_planner__B.q[cartesian_trajectory_planner__B.i] +=
            0.5 *
            cartesian_trajectory_planner__B.acc[cartesian_trajectory_planner__B.i]
            * (cartesian_trajectory_planner__B.t_up *
               cartesian_trajectory_planner__B.t_up) *
            cartesian_trajectory_planner__B.signes[cartesian_trajectory_planner__B.i];
          cartesian_trajectory_planner__B.qd[cartesian_trajectory_planner__B.i] =
            cartesian_trajectory_planner__B.acc[cartesian_trajectory_planner__B.i]
            * cartesian_trajectory_planner__B.t_up *
            cartesian_trajectory_planner__B.signes[cartesian_trajectory_planner__B.i];
          cartesian_trajectory_planner__B.qdd[cartesian_trajectory_planner__B.i]
            =
            cartesian_trajectory_planner__B.acc[cartesian_trajectory_planner__B.i]
            * cartesian_trajectory_planner__B.signes[cartesian_trajectory_planner__B.i];
          break;

         case 2:
          cartesian_trajectory_planner__B.q[cartesian_trajectory_planner__B.i] +=
            cartesian_trajectory_planner__B.max_vel[cartesian_trajectory_planner__B.i]
            * cartesian_trajectory_planner__B.t_up *
            cartesian_trajectory_planner__B.signes[cartesian_trajectory_planner__B.i];
          cartesian_trajectory_planner__B.qd[cartesian_trajectory_planner__B.i] =
            cartesian_trajectory_planner__B.max_vel[cartesian_trajectory_planner__B.i]
            * cartesian_trajectory_planner__B.signes[cartesian_trajectory_planner__B.i];
          cartesian_trajectory_planner__B.qdd[cartesian_trajectory_planner__B.i]
            = 0.0;
          break;

         default:
          cartesian_trajectory_planner__B.q[cartesian_trajectory_planner__B.i] +=
            (cartesian_trajectory_planner__B.act_max_vel[0] *
             cartesian_trajectory_planner__B.t_up - 0.5 *
             cartesian_trajectory_planner__B.acc[cartesian_trajectory_planner__B.i]
             * (cartesian_trajectory_planner__B.t_up *
                cartesian_trajectory_planner__B.t_up)) *
            cartesian_trajectory_planner__B.signes[cartesian_trajectory_planner__B.i];
          cartesian_trajectory_planner__B.qd[cartesian_trajectory_planner__B.i] =
            (cartesian_trajectory_planner__B.act_max_vel[cartesian_trajectory_planner__B.i]
             - cartesian_trajectory_planner__B.t_up *
             cartesian_trajectory_planner__B.acc[cartesian_trajectory_planner__B.i])
            * cartesian_trajectory_planner__B.signes[cartesian_trajectory_planner__B.i];
          cartesian_trajectory_planner__B.qdd[cartesian_trajectory_planner__B.i]
            =
            -cartesian_trajectory_planner__B.acc[cartesian_trajectory_planner__B.i]
            * cartesian_trajectory_planner__B.signes[cartesian_trajectory_planner__B.i];
          break;
        }

        exitg1 = true;
      } else {
        cartesian_trajectory_planner__B.t_up =
          cartesian_trajectory_planner__B.dist;
        switch (cartesian_trajectory_planner__B.k + 1) {
         case 1:
          cartesian_trajectory_planner__B.q[cartesian_trajectory_planner__B.i] +=
            cartesian_trajectory_planner__B.t[cartesian_trajectory_planner__B.i_m]
            * cartesian_trajectory_planner__B.t[cartesian_trajectory_planner__B.i_m]
            * (0.5 *
               cartesian_trajectory_planner__B.acc[cartesian_trajectory_planner__B.i])
            * cartesian_trajectory_planner__B.signes[cartesian_trajectory_planner__B.i];
          cartesian_trajectory_planner__B.qd[cartesian_trajectory_planner__B.i] =
            cartesian_trajectory_planner__B.act_max_vel[cartesian_trajectory_planner__B.i]
            * cartesian_trajectory_planner__B.signes[cartesian_trajectory_planner__B.i];
          cartesian_trajectory_planner__B.qdd[cartesian_trajectory_planner__B.i]
            =
            cartesian_trajectory_planner__B.acc[cartesian_trajectory_planner__B.i]
            * cartesian_trajectory_planner__B.signes[cartesian_trajectory_planner__B.i];
          break;

         case 2:
          cartesian_trajectory_planner__B.q[cartesian_trajectory_planner__B.i] +=
            cartesian_trajectory_planner__B.t[cartesian_trajectory_planner__B.i_m]
            * cartesian_trajectory_planner__B.max_vel[cartesian_trajectory_planner__B.i]
            * cartesian_trajectory_planner__B.signes[cartesian_trajectory_planner__B.i];
          cartesian_trajectory_planner__B.qd[cartesian_trajectory_planner__B.i] =
            cartesian_trajectory_planner__B.act_max_vel[cartesian_trajectory_planner__B.i]
            * cartesian_trajectory_planner__B.signes[cartesian_trajectory_planner__B.i];
          cartesian_trajectory_planner__B.qdd[cartesian_trajectory_planner__B.i]
            = 0.0;
          break;

         default:
          cartesian_trajectory_planner__B.q[cartesian_trajectory_planner__B.i] +=
            (cartesian_trajectory_planner__B.t[cartesian_trajectory_planner__B.i_m]
             * cartesian_trajectory_planner__B.act_max_vel[cartesian_trajectory_planner__B.i]
             - cartesian_trajectory_planner__B.t[cartesian_trajectory_planner__B.i_m]
             * cartesian_trajectory_planner__B.t[cartesian_trajectory_planner__B.i_m]
             * (0.5 *
                cartesian_trajectory_planner__B.acc[cartesian_trajectory_planner__B.i]))
            * cartesian_trajectory_planner__B.signes[cartesian_trajectory_planner__B.i];
          cartesian_trajectory_planner__B.qd[cartesian_trajectory_planner__B.i] =
            0.0;
          cartesian_trajectory_planner__B.qdd[cartesian_trajectory_planner__B.i]
            = 0.0;
          break;
        }

        if (cartesian_trajectory_planner__B.dist < 0.0) {
          exitg1 = true;
        } else {
          cartesian_trajectory_planner__B.k++;
        }
      }
    }

    if (cartesian_trajectory_planner__B.delayed_time == 0.0) {
      cartesian_trajectory_planner__B.qdd[cartesian_trajectory_planner__B.i] =
        0.0;
    }

    // MATLAB Function: '<Root>/MATLAB Function2'
    cartesian_trajectory_planner__B.msg.Velocities[cartesian_trajectory_planner__B.i]
      = cartesian_trajectory_planner__B.qd[cartesian_trajectory_planner__B.i];
    cartesian_trajectory_planner__B.msg.Positions[cartesian_trajectory_planner__B.i]
      = cartesian_trajectory_planner__B.q[cartesian_trajectory_planner__B.i];
    cartesian_trajectory_planner__B.msg.Accelerations[cartesian_trajectory_planner__B.i]
      = cartesian_trajectory_planner__B.qdd[cartesian_trajectory_planner__B.i];
  }

  // Outputs for Atomic SubSystem: '<Root>/Publish'
  // MATLABSystem: '<S7>/SinkBlock'
  Pub_cartesian_trajectory_planner_2_266.publish
    (&cartesian_trajectory_planner__B.msg);

  // End of Outputs for SubSystem: '<Root>/Publish'

  // Update for Delay: '<Root>/Delay' incorporates:
  //   SignalConversion generated from: '<S3>/MATLAB System'

  for (cartesian_trajectory_planner__B.i = 0; cartesian_trajectory_planner__B.i <
       6; cartesian_trajectory_planner__B.i++) {
    cartesian_trajectory_planner_DW.Delay_DSTATE[cartesian_trajectory_planner__B.i]
      =
      cartesian_trajectory_planner__B.MATLABSystem_p.MATLABSystem_o1[cartesian_trajectory_planner__B.i];
  }

  // End of Update for Delay: '<Root>/Delay'

  // Update for Delay: '<Root>/Delay1' incorporates:
  //   SignalConversion generated from: '<S2>/MATLAB System'

  for (cartesian_trajectory_planner__B.i = 0; cartesian_trajectory_planner__B.i <
       6; cartesian_trajectory_planner__B.i++) {
    cartesian_trajectory_planner_DW.Delay1_DSTATE[cartesian_trajectory_planner__B.i]
      =
      cartesian_trajectory_planner__B.MATLABSystem.MATLABSystem_o1[cartesian_trajectory_planner__B.i];
  }

  // End of Update for Delay: '<Root>/Delay1'
}

// Model initialize function
void cartesian_trajectory_planner_2_initialize(void)
{
  // Registration code

  // initialize non-finites
  rt_InitInfAndNaN(sizeof(real_T));

  {
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

    static const char_T tmp_l[11] = { '/', 'q', '1', '_', 'i', 'n', 'i', 't',
      'i', 'a', 'l' };

    static const char_T tmp_m[11] = { '/', 'q', '2', '_', 'i', 'n', 'i', 't',
      'i', 'a', 'l' };

    static const char_T tmp_n[11] = { '/', 'q', '3', '_', 'i', 'n', 'i', 't',
      'i', 'a', 'l' };

    static const char_T tmp_o[11] = { '/', 'q', '4', '_', 'i', 'n', 'i', 't',
      'i', 'a', 'l' };

    static const char_T tmp_p[11] = { '/', 'q', '6', '_', 'i', 'n', 'i', 't',
      'i', 'a', 'l' };

    static const char_T tmp_q[11] = { '/', 'q', '5', '_', 'i', 'n', 'i', 't',
      'i', 'a', 'l' };

    static const char_T tmp_r[12] = { '/', 's', 't', 'a', 'r', 't', '_', 'd',
      'e', 'l', 'a', 'y' };

    static const char_T tmp_s[12] = { '/', 'm', 'a', 'x', '_', 'a', 'n', 'g',
      '_', 'v', 'e', 'l' };

    static const char_T tmp_t[12] = { '/', 'm', 'a', 'x', '_', 'a', 'n', 'g',
      '_', 'a', 'c', 'c' };

    // SystemInitialize for Atomic SubSystem: '<Root>/Subscribe'
    // SystemInitialize for Enabled SubSystem: '<S8>/Enabled Subsystem'
    // SystemInitialize for Outport: '<S12>/Out1'
    cartesian_trajectory_planner__B.In1 =
      cartesian_trajectory_planner__P.Out1_Y0;

    // End of SystemInitialize for SubSystem: '<S8>/Enabled Subsystem'

    // Start for MATLABSystem: '<S8>/SourceBlock'
    cartesian_trajectory_planner_DW.obj_cs.matlabCodegenIsDeleted = true;
    cartesian_trajectory_planner_DW.obj_cs.isInitialized = 0;
    cartesian_trajectory_planner_DW.obj_cs.matlabCodegenIsDeleted = false;
    cartesian_trajectory_planner_DW.obj_cs.isSetupComplete = false;
    cartesian_trajectory_planner_DW.obj_cs.isInitialized = 1;
    for (cartesian_trajectory_planner__B.i_c = 0;
         cartesian_trajectory_planner__B.i_c < 6;
         cartesian_trajectory_planner__B.i_c++) {
      // InitializeConditions for Delay: '<Root>/Delay'
      cartesian_trajectory_planner_DW.Delay_DSTATE[cartesian_trajectory_planner__B.i_c]
        =
        cartesian_trajectory_planner__P.Delay_InitialCondition[cartesian_trajectory_planner__B.i_c];

      // InitializeConditions for Delay: '<Root>/Delay1'
      cartesian_trajectory_planner_DW.Delay1_DSTATE[cartesian_trajectory_planner__B.i_c]
        =
        cartesian_trajectory_planner__P.Delay1_InitialCondition[cartesian_trajectory_planner__B.i_c];

      // Start for MATLABSystem: '<S8>/SourceBlock'
      cartesian_trajectory_planner__B.cv4[cartesian_trajectory_planner__B.i_c] =
        tmp[cartesian_trajectory_planner__B.i_c];
    }

    // Start for MATLABSystem: '<S8>/SourceBlock'
    cartesian_trajectory_planner__B.cv4[6] = '\x00';
    Sub_cartesian_trajectory_planner_2_267.createSubscriber
      (cartesian_trajectory_planner__B.cv4, 1);
    cartesian_trajectory_planner_DW.obj_cs.isSetupComplete = true;

    // End of SystemInitialize for SubSystem: '<Root>/Subscribe'

    // SystemInitialize for Atomic SubSystem: '<Root>/Publish'
    // Start for MATLABSystem: '<S7>/SinkBlock'
    cartesian_trajectory_planner_DW.obj_ej.matlabCodegenIsDeleted = true;
    cartesian_trajectory_planner_DW.obj_ej.isInitialized = 0;
    cartesian_trajectory_planner_DW.obj_ej.matlabCodegenIsDeleted = false;
    cartesian_trajectory_planner_DW.obj_ej.isSetupComplete = false;
    cartesian_trajectory_planner_DW.obj_ej.isInitialized = 1;
    for (cartesian_trajectory_planner__B.i_c = 0;
         cartesian_trajectory_planner__B.i_c < 17;
         cartesian_trajectory_planner__B.i_c++) {
      cartesian_trajectory_planner__B.cv[cartesian_trajectory_planner__B.i_c] =
        tmp_0[cartesian_trajectory_planner__B.i_c];
    }

    cartesian_trajectory_planner__B.cv[17] = '\x00';
    Pub_cartesian_trajectory_planner_2_266.createPublisher
      (cartesian_trajectory_planner__B.cv, 1);
    cartesian_trajectory_planner_DW.obj_ej.isSetupComplete = true;

    // End of Start for MATLABSystem: '<S7>/SinkBlock'
    // End of SystemInitialize for SubSystem: '<Root>/Publish'

    // Start for MATLABSystem: '<S13>/Get Parameter3'
    cartesian_trajectory_planner_DW.obj_e0.matlabCodegenIsDeleted = true;
    cartesian_trajectory_planner_DW.obj_e0.isInitialized = 0;
    cartesian_trajectory_planner_DW.obj_e0.matlabCodegenIsDeleted = false;
    cartesian_trajectory_planner_DW.obj_e0.isSetupComplete = false;
    cartesian_trajectory_planner_DW.obj_e0.isInitialized = 1;
    for (cartesian_trajectory_planner__B.i_c = 0;
         cartesian_trajectory_planner__B.i_c < 5;
         cartesian_trajectory_planner__B.i_c++) {
      cartesian_trajectory_planner__B.cv5[cartesian_trajectory_planner__B.i_c] =
        tmp_1[cartesian_trajectory_planner__B.i_c];
    }

    cartesian_trajectory_planner__B.cv5[5] = '\x00';
    ParamGet_cartesian_trajectory_planner_2_278.initialize
      (cartesian_trajectory_planner__B.cv5);
    ParamGet_cartesian_trajectory_planner_2_278.initialize_error_codes(0, 1, 2,
      3);
    ParamGet_cartesian_trajectory_planner_2_278.set_initial_value(0.0);
    cartesian_trajectory_planner_DW.obj_e0.isSetupComplete = true;

    // End of Start for MATLABSystem: '<S13>/Get Parameter3'

    // Start for MATLABSystem: '<S13>/Get Parameter4'
    cartesian_trajectory_planner_DW.obj_p.matlabCodegenIsDeleted = true;
    cartesian_trajectory_planner_DW.obj_p.isInitialized = 0;
    cartesian_trajectory_planner_DW.obj_p.matlabCodegenIsDeleted = false;
    cartesian_trajectory_planner_DW.obj_p.isSetupComplete = false;
    cartesian_trajectory_planner_DW.obj_p.isInitialized = 1;
    for (cartesian_trajectory_planner__B.i_c = 0;
         cartesian_trajectory_planner__B.i_c < 5;
         cartesian_trajectory_planner__B.i_c++) {
      cartesian_trajectory_planner__B.cv5[cartesian_trajectory_planner__B.i_c] =
        tmp_2[cartesian_trajectory_planner__B.i_c];
    }

    cartesian_trajectory_planner__B.cv5[5] = '\x00';
    ParamGet_cartesian_trajectory_planner_2_279.initialize
      (cartesian_trajectory_planner__B.cv5);
    ParamGet_cartesian_trajectory_planner_2_279.initialize_error_codes(0, 1, 2,
      3);
    ParamGet_cartesian_trajectory_planner_2_279.set_initial_value(0.0);
    cartesian_trajectory_planner_DW.obj_p.isSetupComplete = true;

    // End of Start for MATLABSystem: '<S13>/Get Parameter4'

    // Start for MATLABSystem: '<S13>/Get Parameter5'
    cartesian_trajectory_planner_DW.obj_g.matlabCodegenIsDeleted = true;
    cartesian_trajectory_planner_DW.obj_g.isInitialized = 0;
    cartesian_trajectory_planner_DW.obj_g.matlabCodegenIsDeleted = false;
    cartesian_trajectory_planner_DW.obj_g.isSetupComplete = false;
    cartesian_trajectory_planner_DW.obj_g.isInitialized = 1;
    for (cartesian_trajectory_planner__B.i_c = 0;
         cartesian_trajectory_planner__B.i_c < 5;
         cartesian_trajectory_planner__B.i_c++) {
      cartesian_trajectory_planner__B.cv5[cartesian_trajectory_planner__B.i_c] =
        tmp_3[cartesian_trajectory_planner__B.i_c];
    }

    cartesian_trajectory_planner__B.cv5[5] = '\x00';
    ParamGet_cartesian_trajectory_planner_2_280.initialize
      (cartesian_trajectory_planner__B.cv5);
    ParamGet_cartesian_trajectory_planner_2_280.initialize_error_codes(0, 1, 2,
      3);
    ParamGet_cartesian_trajectory_planner_2_280.set_initial_value(0.0);
    cartesian_trajectory_planner_DW.obj_g.isSetupComplete = true;

    // End of Start for MATLABSystem: '<S13>/Get Parameter5'

    // Start for MATLABSystem: '<S13>/Get Parameter6'
    cartesian_trajectory_planner_DW.obj_or.matlabCodegenIsDeleted = true;
    cartesian_trajectory_planner_DW.obj_or.isInitialized = 0;
    cartesian_trajectory_planner_DW.obj_or.matlabCodegenIsDeleted = false;
    cartesian_trajectory_planner_DW.obj_or.isSetupComplete = false;
    cartesian_trajectory_planner_DW.obj_or.isInitialized = 1;
    for (cartesian_trajectory_planner__B.i_c = 0;
         cartesian_trajectory_planner__B.i_c < 5;
         cartesian_trajectory_planner__B.i_c++) {
      cartesian_trajectory_planner__B.cv5[cartesian_trajectory_planner__B.i_c] =
        tmp_4[cartesian_trajectory_planner__B.i_c];
    }

    cartesian_trajectory_planner__B.cv5[5] = '\x00';
    ParamGet_cartesian_trajectory_planner_2_281.initialize
      (cartesian_trajectory_planner__B.cv5);
    ParamGet_cartesian_trajectory_planner_2_281.initialize_error_codes(0, 1, 2,
      3);
    ParamGet_cartesian_trajectory_planner_2_281.set_initial_value(1.0);
    cartesian_trajectory_planner_DW.obj_or.isSetupComplete = true;

    // End of Start for MATLABSystem: '<S13>/Get Parameter6'

    // Start for MATLABSystem: '<S13>/Get Parameter'
    cartesian_trajectory_planner_DW.obj_a3.matlabCodegenIsDeleted = true;
    cartesian_trajectory_planner_DW.obj_a3.isInitialized = 0;
    cartesian_trajectory_planner_DW.obj_a3.matlabCodegenIsDeleted = false;
    cartesian_trajectory_planner_DW.obj_a3.isSetupComplete = false;
    cartesian_trajectory_planner_DW.obj_a3.isInitialized = 1;
    for (cartesian_trajectory_planner__B.i_c = 0;
         cartesian_trajectory_planner__B.i_c < 5;
         cartesian_trajectory_planner__B.i_c++) {
      cartesian_trajectory_planner__B.cv5[cartesian_trajectory_planner__B.i_c] =
        tmp_5[cartesian_trajectory_planner__B.i_c];
    }

    cartesian_trajectory_planner__B.cv5[5] = '\x00';
    ParamGet_cartesian_trajectory_planner_2_275.initialize
      (cartesian_trajectory_planner__B.cv5);
    ParamGet_cartesian_trajectory_planner_2_275.initialize_error_codes(0, 1, 2,
      3);
    ParamGet_cartesian_trajectory_planner_2_275.set_initial_value(-0.3);
    cartesian_trajectory_planner_DW.obj_a3.isSetupComplete = true;

    // End of Start for MATLABSystem: '<S13>/Get Parameter'

    // Start for MATLABSystem: '<S13>/Get Parameter1'
    cartesian_trajectory_planner_DW.obj_bh.matlabCodegenIsDeleted = true;
    cartesian_trajectory_planner_DW.obj_bh.isInitialized = 0;
    cartesian_trajectory_planner_DW.obj_bh.matlabCodegenIsDeleted = false;
    cartesian_trajectory_planner_DW.obj_bh.isSetupComplete = false;
    cartesian_trajectory_planner_DW.obj_bh.isInitialized = 1;
    for (cartesian_trajectory_planner__B.i_c = 0;
         cartesian_trajectory_planner__B.i_c < 5;
         cartesian_trajectory_planner__B.i_c++) {
      cartesian_trajectory_planner__B.cv5[cartesian_trajectory_planner__B.i_c] =
        tmp_6[cartesian_trajectory_planner__B.i_c];
    }

    cartesian_trajectory_planner__B.cv5[5] = '\x00';
    ParamGet_cartesian_trajectory_planner_2_276.initialize
      (cartesian_trajectory_planner__B.cv5);
    ParamGet_cartesian_trajectory_planner_2_276.initialize_error_codes(0, 1, 2,
      3);
    ParamGet_cartesian_trajectory_planner_2_276.set_initial_value(-0.3);
    cartesian_trajectory_planner_DW.obj_bh.isSetupComplete = true;

    // End of Start for MATLABSystem: '<S13>/Get Parameter1'

    // Start for MATLABSystem: '<S13>/Get Parameter2'
    cartesian_trajectory_planner_DW.obj_b0.matlabCodegenIsDeleted = true;
    cartesian_trajectory_planner_DW.obj_b0.isInitialized = 0;
    cartesian_trajectory_planner_DW.obj_b0.matlabCodegenIsDeleted = false;
    cartesian_trajectory_planner_DW.obj_b0.isSetupComplete = false;
    cartesian_trajectory_planner_DW.obj_b0.isInitialized = 1;
    for (cartesian_trajectory_planner__B.i_c = 0;
         cartesian_trajectory_planner__B.i_c < 5;
         cartesian_trajectory_planner__B.i_c++) {
      cartesian_trajectory_planner__B.cv5[cartesian_trajectory_planner__B.i_c] =
        tmp_7[cartesian_trajectory_planner__B.i_c];
    }

    cartesian_trajectory_planner__B.cv5[5] = '\x00';
    ParamGet_cartesian_trajectory_planner_2_277.initialize
      (cartesian_trajectory_planner__B.cv5);
    ParamGet_cartesian_trajectory_planner_2_277.initialize_error_codes(0, 1, 2,
      3);
    ParamGet_cartesian_trajectory_planner_2_277.set_initial_value(0.3);
    cartesian_trajectory_planner_DW.obj_b0.isSetupComplete = true;

    // End of Start for MATLABSystem: '<S13>/Get Parameter2'
    CoordinateTransformationCo_Init
      (&cartesian_trajectory_planner_DW.CoordinateTransformationConve_p);

    // Start for MATLABSystem: '<S14>/Get Parameter3'
    cartesian_trajectory_planner_DW.obj_o.matlabCodegenIsDeleted = true;
    cartesian_trajectory_planner_DW.obj_o.isInitialized = 0;
    cartesian_trajectory_planner_DW.obj_o.matlabCodegenIsDeleted = false;
    cartesian_trajectory_planner_DW.obj_o.isSetupComplete = false;
    cartesian_trajectory_planner_DW.obj_o.isInitialized = 1;
    for (cartesian_trajectory_planner__B.i_c = 0;
         cartesian_trajectory_planner__B.i_c < 5;
         cartesian_trajectory_planner__B.i_c++) {
      cartesian_trajectory_planner__B.cv5[cartesian_trajectory_planner__B.i_c] =
        tmp_8[cartesian_trajectory_planner__B.i_c];
    }

    cartesian_trajectory_planner__B.cv5[5] = '\x00';
    ParamGet_cartesian_trajectory_planner_2_290.initialize
      (cartesian_trajectory_planner__B.cv5);
    ParamGet_cartesian_trajectory_planner_2_290.initialize_error_codes(0, 1, 2,
      3);
    ParamGet_cartesian_trajectory_planner_2_290.set_initial_value(0.39);
    cartesian_trajectory_planner_DW.obj_o.isSetupComplete = true;

    // End of Start for MATLABSystem: '<S14>/Get Parameter3'

    // Start for MATLABSystem: '<S14>/Get Parameter4'
    cartesian_trajectory_planner_DW.obj_es.matlabCodegenIsDeleted = true;
    cartesian_trajectory_planner_DW.obj_es.isInitialized = 0;
    cartesian_trajectory_planner_DW.obj_es.matlabCodegenIsDeleted = false;
    cartesian_trajectory_planner_DW.obj_es.isSetupComplete = false;
    cartesian_trajectory_planner_DW.obj_es.isInitialized = 1;
    for (cartesian_trajectory_planner__B.i_c = 0;
         cartesian_trajectory_planner__B.i_c < 5;
         cartesian_trajectory_planner__B.i_c++) {
      cartesian_trajectory_planner__B.cv5[cartesian_trajectory_planner__B.i_c] =
        tmp_9[cartesian_trajectory_planner__B.i_c];
    }

    cartesian_trajectory_planner__B.cv5[5] = '\x00';
    ParamGet_cartesian_trajectory_planner_2_291.initialize
      (cartesian_trajectory_planner__B.cv5);
    ParamGet_cartesian_trajectory_planner_2_291.initialize_error_codes(0, 1, 2,
      3);
    ParamGet_cartesian_trajectory_planner_2_291.set_initial_value(0.89);
    cartesian_trajectory_planner_DW.obj_es.isSetupComplete = true;

    // End of Start for MATLABSystem: '<S14>/Get Parameter4'

    // Start for MATLABSystem: '<S14>/Get Parameter5'
    cartesian_trajectory_planner_DW.obj_f4.matlabCodegenIsDeleted = true;
    cartesian_trajectory_planner_DW.obj_f4.isInitialized = 0;
    cartesian_trajectory_planner_DW.obj_f4.matlabCodegenIsDeleted = false;
    cartesian_trajectory_planner_DW.obj_f4.isSetupComplete = false;
    cartesian_trajectory_planner_DW.obj_f4.isInitialized = 1;
    for (cartesian_trajectory_planner__B.i_c = 0;
         cartesian_trajectory_planner__B.i_c < 5;
         cartesian_trajectory_planner__B.i_c++) {
      cartesian_trajectory_planner__B.cv5[cartesian_trajectory_planner__B.i_c] =
        tmp_a[cartesian_trajectory_planner__B.i_c];
    }

    cartesian_trajectory_planner__B.cv5[5] = '\x00';
    ParamGet_cartesian_trajectory_planner_2_292.initialize
      (cartesian_trajectory_planner__B.cv5);
    ParamGet_cartesian_trajectory_planner_2_292.initialize_error_codes(0, 1, 2,
      3);
    ParamGet_cartesian_trajectory_planner_2_292.set_initial_value(0.2);
    cartesian_trajectory_planner_DW.obj_f4.isSetupComplete = true;

    // End of Start for MATLABSystem: '<S14>/Get Parameter5'

    // Start for MATLABSystem: '<S14>/Get Parameter6'
    cartesian_trajectory_planner_DW.obj_en.matlabCodegenIsDeleted = true;
    cartesian_trajectory_planner_DW.obj_en.isInitialized = 0;
    cartesian_trajectory_planner_DW.obj_en.matlabCodegenIsDeleted = false;
    cartesian_trajectory_planner_DW.obj_en.isSetupComplete = false;
    cartesian_trajectory_planner_DW.obj_en.isInitialized = 1;
    for (cartesian_trajectory_planner__B.i_c = 0;
         cartesian_trajectory_planner__B.i_c < 5;
         cartesian_trajectory_planner__B.i_c++) {
      cartesian_trajectory_planner__B.cv5[cartesian_trajectory_planner__B.i_c] =
        tmp_b[cartesian_trajectory_planner__B.i_c];
    }

    cartesian_trajectory_planner__B.cv5[5] = '\x00';
    ParamGet_cartesian_trajectory_planner_2_293.initialize
      (cartesian_trajectory_planner__B.cv5);
    ParamGet_cartesian_trajectory_planner_2_293.initialize_error_codes(0, 1, 2,
      3);
    ParamGet_cartesian_trajectory_planner_2_293.set_initial_value(-0.12);
    cartesian_trajectory_planner_DW.obj_en.isSetupComplete = true;

    // End of Start for MATLABSystem: '<S14>/Get Parameter6'

    // Start for MATLABSystem: '<S14>/Get Parameter'
    cartesian_trajectory_planner_DW.obj_b.matlabCodegenIsDeleted = true;
    cartesian_trajectory_planner_DW.obj_b.isInitialized = 0;
    cartesian_trajectory_planner_DW.obj_b.matlabCodegenIsDeleted = false;
    cartesian_trajectory_planner_DW.obj_b.isSetupComplete = false;
    cartesian_trajectory_planner_DW.obj_b.isInitialized = 1;
    for (cartesian_trajectory_planner__B.i_c = 0;
         cartesian_trajectory_planner__B.i_c < 5;
         cartesian_trajectory_planner__B.i_c++) {
      cartesian_trajectory_planner__B.cv5[cartesian_trajectory_planner__B.i_c] =
        tmp_c[cartesian_trajectory_planner__B.i_c];
    }

    cartesian_trajectory_planner__B.cv5[5] = '\x00';
    ParamGet_cartesian_trajectory_planner_2_287.initialize
      (cartesian_trajectory_planner__B.cv5);
    ParamGet_cartesian_trajectory_planner_2_287.initialize_error_codes(0, 1, 2,
      3);
    ParamGet_cartesian_trajectory_planner_2_287.set_initial_value(0.3);
    cartesian_trajectory_planner_DW.obj_b.isSetupComplete = true;

    // End of Start for MATLABSystem: '<S14>/Get Parameter'

    // Start for MATLABSystem: '<S14>/Get Parameter1'
    cartesian_trajectory_planner_DW.obj_i.matlabCodegenIsDeleted = true;
    cartesian_trajectory_planner_DW.obj_i.isInitialized = 0;
    cartesian_trajectory_planner_DW.obj_i.matlabCodegenIsDeleted = false;
    cartesian_trajectory_planner_DW.obj_i.isSetupComplete = false;
    cartesian_trajectory_planner_DW.obj_i.isInitialized = 1;
    for (cartesian_trajectory_planner__B.i_c = 0;
         cartesian_trajectory_planner__B.i_c < 5;
         cartesian_trajectory_planner__B.i_c++) {
      cartesian_trajectory_planner__B.cv5[cartesian_trajectory_planner__B.i_c] =
        tmp_d[cartesian_trajectory_planner__B.i_c];
    }

    cartesian_trajectory_planner__B.cv5[5] = '\x00';
    ParamGet_cartesian_trajectory_planner_2_288.initialize
      (cartesian_trajectory_planner__B.cv5);
    ParamGet_cartesian_trajectory_planner_2_288.initialize_error_codes(0, 1, 2,
      3);
    ParamGet_cartesian_trajectory_planner_2_288.set_initial_value(0.3);
    cartesian_trajectory_planner_DW.obj_i.isSetupComplete = true;

    // End of Start for MATLABSystem: '<S14>/Get Parameter1'

    // Start for MATLABSystem: '<S14>/Get Parameter2'
    cartesian_trajectory_planner_DW.obj_d.matlabCodegenIsDeleted = true;
    cartesian_trajectory_planner_DW.obj_d.isInitialized = 0;
    cartesian_trajectory_planner_DW.obj_d.matlabCodegenIsDeleted = false;
    cartesian_trajectory_planner_DW.obj_d.isSetupComplete = false;
    cartesian_trajectory_planner_DW.obj_d.isInitialized = 1;
    for (cartesian_trajectory_planner__B.i_c = 0;
         cartesian_trajectory_planner__B.i_c < 5;
         cartesian_trajectory_planner__B.i_c++) {
      cartesian_trajectory_planner__B.cv5[cartesian_trajectory_planner__B.i_c] =
        tmp_e[cartesian_trajectory_planner__B.i_c];
    }

    cartesian_trajectory_planner__B.cv5[5] = '\x00';
    ParamGet_cartesian_trajectory_planner_2_289.initialize
      (cartesian_trajectory_planner__B.cv5);
    ParamGet_cartesian_trajectory_planner_2_289.initialize_error_codes(0, 1, 2,
      3);
    ParamGet_cartesian_trajectory_planner_2_289.set_initial_value(0.3);
    cartesian_trajectory_planner_DW.obj_d.isSetupComplete = true;

    // End of Start for MATLABSystem: '<S14>/Get Parameter2'
    CoordinateTransformationCo_Init
      (&cartesian_trajectory_planner_DW.CoordinateTransformationConv_pn);

    // Start for MATLABSystem: '<S11>/Get Parameter'
    cartesian_trajectory_planner_DW.obj.matlabCodegenIsDeleted = true;
    cartesian_trajectory_planner_DW.obj.isInitialized = 0;
    cartesian_trajectory_planner_DW.obj.matlabCodegenIsDeleted = false;
    cartesian_trajectory_planner_DW.obj.isSetupComplete = false;
    cartesian_trajectory_planner_DW.obj.isInitialized = 1;
    for (cartesian_trajectory_planner__B.i_c = 0;
         cartesian_trajectory_planner__B.i_c < 10;
         cartesian_trajectory_planner__B.i_c++) {
      cartesian_trajectory_planner__B.cv3[cartesian_trajectory_planner__B.i_c] =
        tmp_f[cartesian_trajectory_planner__B.i_c];
    }

    cartesian_trajectory_planner__B.cv3[10] = '\x00';
    ParamGet_cartesian_trajectory_planner_2_299.initialize
      (cartesian_trajectory_planner__B.cv3);
    ParamGet_cartesian_trajectory_planner_2_299.initialize_error_codes(0, 1, 2,
      3);
    ParamGet_cartesian_trajectory_planner_2_299.set_initial_value(0.0);
    cartesian_trajectory_planner_DW.obj.isSetupComplete = true;

    // End of Start for MATLABSystem: '<S11>/Get Parameter'

    // Start for MATLABSystem: '<S11>/Get Parameter1'
    cartesian_trajectory_planner_DW.obj_m.matlabCodegenIsDeleted = true;
    cartesian_trajectory_planner_DW.obj_m.isInitialized = 0;
    cartesian_trajectory_planner_DW.obj_m.matlabCodegenIsDeleted = false;
    cartesian_trajectory_planner_DW.obj_m.isSetupComplete = false;
    cartesian_trajectory_planner_DW.obj_m.isInitialized = 1;
    for (cartesian_trajectory_planner__B.i_c = 0;
         cartesian_trajectory_planner__B.i_c < 10;
         cartesian_trajectory_planner__B.i_c++) {
      cartesian_trajectory_planner__B.cv3[cartesian_trajectory_planner__B.i_c] =
        tmp_g[cartesian_trajectory_planner__B.i_c];
    }

    cartesian_trajectory_planner__B.cv3[10] = '\x00';
    ParamGet_cartesian_trajectory_planner_2_300.initialize
      (cartesian_trajectory_planner__B.cv3);
    ParamGet_cartesian_trajectory_planner_2_300.initialize_error_codes(0, 1, 2,
      3);
    ParamGet_cartesian_trajectory_planner_2_300.set_initial_value(0.0);
    cartesian_trajectory_planner_DW.obj_m.isSetupComplete = true;

    // End of Start for MATLABSystem: '<S11>/Get Parameter1'

    // Start for MATLABSystem: '<S11>/Get Parameter2'
    cartesian_trajectory_planner_DW.obj_c.matlabCodegenIsDeleted = true;
    cartesian_trajectory_planner_DW.obj_c.isInitialized = 0;
    cartesian_trajectory_planner_DW.obj_c.matlabCodegenIsDeleted = false;
    cartesian_trajectory_planner_DW.obj_c.isSetupComplete = false;
    cartesian_trajectory_planner_DW.obj_c.isInitialized = 1;
    for (cartesian_trajectory_planner__B.i_c = 0;
         cartesian_trajectory_planner__B.i_c < 10;
         cartesian_trajectory_planner__B.i_c++) {
      cartesian_trajectory_planner__B.cv3[cartesian_trajectory_planner__B.i_c] =
        tmp_h[cartesian_trajectory_planner__B.i_c];
    }

    cartesian_trajectory_planner__B.cv3[10] = '\x00';
    ParamGet_cartesian_trajectory_planner_2_301.initialize
      (cartesian_trajectory_planner__B.cv3);
    ParamGet_cartesian_trajectory_planner_2_301.initialize_error_codes(0, 1, 2,
      3);
    ParamGet_cartesian_trajectory_planner_2_301.set_initial_value(0.0);
    cartesian_trajectory_planner_DW.obj_c.isSetupComplete = true;

    // End of Start for MATLABSystem: '<S11>/Get Parameter2'

    // Start for MATLABSystem: '<S11>/Get Parameter3'
    cartesian_trajectory_planner_DW.obj_f.matlabCodegenIsDeleted = true;
    cartesian_trajectory_planner_DW.obj_f.isInitialized = 0;
    cartesian_trajectory_planner_DW.obj_f.matlabCodegenIsDeleted = false;
    cartesian_trajectory_planner_DW.obj_f.isSetupComplete = false;
    cartesian_trajectory_planner_DW.obj_f.isInitialized = 1;
    for (cartesian_trajectory_planner__B.i_c = 0;
         cartesian_trajectory_planner__B.i_c < 10;
         cartesian_trajectory_planner__B.i_c++) {
      cartesian_trajectory_planner__B.cv3[cartesian_trajectory_planner__B.i_c] =
        tmp_i[cartesian_trajectory_planner__B.i_c];
    }

    cartesian_trajectory_planner__B.cv3[10] = '\x00';
    ParamGet_cartesian_trajectory_planner_2_302.initialize
      (cartesian_trajectory_planner__B.cv3);
    ParamGet_cartesian_trajectory_planner_2_302.initialize_error_codes(0, 1, 2,
      3);
    ParamGet_cartesian_trajectory_planner_2_302.set_initial_value(1.0);
    cartesian_trajectory_planner_DW.obj_f.isSetupComplete = true;

    // End of Start for MATLABSystem: '<S11>/Get Parameter3'

    // Start for MATLABSystem: '<S11>/Get Parameter4'
    cartesian_trajectory_planner_DW.obj_e.matlabCodegenIsDeleted = true;
    cartesian_trajectory_planner_DW.obj_e.isInitialized = 0;
    cartesian_trajectory_planner_DW.obj_e.matlabCodegenIsDeleted = false;
    cartesian_trajectory_planner_DW.obj_e.isSetupComplete = false;
    cartesian_trajectory_planner_DW.obj_e.isInitialized = 1;
    for (cartesian_trajectory_planner__B.i_c = 0;
         cartesian_trajectory_planner__B.i_c < 10;
         cartesian_trajectory_planner__B.i_c++) {
      cartesian_trajectory_planner__B.cv3[cartesian_trajectory_planner__B.i_c] =
        tmp_j[cartesian_trajectory_planner__B.i_c];
    }

    cartesian_trajectory_planner__B.cv3[10] = '\x00';
    ParamGet_cartesian_trajectory_planner_2_303.initialize
      (cartesian_trajectory_planner__B.cv3);
    ParamGet_cartesian_trajectory_planner_2_303.initialize_error_codes(0, 1, 2,
      3);
    ParamGet_cartesian_trajectory_planner_2_303.set_initial_value(1.0);
    cartesian_trajectory_planner_DW.obj_e.isSetupComplete = true;

    // End of Start for MATLABSystem: '<S11>/Get Parameter4'

    // Start for MATLABSystem: '<S11>/Get Parameter5'
    cartesian_trajectory_planner_DW.obj_a.matlabCodegenIsDeleted = true;
    cartesian_trajectory_planner_DW.obj_a.isInitialized = 0;
    cartesian_trajectory_planner_DW.obj_a.matlabCodegenIsDeleted = false;
    cartesian_trajectory_planner_DW.obj_a.isSetupComplete = false;
    cartesian_trajectory_planner_DW.obj_a.isInitialized = 1;
    for (cartesian_trajectory_planner__B.i_c = 0;
         cartesian_trajectory_planner__B.i_c < 10;
         cartesian_trajectory_planner__B.i_c++) {
      cartesian_trajectory_planner__B.cv3[cartesian_trajectory_planner__B.i_c] =
        tmp_k[cartesian_trajectory_planner__B.i_c];
    }

    cartesian_trajectory_planner__B.cv3[10] = '\x00';
    ParamGet_cartesian_trajectory_planner_2_304.initialize
      (cartesian_trajectory_planner__B.cv3);
    ParamGet_cartesian_trajectory_planner_2_304.initialize_error_codes(0, 1, 2,
      3);
    ParamGet_cartesian_trajectory_planner_2_304.set_initial_value(1.0);
    cartesian_trajectory_planner_DW.obj_a.isSetupComplete = true;

    // End of Start for MATLABSystem: '<S11>/Get Parameter5'
    cartesian_tra_MATLABSystem_Init
      (&cartesian_trajectory_planner__B.MATLABSystem_p,
       &cartesian_trajectory_planner_DW.MATLABSystem_p);
    cartesian_tra_MATLABSystem_Init
      (&cartesian_trajectory_planner__B.MATLABSystem,
       &cartesian_trajectory_planner_DW.MATLABSystem);

    // Start for MATLABSystem: '<S10>/Set Parameter'
    cartesian_trajectory_planner_DW.obj_gt.matlabCodegenIsDeleted = true;
    cartesian_trajectory_planner_DW.obj_gt.isInitialized = 0;
    cartesian_trajectory_planner_DW.obj_gt.matlabCodegenIsDeleted = false;
    cartesian_trajectory_planner_DW.obj_gt.isSetupComplete = false;
    cartesian_trajectory_planner_DW.obj_gt.isInitialized = 1;
    for (cartesian_trajectory_planner__B.i_c = 0;
         cartesian_trajectory_planner__B.i_c < 11;
         cartesian_trajectory_planner__B.i_c++) {
      cartesian_trajectory_planner__B.cv2[cartesian_trajectory_planner__B.i_c] =
        tmp_l[cartesian_trajectory_planner__B.i_c];
    }

    cartesian_trajectory_planner__B.cv2[11] = '\x00';
    ParamSet_cartesian_trajectory_planner_2_325.initialize
      (cartesian_trajectory_planner__B.cv2);
    cartesian_trajectory_planner_DW.obj_gt.isSetupComplete = true;

    // End of Start for MATLABSystem: '<S10>/Set Parameter'

    // Start for MATLABSystem: '<S10>/Set Parameter1'
    cartesian_trajectory_planner_DW.obj_kg.matlabCodegenIsDeleted = true;
    cartesian_trajectory_planner_DW.obj_kg.isInitialized = 0;
    cartesian_trajectory_planner_DW.obj_kg.matlabCodegenIsDeleted = false;
    cartesian_trajectory_planner_DW.obj_kg.isSetupComplete = false;
    cartesian_trajectory_planner_DW.obj_kg.isInitialized = 1;
    for (cartesian_trajectory_planner__B.i_c = 0;
         cartesian_trajectory_planner__B.i_c < 11;
         cartesian_trajectory_planner__B.i_c++) {
      cartesian_trajectory_planner__B.cv2[cartesian_trajectory_planner__B.i_c] =
        tmp_m[cartesian_trajectory_planner__B.i_c];
    }

    cartesian_trajectory_planner__B.cv2[11] = '\x00';
    ParamSet_cartesian_trajectory_planner_2_326.initialize
      (cartesian_trajectory_planner__B.cv2);
    cartesian_trajectory_planner_DW.obj_kg.isSetupComplete = true;

    // End of Start for MATLABSystem: '<S10>/Set Parameter1'

    // Start for MATLABSystem: '<S10>/Set Parameter2'
    cartesian_trajectory_planner_DW.obj_ef.matlabCodegenIsDeleted = true;
    cartesian_trajectory_planner_DW.obj_ef.isInitialized = 0;
    cartesian_trajectory_planner_DW.obj_ef.matlabCodegenIsDeleted = false;
    cartesian_trajectory_planner_DW.obj_ef.isSetupComplete = false;
    cartesian_trajectory_planner_DW.obj_ef.isInitialized = 1;
    for (cartesian_trajectory_planner__B.i_c = 0;
         cartesian_trajectory_planner__B.i_c < 11;
         cartesian_trajectory_planner__B.i_c++) {
      cartesian_trajectory_planner__B.cv2[cartesian_trajectory_planner__B.i_c] =
        tmp_n[cartesian_trajectory_planner__B.i_c];
    }

    cartesian_trajectory_planner__B.cv2[11] = '\x00';
    ParamSet_cartesian_trajectory_planner_2_327.initialize
      (cartesian_trajectory_planner__B.cv2);
    cartesian_trajectory_planner_DW.obj_ef.isSetupComplete = true;

    // End of Start for MATLABSystem: '<S10>/Set Parameter2'

    // Start for MATLABSystem: '<S10>/Set Parameter3'
    cartesian_trajectory_planner_DW.obj_kf.matlabCodegenIsDeleted = true;
    cartesian_trajectory_planner_DW.obj_kf.isInitialized = 0;
    cartesian_trajectory_planner_DW.obj_kf.matlabCodegenIsDeleted = false;
    cartesian_trajectory_planner_DW.obj_kf.isSetupComplete = false;
    cartesian_trajectory_planner_DW.obj_kf.isInitialized = 1;
    for (cartesian_trajectory_planner__B.i_c = 0;
         cartesian_trajectory_planner__B.i_c < 11;
         cartesian_trajectory_planner__B.i_c++) {
      cartesian_trajectory_planner__B.cv2[cartesian_trajectory_planner__B.i_c] =
        tmp_o[cartesian_trajectory_planner__B.i_c];
    }

    cartesian_trajectory_planner__B.cv2[11] = '\x00';
    ParamSet_cartesian_trajectory_planner_2_328.initialize
      (cartesian_trajectory_planner__B.cv2);
    cartesian_trajectory_planner_DW.obj_kf.isSetupComplete = true;

    // End of Start for MATLABSystem: '<S10>/Set Parameter3'

    // Start for MATLABSystem: '<S10>/Set Parameter4'
    cartesian_trajectory_planner_DW.obj_gc.matlabCodegenIsDeleted = true;
    cartesian_trajectory_planner_DW.obj_gc.isInitialized = 0;
    cartesian_trajectory_planner_DW.obj_gc.matlabCodegenIsDeleted = false;
    cartesian_trajectory_planner_DW.obj_gc.isSetupComplete = false;
    cartesian_trajectory_planner_DW.obj_gc.isInitialized = 1;
    for (cartesian_trajectory_planner__B.i_c = 0;
         cartesian_trajectory_planner__B.i_c < 11;
         cartesian_trajectory_planner__B.i_c++) {
      cartesian_trajectory_planner__B.cv2[cartesian_trajectory_planner__B.i_c] =
        tmp_p[cartesian_trajectory_planner__B.i_c];
    }

    cartesian_trajectory_planner__B.cv2[11] = '\x00';
    ParamSet_cartesian_trajectory_planner_2_329.initialize
      (cartesian_trajectory_planner__B.cv2);
    cartesian_trajectory_planner_DW.obj_gc.isSetupComplete = true;

    // End of Start for MATLABSystem: '<S10>/Set Parameter4'

    // Start for MATLABSystem: '<S10>/Set Parameter5'
    cartesian_trajectory_planner_DW.obj_oe.matlabCodegenIsDeleted = true;
    cartesian_trajectory_planner_DW.obj_oe.isInitialized = 0;
    cartesian_trajectory_planner_DW.obj_oe.matlabCodegenIsDeleted = false;
    cartesian_trajectory_planner_DW.obj_oe.isSetupComplete = false;
    cartesian_trajectory_planner_DW.obj_oe.isInitialized = 1;
    for (cartesian_trajectory_planner__B.i_c = 0;
         cartesian_trajectory_planner__B.i_c < 11;
         cartesian_trajectory_planner__B.i_c++) {
      cartesian_trajectory_planner__B.cv2[cartesian_trajectory_planner__B.i_c] =
        tmp_q[cartesian_trajectory_planner__B.i_c];
    }

    cartesian_trajectory_planner__B.cv2[11] = '\x00';
    ParamSet_cartesian_trajectory_planner_2_330.initialize
      (cartesian_trajectory_planner__B.cv2);
    cartesian_trajectory_planner_DW.obj_oe.isSetupComplete = true;

    // End of Start for MATLABSystem: '<S10>/Set Parameter5'

    // Start for MATLABSystem: '<S9>/Get Parameter'
    cartesian_trajectory_planner_DW.obj_iy.matlabCodegenIsDeleted = true;
    cartesian_trajectory_planner_DW.obj_iy.isInitialized = 0;
    cartesian_trajectory_planner_DW.obj_iy.matlabCodegenIsDeleted = false;
    cartesian_trajectory_planner_DW.obj_iy.isSetupComplete = false;
    cartesian_trajectory_planner_DW.obj_iy.isInitialized = 1;
    for (cartesian_trajectory_planner__B.i_c = 0;
         cartesian_trajectory_planner__B.i_c < 12;
         cartesian_trajectory_planner__B.i_c++) {
      cartesian_trajectory_planner__B.cv1[cartesian_trajectory_planner__B.i_c] =
        tmp_r[cartesian_trajectory_planner__B.i_c];
    }

    cartesian_trajectory_planner__B.cv1[12] = '\x00';
    ParamGet_cartesian_trajectory_planner_2_270.initialize
      (cartesian_trajectory_planner__B.cv1);
    ParamGet_cartesian_trajectory_planner_2_270.initialize_error_codes(0, 1, 2,
      3);
    ParamGet_cartesian_trajectory_planner_2_270.set_initial_value(5.0);
    cartesian_trajectory_planner_DW.obj_iy.isSetupComplete = true;

    // End of Start for MATLABSystem: '<S9>/Get Parameter'

    // Start for MATLABSystem: '<S9>/Get Parameter2'
    cartesian_trajectory_planner_DW.obj_cu.matlabCodegenIsDeleted = true;
    cartesian_trajectory_planner_DW.obj_cu.isInitialized = 0;
    cartesian_trajectory_planner_DW.obj_cu.matlabCodegenIsDeleted = false;
    cartesian_trajectory_planner_DW.obj_cu.isSetupComplete = false;
    cartesian_trajectory_planner_DW.obj_cu.isInitialized = 1;
    for (cartesian_trajectory_planner__B.i_c = 0;
         cartesian_trajectory_planner__B.i_c < 12;
         cartesian_trajectory_planner__B.i_c++) {
      cartesian_trajectory_planner__B.cv1[cartesian_trajectory_planner__B.i_c] =
        tmp_s[cartesian_trajectory_planner__B.i_c];
    }

    cartesian_trajectory_planner__B.cv1[12] = '\x00';
    ParamGet_cartesian_trajectory_planner_2_271.initialize
      (cartesian_trajectory_planner__B.cv1);
    ParamGet_cartesian_trajectory_planner_2_271.initialize_error_codes(0, 1, 2,
      3);
    ParamGet_cartesian_trajectory_planner_2_271.set_initial_value(1.0);
    cartesian_trajectory_planner_DW.obj_cu.isSetupComplete = true;

    // End of Start for MATLABSystem: '<S9>/Get Parameter2'

    // Start for MATLABSystem: '<S9>/Get Parameter3'
    cartesian_trajectory_planner_DW.obj_k.matlabCodegenIsDeleted = true;
    cartesian_trajectory_planner_DW.obj_k.isInitialized = 0;
    cartesian_trajectory_planner_DW.obj_k.matlabCodegenIsDeleted = false;
    cartesian_trajectory_planner_DW.obj_k.isSetupComplete = false;
    cartesian_trajectory_planner_DW.obj_k.isInitialized = 1;
    for (cartesian_trajectory_planner__B.i_c = 0;
         cartesian_trajectory_planner__B.i_c < 12;
         cartesian_trajectory_planner__B.i_c++) {
      cartesian_trajectory_planner__B.cv1[cartesian_trajectory_planner__B.i_c] =
        tmp_t[cartesian_trajectory_planner__B.i_c];
    }

    cartesian_trajectory_planner__B.cv1[12] = '\x00';
    ParamGet_cartesian_trajectory_planner_2_272.initialize
      (cartesian_trajectory_planner__B.cv1);
    ParamGet_cartesian_trajectory_planner_2_272.initialize_error_codes(0, 1, 2,
      3);
    ParamGet_cartesian_trajectory_planner_2_272.set_initial_value(1.0);
    cartesian_trajectory_planner_DW.obj_k.isSetupComplete = true;

    // End of Start for MATLABSystem: '<S9>/Get Parameter3'
  }
}

// Model terminate function
void cartesian_trajectory_planner_2_terminate(void)
{
  // Terminate for MATLABSystem: '<S13>/Get Parameter3'
  matlabCodegenHandle_matlabC_jbz(&cartesian_trajectory_planner_DW.obj_e0);

  // Terminate for MATLABSystem: '<S13>/Get Parameter4'
  matlabCodegenHandle_matlabC_jbz(&cartesian_trajectory_planner_DW.obj_p);

  // Terminate for MATLABSystem: '<S13>/Get Parameter5'
  matlabCodegenHandle_matlabC_jbz(&cartesian_trajectory_planner_DW.obj_g);

  // Terminate for MATLABSystem: '<S13>/Get Parameter6'
  matlabCodegenHandle_matlabC_jbz(&cartesian_trajectory_planner_DW.obj_or);

  // Terminate for MATLABSystem: '<S13>/Get Parameter'
  matlabCodegenHandle_matlabC_jbz(&cartesian_trajectory_planner_DW.obj_a3);

  // Terminate for MATLABSystem: '<S13>/Get Parameter1'
  matlabCodegenHandle_matlabC_jbz(&cartesian_trajectory_planner_DW.obj_bh);

  // Terminate for MATLABSystem: '<S13>/Get Parameter2'
  matlabCodegenHandle_matlabC_jbz(&cartesian_trajectory_planner_DW.obj_b0);

  // Terminate for MATLABSystem: '<S14>/Get Parameter3'
  matlabCodegenHandle_matlabC_jbz(&cartesian_trajectory_planner_DW.obj_o);

  // Terminate for MATLABSystem: '<S14>/Get Parameter4'
  matlabCodegenHandle_matlabC_jbz(&cartesian_trajectory_planner_DW.obj_es);

  // Terminate for MATLABSystem: '<S14>/Get Parameter5'
  matlabCodegenHandle_matlabC_jbz(&cartesian_trajectory_planner_DW.obj_f4);

  // Terminate for MATLABSystem: '<S14>/Get Parameter6'
  matlabCodegenHandle_matlabC_jbz(&cartesian_trajectory_planner_DW.obj_en);

  // Terminate for MATLABSystem: '<S14>/Get Parameter'
  matlabCodegenHandle_matlabC_jbz(&cartesian_trajectory_planner_DW.obj_b);

  // Terminate for MATLABSystem: '<S14>/Get Parameter1'
  matlabCodegenHandle_matlabC_jbz(&cartesian_trajectory_planner_DW.obj_i);

  // Terminate for MATLABSystem: '<S14>/Get Parameter2'
  matlabCodegenHandle_matlabC_jbz(&cartesian_trajectory_planner_DW.obj_d);

  // Terminate for MATLABSystem: '<S11>/Get Parameter'
  matlabCodegenHandle_matlabC_jbz(&cartesian_trajectory_planner_DW.obj);

  // Terminate for MATLABSystem: '<S11>/Get Parameter1'
  matlabCodegenHandle_matlabC_jbz(&cartesian_trajectory_planner_DW.obj_m);

  // Terminate for MATLABSystem: '<S11>/Get Parameter2'
  matlabCodegenHandle_matlabC_jbz(&cartesian_trajectory_planner_DW.obj_c);

  // Terminate for MATLABSystem: '<S11>/Get Parameter3'
  matlabCodegenHandle_matlabC_jbz(&cartesian_trajectory_planner_DW.obj_f);

  // Terminate for MATLABSystem: '<S11>/Get Parameter4'
  matlabCodegenHandle_matlabC_jbz(&cartesian_trajectory_planner_DW.obj_e);

  // Terminate for MATLABSystem: '<S11>/Get Parameter5'
  matlabCodegenHandle_matlabC_jbz(&cartesian_trajectory_planner_DW.obj_a);
  cartesian_tra_MATLABSystem_Term
    (&cartesian_trajectory_planner_DW.MATLABSystem_p);
  cartesian_tra_MATLABSystem_Term(&cartesian_trajectory_planner_DW.MATLABSystem);

  // Terminate for MATLABSystem: '<S10>/Set Parameter'
  cartesian_tr_matlabCodegenHa_ku(&cartesian_trajectory_planner_DW.obj_gt);

  // Terminate for MATLABSystem: '<S10>/Set Parameter1'
  cartesian_tr_matlabCodegenHa_ku(&cartesian_trajectory_planner_DW.obj_kg);

  // Terminate for MATLABSystem: '<S10>/Set Parameter2'
  cartesian_tr_matlabCodegenHa_ku(&cartesian_trajectory_planner_DW.obj_ef);

  // Terminate for MATLABSystem: '<S10>/Set Parameter3'
  cartesian_tr_matlabCodegenHa_ku(&cartesian_trajectory_planner_DW.obj_kf);

  // Terminate for MATLABSystem: '<S10>/Set Parameter4'
  cartesian_tr_matlabCodegenHa_ku(&cartesian_trajectory_planner_DW.obj_gc);

  // Terminate for MATLABSystem: '<S10>/Set Parameter5'
  cartesian_tr_matlabCodegenHa_ku(&cartesian_trajectory_planner_DW.obj_oe);

  // Terminate for Atomic SubSystem: '<Root>/Subscribe'
  // Terminate for MATLABSystem: '<S8>/SourceBlock'
  matlabCodegenHandle_matlabCo_jb(&cartesian_trajectory_planner_DW.obj_cs);

  // End of Terminate for SubSystem: '<Root>/Subscribe'

  // Terminate for MATLABSystem: '<S9>/Get Parameter'
  matlabCodegenHandle_matlabC_jbz(&cartesian_trajectory_planner_DW.obj_iy);

  // Terminate for MATLABSystem: '<S9>/Get Parameter2'
  matlabCodegenHandle_matlabC_jbz(&cartesian_trajectory_planner_DW.obj_cu);

  // Terminate for MATLABSystem: '<S9>/Get Parameter3'
  matlabCodegenHandle_matlabC_jbz(&cartesian_trajectory_planner_DW.obj_k);

  // Terminate for Atomic SubSystem: '<Root>/Publish'
  // Terminate for MATLABSystem: '<S7>/SinkBlock'
  matlabCodegenHandle_matlabCod_j(&cartesian_trajectory_planner_DW.obj_ej);

  // End of Terminate for SubSystem: '<Root>/Publish'
}

//
// File trailer for generated code.
//
// [EOF]
//
