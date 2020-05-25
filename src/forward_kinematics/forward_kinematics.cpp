//
// Academic License - for use in teaching, academic research, and meeting
// course requirements at degree granting institutions only.  Not for
// government, commercial, or other organizational use.
//
// File: forward_kinematics.cpp
//
// Code generated for Simulink model 'forward_kinematics'.
//
// Model version                  : 1.129
// Simulink Coder version         : 9.3 (R2020a) 18-Nov-2019
// C/C++ source code generated on : Mon May 25 15:27:21 2020
//
// Target selection: ert.tlc
// Embedded hardware selection: Generic->Unspecified (assume 32-bit Generic)
// Code generation objectives: Unspecified
// Validation result: Not run
//
#include "forward_kinematics.h"
#include "forward_kinematics_private.h"

// Block signals (default storage)
B_forward_kinematics_T forward_kinematics_B;

// Block states (default storage)
DW_forward_kinematics_T forward_kinematics_DW;

// Real-time model
RT_MODEL_forward_kinematics_T forward_kinematics_M_ =
  RT_MODEL_forward_kinematics_T();
RT_MODEL_forward_kinematics_T *const forward_kinematics_M =
  &forward_kinematics_M_;

// Forward declaration for local functions
static void forward_kinemat_SystemCore_step(boolean_T *varargout_1,
  SL_Bus_forward_kinematics_std_msgs_String varargout_2_Name[16], uint32_T
  *varargout_2_Name_SL_Info_Curren, uint32_T *varargout_2_Name_SL_Info_Receiv,
  real_T varargout_2_Position[128], uint32_T *varargout_2_Position_SL_Info_Cu,
  uint32_T *varargout_2_Position_SL_Info_Re, real_T varargout_2_Velocity[128],
  uint32_T *varargout_2_Velocity_SL_Info_Cu, uint32_T
  *varargout_2_Velocity_SL_Info_Re, real_T varargout_2_Effort[128], uint32_T
  *varargout_2_Effort_SL_Info_Curr, uint32_T *varargout_2_Effort_SL_Info_Rece,
  SL_Bus_forward_kinematics_std_msgs_Header *varargout_2_Header);
static void forward_kinemati_emxInit_real_T(emxArray_real_T_forward_kinem_T
  **pEmxArray, int32_T numDimensions);
static void forward_kin_emxInit_f_cell_wrap(emxArray_f_cell_wrap_forward__T
  **pEmxArray, int32_T numDimensions);
static void forward_kinemati_emxInit_char_T(emxArray_char_T_forward_kinem_T
  **pEmxArray, int32_T numDimensions);
static void f_emxEnsureCapacity_f_cell_wrap(emxArray_f_cell_wrap_forward__T
  *emxArray, int32_T oldNumel);
static void forwar_emxEnsureCapacity_char_T(emxArray_char_T_forward_kinem_T
  *emxArray, int32_T oldNumel);
static void fo_rigidBodyJoint_get_JointAxis(const
  c_rigidBodyJoint_forward_kine_T *obj, real_T ax[3]);
static void forward_kinemati_emxFree_char_T(emxArray_char_T_forward_kinem_T
  **pEmxArray);
static void RigidBodyTree_forwardKinematics(p_robotics_manip_internal_Rig_T *obj,
  const real_T qvec[6], emxArray_f_cell_wrap_forward__T *Ttree);
static void forwar_emxEnsureCapacity_real_T(emxArray_real_T_forward_kinem_T
  *emxArray, int32_T oldNumel);
static void forward_kinemati_emxFree_real_T(emxArray_real_T_forward_kinem_T
  **pEmxArray);
static void forward_kin_emxFree_f_cell_wrap(emxArray_f_cell_wrap_forward__T
  **pEmxArray);
static void RigidBodyTree_geometricJacobian(p_robotics_manip_internal_Rig_T *obj,
  const real_T Q[6], emxArray_real_T_forward_kinem_T *Jac);
static void rigidBodyJoint_get_JointAxis_e(const c_rigidBodyJoint_forward_ki_e_T
  *obj, real_T ax[3]);
static void RigidBodyTree_forwardKinemati_e(p_robotics_manip_internal_R_e_T *obj,
  const real_T qvec[6], emxArray_f_cell_wrap_forward__T *Ttree);
static boolean_T forward_kinematics_anyNonFinite(const real_T x[16]);
static real_T forward_kinematic_rt_hypotd_snf(real_T u0, real_T u1);
static real_T forward_kinematics_xzlangeM(const creal_T x[16]);
static void forward_kinematics_xzlascl(real_T cfrom, real_T cto, creal_T A[16]);
static real_T forward_kinematics_xzlanhs(const creal_T A[16], int32_T ilo,
  int32_T ihi);
static void forward_kinematics_xzlartg_k(const creal_T f, const creal_T g,
  real_T *cs, creal_T *sn);
static void forward_kinematics_xzlartg(const creal_T f, const creal_T g, real_T *
  cs, creal_T *sn, creal_T *r);
static void forward_kinematics_xzhgeqz(creal_T A[16], int32_T ilo, int32_T ihi,
  creal_T Z[16], int32_T *info, creal_T alpha1[4], creal_T beta1[4]);
static void forward_kinematics_xztgevc(const creal_T A[16], creal_T V[16]);
static void forward_kinematics_xzggev(creal_T A[16], int32_T *info, creal_T
  alpha1[4], creal_T beta1[4], creal_T V[16]);
static real_T forward_kinematics_xnrm2(int32_T n, const real_T x[16], int32_T
  ix0);
static void forward_kinematics_xzlarf(int32_T m, int32_T n, int32_T iv0, real_T
  tau, real_T C[16], int32_T ic0, real_T work[4]);
static void forward_kinematics_xgehrd(real_T a[16], real_T tau[3]);
static real_T forward_kinematics_xnrm2_k(int32_T n, const real_T x[3]);
static real_T forward_kinematics_xzlarfg(int32_T n, real_T *alpha1, real_T x[3]);
static void forward_kinematics_xdlanv2(real_T *a, real_T *b, real_T *c, real_T
  *d, real_T *rt1r, real_T *rt1i, real_T *rt2r, real_T *rt2i, real_T *cs, real_T
  *sn);
static void forward_kinematics_xrot(int32_T n, real_T x[16], int32_T ix0,
  int32_T iy0, real_T c, real_T s);
static void forward_kinematics_xrot_m(int32_T n, real_T x[16], int32_T ix0,
  int32_T iy0, real_T c, real_T s);
static int32_T forward_kinematics_eml_dlahqr(real_T h[16], real_T z[16]);
static void forward_kinematics_eig(const real_T A[16], creal_T V[16], creal_T D
  [4]);
static void matlabCodegenHandle_matlabCo_ef(ros_slros_internal_block_Subs_T *obj);
static void emxFreeStruct_c_rigidBodyJoint(c_rigidBodyJoint_forward_kine_T
  *pStruct);
static void emxFreeStruct_o_robotics_manip_(o_robotics_manip_internal_Rig_T
  *pStruct);
static void emxFreeStruct_p_robotics_manip_(p_robotics_manip_internal_Rig_T
  *pStruct);
static void emxFreeStruct_robotics_slmanip_(robotics_slmanip_internal_blo_T
  *pStruct);
static void emxFreeStruct_n_robotics_manip_(n_robotics_manip_internal_Rig_T
  *pStruct);
static void emxFreeStruct_c_rigidBodyJoint1(c_rigidBodyJoint_forward_ki_e_T
  *pStruct);
static void emxFreeStruct_o_robotics_mani_e(o_robotics_manip_internal_R_e_T
  *pStruct);
static void emxFreeStruct_p_robotics_mani_e(p_robotics_manip_internal_R_e_T
  *pStruct);
static void emxFreeStruct_robotics_slmani_e(robotics_slmanip_internal_b_e_T
  *pStruct);
static void emxFreeStruct_n_robotics_mani_e(n_robotics_manip_internal_R_e_T
  *pStruct);
static void matlabCodegenHandle_matlabCodeg(ros_slros_internal_block_Publ_T *obj);
static void emxInitStruct_c_rigidBodyJoint(c_rigidBodyJoint_forward_kine_T
  *pStruct);
static void emxInitStruct_o_robotics_manip_(o_robotics_manip_internal_Rig_T
  *pStruct);
static void emxInitStruct_p_robotics_manip_(p_robotics_manip_internal_Rig_T
  *pStruct);
static void emxInitStruct_robotics_slmanip_(robotics_slmanip_internal_blo_T
  *pStruct);
static void emxInitStruct_n_robotics_manip_(n_robotics_manip_internal_Rig_T
  *pStruct);
static n_robotics_manip_internal_Rig_T *forward_kin_RigidBody_RigidBody
  (n_robotics_manip_internal_Rig_T *obj);
static n_robotics_manip_internal_Rig_T *forward_k_RigidBody_RigidBody_e
  (n_robotics_manip_internal_Rig_T *obj);
static n_robotics_manip_internal_Rig_T *forward__RigidBody_RigidBody_ef
  (n_robotics_manip_internal_Rig_T *obj);
static n_robotics_manip_internal_Rig_T *forward_RigidBody_RigidBody_efg
  (n_robotics_manip_internal_Rig_T *obj);
static n_robotics_manip_internal_Rig_T *forwar_RigidBody_RigidBody_efgd
  (n_robotics_manip_internal_Rig_T *obj);
static n_robotics_manip_internal_Rig_T *forwa_RigidBody_RigidBody_efgdb
  (n_robotics_manip_internal_Rig_T *obj);
static n_robotics_manip_internal_Rig_T *forw_RigidBody_RigidBody_efgdbo
  (n_robotics_manip_internal_Rig_T *obj);
static n_robotics_manip_internal_Rig_T *for_RigidBody_RigidBody_efgdboc
  (n_robotics_manip_internal_Rig_T *obj);
static o_robotics_manip_internal_Rig_T *fo_RigidBody_RigidBody_efgdboc5
  (o_robotics_manip_internal_Rig_T *obj);
static p_robotics_manip_internal_Rig_T *for_RigidBodyTree_RigidBodyTree
  (p_robotics_manip_internal_Rig_T *obj, n_robotics_manip_internal_Rig_T *iobj_0,
   n_robotics_manip_internal_Rig_T *iobj_1, n_robotics_manip_internal_Rig_T
   *iobj_2, n_robotics_manip_internal_Rig_T *iobj_3,
   n_robotics_manip_internal_Rig_T *iobj_4, n_robotics_manip_internal_Rig_T
   *iobj_5, n_robotics_manip_internal_Rig_T *iobj_6,
   n_robotics_manip_internal_Rig_T *iobj_7);
static void emxInitStruct_c_rigidBodyJoint1(c_rigidBodyJoint_forward_ki_e_T
  *pStruct);
static void emxInitStruct_o_robotics_mani_e(o_robotics_manip_internal_R_e_T
  *pStruct);
static void emxInitStruct_p_robotics_mani_e(p_robotics_manip_internal_R_e_T
  *pStruct);
static void emxInitStruct_robotics_slmani_e(robotics_slmanip_internal_b_e_T
  *pStruct);
static void emxInitStruct_n_robotics_mani_e(n_robotics_manip_internal_R_e_T
  *pStruct);
static n_robotics_manip_internal_R_e_T *f_RigidBody_RigidBody_efgdboc5e
  (n_robotics_manip_internal_R_e_T *obj);
static n_robotics_manip_internal_R_e_T *RigidBody_RigidBody_efgdboc5ed
  (n_robotics_manip_internal_R_e_T *obj);
static n_robotics_manip_internal_R_e_T *RigidBody_RigidBody_efgdboc5edi
  (n_robotics_manip_internal_R_e_T *obj);
static n_robotics_manip_internal_R_e_T *RigidBody_RigidBod_efgdboc5edi3
  (n_robotics_manip_internal_R_e_T *obj);
static n_robotics_manip_internal_R_e_T *RigidBody_RigidBo_efgdboc5edi3c
  (n_robotics_manip_internal_R_e_T *obj);
static p_robotics_manip_internal_R_e_T *f_RigidBodyTree_RigidBodyTree_e
  (p_robotics_manip_internal_R_e_T *obj, n_robotics_manip_internal_R_e_T *iobj_0,
   n_robotics_manip_internal_R_e_T *iobj_1, n_robotics_manip_internal_R_e_T
   *iobj_2, n_robotics_manip_internal_R_e_T *iobj_3,
   n_robotics_manip_internal_R_e_T *iobj_4, n_robotics_manip_internal_R_e_T
   *iobj_5, n_robotics_manip_internal_R_e_T *iobj_6,
   n_robotics_manip_internal_R_e_T *iobj_7);
int32_T div_nzp_s32(int32_T numerator, int32_T denominator)
{
  uint32_T tempAbsQuotient;
  tempAbsQuotient = (numerator < 0 ? ~static_cast<uint32_T>(numerator) + 1U :
                     static_cast<uint32_T>(numerator)) / (denominator < 0 ? ~
    static_cast<uint32_T>(denominator) + 1U : static_cast<uint32_T>(denominator));
  return (numerator < 0) != (denominator < 0) ? -static_cast<int32_T>
    (tempAbsQuotient) : static_cast<int32_T>(tempAbsQuotient);
}

static void forward_kinemat_SystemCore_step(boolean_T *varargout_1,
  SL_Bus_forward_kinematics_std_msgs_String varargout_2_Name[16], uint32_T
  *varargout_2_Name_SL_Info_Curren, uint32_T *varargout_2_Name_SL_Info_Receiv,
  real_T varargout_2_Position[128], uint32_T *varargout_2_Position_SL_Info_Cu,
  uint32_T *varargout_2_Position_SL_Info_Re, real_T varargout_2_Velocity[128],
  uint32_T *varargout_2_Velocity_SL_Info_Cu, uint32_T
  *varargout_2_Velocity_SL_Info_Re, real_T varargout_2_Effort[128], uint32_T
  *varargout_2_Effort_SL_Info_Curr, uint32_T *varargout_2_Effort_SL_Info_Rece,
  SL_Bus_forward_kinematics_std_msgs_Header *varargout_2_Header)
{
  *varargout_1 = Sub_forward_kinematics_286.getLatestMessage
    (&forward_kinematics_B.b_varargout_2);
  memcpy(&varargout_2_Name[0], &forward_kinematics_B.b_varargout_2.Name[0],
         sizeof(SL_Bus_forward_kinematics_std_msgs_String) << 4U);
  *varargout_2_Name_SL_Info_Curren =
    forward_kinematics_B.b_varargout_2.Name_SL_Info.CurrentLength;
  *varargout_2_Name_SL_Info_Receiv =
    forward_kinematics_B.b_varargout_2.Name_SL_Info.ReceivedLength;
  *varargout_2_Position_SL_Info_Cu =
    forward_kinematics_B.b_varargout_2.Position_SL_Info.CurrentLength;
  *varargout_2_Position_SL_Info_Re =
    forward_kinematics_B.b_varargout_2.Position_SL_Info.ReceivedLength;
  *varargout_2_Velocity_SL_Info_Cu =
    forward_kinematics_B.b_varargout_2.Velocity_SL_Info.CurrentLength;
  *varargout_2_Velocity_SL_Info_Re =
    forward_kinematics_B.b_varargout_2.Velocity_SL_Info.ReceivedLength;
  memcpy(&varargout_2_Position[0], &forward_kinematics_B.b_varargout_2.Position
         [0], sizeof(real_T) << 7U);
  memcpy(&varargout_2_Velocity[0], &forward_kinematics_B.b_varargout_2.Velocity
         [0], sizeof(real_T) << 7U);
  memcpy(&varargout_2_Effort[0], &forward_kinematics_B.b_varargout_2.Effort[0],
         sizeof(real_T) << 7U);
  *varargout_2_Effort_SL_Info_Curr =
    forward_kinematics_B.b_varargout_2.Effort_SL_Info.CurrentLength;
  *varargout_2_Effort_SL_Info_Rece =
    forward_kinematics_B.b_varargout_2.Effort_SL_Info.ReceivedLength;
  *varargout_2_Header = forward_kinematics_B.b_varargout_2.Header;
}

static void forward_kinemati_emxInit_real_T(emxArray_real_T_forward_kinem_T
  **pEmxArray, int32_T numDimensions)
{
  emxArray_real_T_forward_kinem_T *emxArray;
  int32_T i;
  *pEmxArray = (emxArray_real_T_forward_kinem_T *)malloc(sizeof
    (emxArray_real_T_forward_kinem_T));
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

static void forward_kin_emxInit_f_cell_wrap(emxArray_f_cell_wrap_forward__T
  **pEmxArray, int32_T numDimensions)
{
  emxArray_f_cell_wrap_forward__T *emxArray;
  int32_T i;
  *pEmxArray = (emxArray_f_cell_wrap_forward__T *)malloc(sizeof
    (emxArray_f_cell_wrap_forward__T));
  emxArray = *pEmxArray;
  emxArray->data = (f_cell_wrap_forward_kinematic_T *)NULL;
  emxArray->numDimensions = numDimensions;
  emxArray->size = (int32_T *)malloc(sizeof(int32_T) * numDimensions);
  emxArray->allocatedSize = 0;
  emxArray->canFreeData = true;
  for (i = 0; i < numDimensions; i++) {
    emxArray->size[i] = 0;
  }
}

static void forward_kinemati_emxInit_char_T(emxArray_char_T_forward_kinem_T
  **pEmxArray, int32_T numDimensions)
{
  emxArray_char_T_forward_kinem_T *emxArray;
  *pEmxArray = (emxArray_char_T_forward_kinem_T *)malloc(sizeof
    (emxArray_char_T_forward_kinem_T));
  emxArray = *pEmxArray;
  emxArray->data = (char_T *)NULL;
  emxArray->numDimensions = numDimensions;
  emxArray->size = (int32_T *)malloc(sizeof(int32_T) * numDimensions);
  emxArray->allocatedSize = 0;
  emxArray->canFreeData = true;
  for (forward_kinematics_B.i_c = 0; forward_kinematics_B.i_c < numDimensions;
       forward_kinematics_B.i_c++) {
    emxArray->size[forward_kinematics_B.i_c] = 0;
  }
}

static void f_emxEnsureCapacity_f_cell_wrap(emxArray_f_cell_wrap_forward__T
  *emxArray, int32_T oldNumel)
{
  void *newData;
  if (oldNumel < 0) {
    oldNumel = 0;
  }

  forward_kinematics_B.newNumel_h = 1;
  for (forward_kinematics_B.i_e = 0; forward_kinematics_B.i_e <
       emxArray->numDimensions; forward_kinematics_B.i_e++) {
    forward_kinematics_B.newNumel_h *= emxArray->size[forward_kinematics_B.i_e];
  }

  if (forward_kinematics_B.newNumel_h > emxArray->allocatedSize) {
    forward_kinematics_B.i_e = emxArray->allocatedSize;
    if (forward_kinematics_B.i_e < 16) {
      forward_kinematics_B.i_e = 16;
    }

    while (forward_kinematics_B.i_e < forward_kinematics_B.newNumel_h) {
      if (forward_kinematics_B.i_e > 1073741823) {
        forward_kinematics_B.i_e = MAX_int32_T;
      } else {
        forward_kinematics_B.i_e <<= 1;
      }
    }

    newData = calloc(static_cast<uint32_T>(forward_kinematics_B.i_e), sizeof
                     (f_cell_wrap_forward_kinematic_T));
    if (emxArray->data != NULL) {
      memcpy(newData, emxArray->data, sizeof(f_cell_wrap_forward_kinematic_T)
             * oldNumel);
      if (emxArray->canFreeData) {
        free(emxArray->data);
      }
    }

    emxArray->data = (f_cell_wrap_forward_kinematic_T *)newData;
    emxArray->allocatedSize = forward_kinematics_B.i_e;
    emxArray->canFreeData = true;
  }
}

static void forwar_emxEnsureCapacity_char_T(emxArray_char_T_forward_kinem_T
  *emxArray, int32_T oldNumel)
{
  void *newData;
  if (oldNumel < 0) {
    oldNumel = 0;
  }

  forward_kinematics_B.newNumel = 1;
  for (forward_kinematics_B.i_f = 0; forward_kinematics_B.i_f <
       emxArray->numDimensions; forward_kinematics_B.i_f++) {
    forward_kinematics_B.newNumel *= emxArray->size[forward_kinematics_B.i_f];
  }

  if (forward_kinematics_B.newNumel > emxArray->allocatedSize) {
    forward_kinematics_B.i_f = emxArray->allocatedSize;
    if (forward_kinematics_B.i_f < 16) {
      forward_kinematics_B.i_f = 16;
    }

    while (forward_kinematics_B.i_f < forward_kinematics_B.newNumel) {
      if (forward_kinematics_B.i_f > 1073741823) {
        forward_kinematics_B.i_f = MAX_int32_T;
      } else {
        forward_kinematics_B.i_f <<= 1;
      }
    }

    newData = calloc(static_cast<uint32_T>(forward_kinematics_B.i_f), sizeof
                     (char_T));
    if (emxArray->data != NULL) {
      memcpy(newData, emxArray->data, sizeof(char_T) * oldNumel);
      if (emxArray->canFreeData) {
        free(emxArray->data);
      }
    }

    emxArray->data = (char_T *)newData;
    emxArray->allocatedSize = forward_kinematics_B.i_f;
    emxArray->canFreeData = true;
  }
}

static void fo_rigidBodyJoint_get_JointAxis(const
  c_rigidBodyJoint_forward_kine_T *obj, real_T ax[3])
{
  static const char_T tmp[8] = { 'r', 'e', 'v', 'o', 'l', 'u', 't', 'e' };

  static const char_T tmp_0[9] = { 'p', 'r', 'i', 's', 'm', 'a', 't', 'i', 'c' };

  boolean_T guard1 = false;
  int32_T exitg1;
  for (forward_kinematics_B.b_kstr_e = 0; forward_kinematics_B.b_kstr_e < 8;
       forward_kinematics_B.b_kstr_e++) {
    forward_kinematics_B.b_d[forward_kinematics_B.b_kstr_e] =
      tmp[forward_kinematics_B.b_kstr_e];
  }

  forward_kinematics_B.b_bool_pt = false;
  if (obj->Type->size[1] == 8) {
    forward_kinematics_B.b_kstr_e = 1;
    do {
      exitg1 = 0;
      if (forward_kinematics_B.b_kstr_e - 1 < 8) {
        forward_kinematics_B.kstr_p = forward_kinematics_B.b_kstr_e - 1;
        if (obj->Type->data[forward_kinematics_B.kstr_p] !=
            forward_kinematics_B.b_d[forward_kinematics_B.kstr_p]) {
          exitg1 = 1;
        } else {
          forward_kinematics_B.b_kstr_e++;
        }
      } else {
        forward_kinematics_B.b_bool_pt = true;
        exitg1 = 1;
      }
    } while (exitg1 == 0);
  }

  guard1 = false;
  if (forward_kinematics_B.b_bool_pt) {
    guard1 = true;
  } else {
    for (forward_kinematics_B.b_kstr_e = 0; forward_kinematics_B.b_kstr_e < 9;
         forward_kinematics_B.b_kstr_e++) {
      forward_kinematics_B.b_bs[forward_kinematics_B.b_kstr_e] =
        tmp_0[forward_kinematics_B.b_kstr_e];
    }

    forward_kinematics_B.b_bool_pt = false;
    if (obj->Type->size[1] == 9) {
      forward_kinematics_B.b_kstr_e = 1;
      do {
        exitg1 = 0;
        if (forward_kinematics_B.b_kstr_e - 1 < 9) {
          forward_kinematics_B.kstr_p = forward_kinematics_B.b_kstr_e - 1;
          if (obj->Type->data[forward_kinematics_B.kstr_p] !=
              forward_kinematics_B.b_bs[forward_kinematics_B.kstr_p]) {
            exitg1 = 1;
          } else {
            forward_kinematics_B.b_kstr_e++;
          }
        } else {
          forward_kinematics_B.b_bool_pt = true;
          exitg1 = 1;
        }
      } while (exitg1 == 0);
    }

    if (forward_kinematics_B.b_bool_pt) {
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

static void forward_kinemati_emxFree_char_T(emxArray_char_T_forward_kinem_T
  **pEmxArray)
{
  if (*pEmxArray != (emxArray_char_T_forward_kinem_T *)NULL) {
    if (((*pEmxArray)->data != (char_T *)NULL) && (*pEmxArray)->canFreeData) {
      free((*pEmxArray)->data);
    }

    free((*pEmxArray)->size);
    free(*pEmxArray);
    *pEmxArray = (emxArray_char_T_forward_kinem_T *)NULL;
  }
}

static void RigidBodyTree_forwardKinematics(p_robotics_manip_internal_Rig_T *obj,
  const real_T qvec[6], emxArray_f_cell_wrap_forward__T *Ttree)
{
  n_robotics_manip_internal_Rig_T *body;
  emxArray_char_T_forward_kinem_T *switch_expression;
  static const int8_T tmp[16] = { 1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1
  };

  static const char_T tmp_0[5] = { 'f', 'i', 'x', 'e', 'd' };

  static const char_T tmp_1[8] = { 'r', 'e', 'v', 'o', 'l', 'u', 't', 'e' };

  int32_T exitg1;
  forward_kinematics_B.n = obj->NumBodies;
  for (forward_kinematics_B.b_kstr_c = 0; forward_kinematics_B.b_kstr_c < 16;
       forward_kinematics_B.b_kstr_c++) {
    forward_kinematics_B.c_f1[forward_kinematics_B.b_kstr_c] =
      tmp[forward_kinematics_B.b_kstr_c];
  }

  forward_kinematics_B.b_kstr_c = Ttree->size[0] * Ttree->size[1];
  Ttree->size[0] = 1;
  forward_kinematics_B.e = static_cast<int32_T>(forward_kinematics_B.n);
  Ttree->size[1] = forward_kinematics_B.e;
  f_emxEnsureCapacity_f_cell_wrap(Ttree, forward_kinematics_B.b_kstr_c);
  if (forward_kinematics_B.e != 0) {
    forward_kinematics_B.ntilecols = forward_kinematics_B.e - 1;
    if (0 <= forward_kinematics_B.ntilecols) {
      memcpy(&forward_kinematics_B.expl_temp.f1[0], &forward_kinematics_B.c_f1[0],
             sizeof(real_T) << 4U);
    }

    for (forward_kinematics_B.b_jtilecol = 0; forward_kinematics_B.b_jtilecol <=
         forward_kinematics_B.ntilecols; forward_kinematics_B.b_jtilecol++) {
      Ttree->data[forward_kinematics_B.b_jtilecol] =
        forward_kinematics_B.expl_temp;
    }
  }

  forward_kinematics_B.k = 1.0;
  forward_kinematics_B.ntilecols = static_cast<int32_T>(forward_kinematics_B.n)
    - 1;
  forward_kinemati_emxInit_char_T(&switch_expression, 2);
  if (0 <= forward_kinematics_B.ntilecols) {
    for (forward_kinematics_B.b_kstr_c = 0; forward_kinematics_B.b_kstr_c < 5;
         forward_kinematics_B.b_kstr_c++) {
      forward_kinematics_B.b_f[forward_kinematics_B.b_kstr_c] =
        tmp_0[forward_kinematics_B.b_kstr_c];
    }
  }

  for (forward_kinematics_B.b_jtilecol = 0; forward_kinematics_B.b_jtilecol <=
       forward_kinematics_B.ntilecols; forward_kinematics_B.b_jtilecol++) {
    body = obj->Bodies[forward_kinematics_B.b_jtilecol];
    forward_kinematics_B.n = body->JointInternal.PositionNumber;
    forward_kinematics_B.n += forward_kinematics_B.k;
    if (forward_kinematics_B.k > forward_kinematics_B.n - 1.0) {
      forward_kinematics_B.e = 0;
      forward_kinematics_B.d_m = 0;
    } else {
      forward_kinematics_B.e = static_cast<int32_T>(forward_kinematics_B.k) - 1;
      forward_kinematics_B.d_m = static_cast<int32_T>(forward_kinematics_B.n -
        1.0);
    }

    forward_kinematics_B.b_kstr_c = switch_expression->size[0] *
      switch_expression->size[1];
    switch_expression->size[0] = 1;
    switch_expression->size[1] = body->JointInternal.Type->size[1];
    forwar_emxEnsureCapacity_char_T(switch_expression,
      forward_kinematics_B.b_kstr_c);
    forward_kinematics_B.loop_ub_f = body->JointInternal.Type->size[0] *
      body->JointInternal.Type->size[1] - 1;
    for (forward_kinematics_B.b_kstr_c = 0; forward_kinematics_B.b_kstr_c <=
         forward_kinematics_B.loop_ub_f; forward_kinematics_B.b_kstr_c++) {
      switch_expression->data[forward_kinematics_B.b_kstr_c] =
        body->JointInternal.Type->data[forward_kinematics_B.b_kstr_c];
    }

    forward_kinematics_B.b_bool_p = false;
    if (switch_expression->size[1] == 5) {
      forward_kinematics_B.b_kstr_c = 1;
      do {
        exitg1 = 0;
        if (forward_kinematics_B.b_kstr_c - 1 < 5) {
          forward_kinematics_B.loop_ub_f = forward_kinematics_B.b_kstr_c - 1;
          if (switch_expression->data[forward_kinematics_B.loop_ub_f] !=
              forward_kinematics_B.b_f[forward_kinematics_B.loop_ub_f]) {
            exitg1 = 1;
          } else {
            forward_kinematics_B.b_kstr_c++;
          }
        } else {
          forward_kinematics_B.b_bool_p = true;
          exitg1 = 1;
        }
      } while (exitg1 == 0);
    }

    if (forward_kinematics_B.b_bool_p) {
      forward_kinematics_B.b_kstr_c = 0;
    } else {
      for (forward_kinematics_B.b_kstr_c = 0; forward_kinematics_B.b_kstr_c < 8;
           forward_kinematics_B.b_kstr_c++) {
        forward_kinematics_B.b_bn[forward_kinematics_B.b_kstr_c] =
          tmp_1[forward_kinematics_B.b_kstr_c];
      }

      forward_kinematics_B.b_bool_p = false;
      if (switch_expression->size[1] == 8) {
        forward_kinematics_B.b_kstr_c = 1;
        do {
          exitg1 = 0;
          if (forward_kinematics_B.b_kstr_c - 1 < 8) {
            forward_kinematics_B.loop_ub_f = forward_kinematics_B.b_kstr_c - 1;
            if (switch_expression->data[forward_kinematics_B.loop_ub_f] !=
                forward_kinematics_B.b_bn[forward_kinematics_B.loop_ub_f]) {
              exitg1 = 1;
            } else {
              forward_kinematics_B.b_kstr_c++;
            }
          } else {
            forward_kinematics_B.b_bool_p = true;
            exitg1 = 1;
          }
        } while (exitg1 == 0);
      }

      if (forward_kinematics_B.b_bool_p) {
        forward_kinematics_B.b_kstr_c = 1;
      } else {
        forward_kinematics_B.b_kstr_c = -1;
      }
    }

    switch (forward_kinematics_B.b_kstr_c) {
     case 0:
      memset(&forward_kinematics_B.c_f1[0], 0, sizeof(real_T) << 4U);
      forward_kinematics_B.c_f1[0] = 1.0;
      forward_kinematics_B.c_f1[5] = 1.0;
      forward_kinematics_B.c_f1[10] = 1.0;
      forward_kinematics_B.c_f1[15] = 1.0;
      break;

     case 1:
      fo_rigidBodyJoint_get_JointAxis(&body->JointInternal,
        forward_kinematics_B.v);
      forward_kinematics_B.d_m -= forward_kinematics_B.e;
      for (forward_kinematics_B.b_kstr_c = 0; forward_kinematics_B.b_kstr_c <
           forward_kinematics_B.d_m; forward_kinematics_B.b_kstr_c++) {
        forward_kinematics_B.e_data[forward_kinematics_B.b_kstr_c] =
          forward_kinematics_B.e + forward_kinematics_B.b_kstr_c;
      }

      forward_kinematics_B.result_data[0] = forward_kinematics_B.v[0];
      forward_kinematics_B.result_data[1] = forward_kinematics_B.v[1];
      forward_kinematics_B.result_data[2] = forward_kinematics_B.v[2];
      if (0 <= (forward_kinematics_B.d_m != 0) - 1) {
        forward_kinematics_B.result_data[3] = qvec[forward_kinematics_B.e_data[0]];
      }

      forward_kinematics_B.k = 1.0 / sqrt((forward_kinematics_B.result_data[0] *
        forward_kinematics_B.result_data[0] + forward_kinematics_B.result_data[1]
        * forward_kinematics_B.result_data[1]) +
        forward_kinematics_B.result_data[2] * forward_kinematics_B.result_data[2]);
      forward_kinematics_B.v[0] = forward_kinematics_B.result_data[0] *
        forward_kinematics_B.k;
      forward_kinematics_B.v[1] = forward_kinematics_B.result_data[1] *
        forward_kinematics_B.k;
      forward_kinematics_B.v[2] = forward_kinematics_B.result_data[2] *
        forward_kinematics_B.k;
      forward_kinematics_B.k = cos(forward_kinematics_B.result_data[3]);
      forward_kinematics_B.sth = sin(forward_kinematics_B.result_data[3]);
      forward_kinematics_B.tempR[0] = forward_kinematics_B.v[0] *
        forward_kinematics_B.v[0] * (1.0 - forward_kinematics_B.k) +
        forward_kinematics_B.k;
      forward_kinematics_B.tempR_tmp = forward_kinematics_B.v[1] *
        forward_kinematics_B.v[0] * (1.0 - forward_kinematics_B.k);
      forward_kinematics_B.tempR_tmp_f = forward_kinematics_B.v[2] *
        forward_kinematics_B.sth;
      forward_kinematics_B.tempR[1] = forward_kinematics_B.tempR_tmp -
        forward_kinematics_B.tempR_tmp_f;
      forward_kinematics_B.tempR_tmp_a = forward_kinematics_B.v[2] *
        forward_kinematics_B.v[0] * (1.0 - forward_kinematics_B.k);
      forward_kinematics_B.tempR_tmp_j = forward_kinematics_B.v[1] *
        forward_kinematics_B.sth;
      forward_kinematics_B.tempR[2] = forward_kinematics_B.tempR_tmp_a +
        forward_kinematics_B.tempR_tmp_j;
      forward_kinematics_B.tempR[3] = forward_kinematics_B.tempR_tmp +
        forward_kinematics_B.tempR_tmp_f;
      forward_kinematics_B.tempR[4] = forward_kinematics_B.v[1] *
        forward_kinematics_B.v[1] * (1.0 - forward_kinematics_B.k) +
        forward_kinematics_B.k;
      forward_kinematics_B.tempR_tmp = forward_kinematics_B.v[2] *
        forward_kinematics_B.v[1] * (1.0 - forward_kinematics_B.k);
      forward_kinematics_B.tempR_tmp_f = forward_kinematics_B.v[0] *
        forward_kinematics_B.sth;
      forward_kinematics_B.tempR[5] = forward_kinematics_B.tempR_tmp -
        forward_kinematics_B.tempR_tmp_f;
      forward_kinematics_B.tempR[6] = forward_kinematics_B.tempR_tmp_a -
        forward_kinematics_B.tempR_tmp_j;
      forward_kinematics_B.tempR[7] = forward_kinematics_B.tempR_tmp +
        forward_kinematics_B.tempR_tmp_f;
      forward_kinematics_B.tempR[8] = forward_kinematics_B.v[2] *
        forward_kinematics_B.v[2] * (1.0 - forward_kinematics_B.k) +
        forward_kinematics_B.k;
      for (forward_kinematics_B.b_kstr_c = 0; forward_kinematics_B.b_kstr_c < 3;
           forward_kinematics_B.b_kstr_c++) {
        forward_kinematics_B.e = forward_kinematics_B.b_kstr_c + 1;
        forward_kinematics_B.R_p[forward_kinematics_B.e - 1] =
          forward_kinematics_B.tempR[(forward_kinematics_B.e - 1) * 3];
        forward_kinematics_B.e = forward_kinematics_B.b_kstr_c + 1;
        forward_kinematics_B.R_p[forward_kinematics_B.e + 2] =
          forward_kinematics_B.tempR[(forward_kinematics_B.e - 1) * 3 + 1];
        forward_kinematics_B.e = forward_kinematics_B.b_kstr_c + 1;
        forward_kinematics_B.R_p[forward_kinematics_B.e + 5] =
          forward_kinematics_B.tempR[(forward_kinematics_B.e - 1) * 3 + 2];
      }

      memset(&forward_kinematics_B.c_f1[0], 0, sizeof(real_T) << 4U);
      for (forward_kinematics_B.b_kstr_c = 0; forward_kinematics_B.b_kstr_c < 3;
           forward_kinematics_B.b_kstr_c++) {
        forward_kinematics_B.d_m = forward_kinematics_B.b_kstr_c << 2;
        forward_kinematics_B.c_f1[forward_kinematics_B.d_m] =
          forward_kinematics_B.R_p[3 * forward_kinematics_B.b_kstr_c];
        forward_kinematics_B.c_f1[forward_kinematics_B.d_m + 1] =
          forward_kinematics_B.R_p[3 * forward_kinematics_B.b_kstr_c + 1];
        forward_kinematics_B.c_f1[forward_kinematics_B.d_m + 2] =
          forward_kinematics_B.R_p[3 * forward_kinematics_B.b_kstr_c + 2];
      }

      forward_kinematics_B.c_f1[15] = 1.0;
      break;

     default:
      fo_rigidBodyJoint_get_JointAxis(&body->JointInternal,
        forward_kinematics_B.v);
      memset(&forward_kinematics_B.tempR[0], 0, 9U * sizeof(real_T));
      forward_kinematics_B.tempR[0] = 1.0;
      forward_kinematics_B.tempR[4] = 1.0;
      forward_kinematics_B.tempR[8] = 1.0;
      for (forward_kinematics_B.b_kstr_c = 0; forward_kinematics_B.b_kstr_c < 3;
           forward_kinematics_B.b_kstr_c++) {
        forward_kinematics_B.d_m = forward_kinematics_B.b_kstr_c << 2;
        forward_kinematics_B.c_f1[forward_kinematics_B.d_m] =
          forward_kinematics_B.tempR[3 * forward_kinematics_B.b_kstr_c];
        forward_kinematics_B.c_f1[forward_kinematics_B.d_m + 1] =
          forward_kinematics_B.tempR[3 * forward_kinematics_B.b_kstr_c + 1];
        forward_kinematics_B.c_f1[forward_kinematics_B.d_m + 2] =
          forward_kinematics_B.tempR[3 * forward_kinematics_B.b_kstr_c + 2];
        forward_kinematics_B.c_f1[forward_kinematics_B.b_kstr_c + 12] =
          forward_kinematics_B.v[forward_kinematics_B.b_kstr_c] *
          qvec[forward_kinematics_B.e];
      }

      forward_kinematics_B.c_f1[3] = 0.0;
      forward_kinematics_B.c_f1[7] = 0.0;
      forward_kinematics_B.c_f1[11] = 0.0;
      forward_kinematics_B.c_f1[15] = 1.0;
      break;
    }

    for (forward_kinematics_B.b_kstr_c = 0; forward_kinematics_B.b_kstr_c < 16;
         forward_kinematics_B.b_kstr_c++) {
      forward_kinematics_B.a[forward_kinematics_B.b_kstr_c] =
        body->JointInternal.JointToParentTransform[forward_kinematics_B.b_kstr_c];
    }

    for (forward_kinematics_B.b_kstr_c = 0; forward_kinematics_B.b_kstr_c < 16;
         forward_kinematics_B.b_kstr_c++) {
      forward_kinematics_B.b[forward_kinematics_B.b_kstr_c] =
        body->JointInternal.ChildToJointTransform[forward_kinematics_B.b_kstr_c];
    }

    for (forward_kinematics_B.b_kstr_c = 0; forward_kinematics_B.b_kstr_c < 4;
         forward_kinematics_B.b_kstr_c++) {
      for (forward_kinematics_B.e = 0; forward_kinematics_B.e < 4;
           forward_kinematics_B.e++) {
        forward_kinematics_B.d_m = forward_kinematics_B.e << 2;
        forward_kinematics_B.loop_ub_f = forward_kinematics_B.b_kstr_c +
          forward_kinematics_B.d_m;
        forward_kinematics_B.a_c[forward_kinematics_B.loop_ub_f] = 0.0;
        forward_kinematics_B.a_c[forward_kinematics_B.loop_ub_f] +=
          forward_kinematics_B.c_f1[forward_kinematics_B.d_m] *
          forward_kinematics_B.a[forward_kinematics_B.b_kstr_c];
        forward_kinematics_B.a_c[forward_kinematics_B.loop_ub_f] +=
          forward_kinematics_B.c_f1[forward_kinematics_B.d_m + 1] *
          forward_kinematics_B.a[forward_kinematics_B.b_kstr_c + 4];
        forward_kinematics_B.a_c[forward_kinematics_B.loop_ub_f] +=
          forward_kinematics_B.c_f1[forward_kinematics_B.d_m + 2] *
          forward_kinematics_B.a[forward_kinematics_B.b_kstr_c + 8];
        forward_kinematics_B.a_c[forward_kinematics_B.loop_ub_f] +=
          forward_kinematics_B.c_f1[forward_kinematics_B.d_m + 3] *
          forward_kinematics_B.a[forward_kinematics_B.b_kstr_c + 12];
      }

      for (forward_kinematics_B.e = 0; forward_kinematics_B.e < 4;
           forward_kinematics_B.e++) {
        forward_kinematics_B.d_m = forward_kinematics_B.e << 2;
        forward_kinematics_B.loop_ub_f = forward_kinematics_B.b_kstr_c +
          forward_kinematics_B.d_m;
        Ttree->data[forward_kinematics_B.b_jtilecol]
          .f1[forward_kinematics_B.loop_ub_f] = 0.0;
        Ttree->data[forward_kinematics_B.b_jtilecol]
          .f1[forward_kinematics_B.loop_ub_f] +=
          forward_kinematics_B.b[forward_kinematics_B.d_m] *
          forward_kinematics_B.a_c[forward_kinematics_B.b_kstr_c];
        Ttree->data[forward_kinematics_B.b_jtilecol]
          .f1[forward_kinematics_B.loop_ub_f] +=
          forward_kinematics_B.b[forward_kinematics_B.d_m + 1] *
          forward_kinematics_B.a_c[forward_kinematics_B.b_kstr_c + 4];
        Ttree->data[forward_kinematics_B.b_jtilecol]
          .f1[forward_kinematics_B.loop_ub_f] +=
          forward_kinematics_B.b[forward_kinematics_B.d_m + 2] *
          forward_kinematics_B.a_c[forward_kinematics_B.b_kstr_c + 8];
        Ttree->data[forward_kinematics_B.b_jtilecol]
          .f1[forward_kinematics_B.loop_ub_f] +=
          forward_kinematics_B.b[forward_kinematics_B.d_m + 3] *
          forward_kinematics_B.a_c[forward_kinematics_B.b_kstr_c + 12];
      }
    }

    forward_kinematics_B.k = forward_kinematics_B.n;
    if (body->ParentIndex > 0.0) {
      for (forward_kinematics_B.b_kstr_c = 0; forward_kinematics_B.b_kstr_c < 16;
           forward_kinematics_B.b_kstr_c++) {
        forward_kinematics_B.a[forward_kinematics_B.b_kstr_c] = Ttree->data[
          static_cast<int32_T>(body->ParentIndex) - 1]
          .f1[forward_kinematics_B.b_kstr_c];
      }

      for (forward_kinematics_B.b_kstr_c = 0; forward_kinematics_B.b_kstr_c < 4;
           forward_kinematics_B.b_kstr_c++) {
        for (forward_kinematics_B.e = 0; forward_kinematics_B.e < 4;
             forward_kinematics_B.e++) {
          forward_kinematics_B.d_m = forward_kinematics_B.e << 2;
          forward_kinematics_B.loop_ub_f = forward_kinematics_B.b_kstr_c +
            forward_kinematics_B.d_m;
          forward_kinematics_B.a_c[forward_kinematics_B.loop_ub_f] = 0.0;
          forward_kinematics_B.a_c[forward_kinematics_B.loop_ub_f] +=
            Ttree->data[forward_kinematics_B.b_jtilecol]
            .f1[forward_kinematics_B.d_m] *
            forward_kinematics_B.a[forward_kinematics_B.b_kstr_c];
          forward_kinematics_B.a_c[forward_kinematics_B.loop_ub_f] +=
            Ttree->data[forward_kinematics_B.b_jtilecol]
            .f1[forward_kinematics_B.d_m + 1] *
            forward_kinematics_B.a[forward_kinematics_B.b_kstr_c + 4];
          forward_kinematics_B.a_c[forward_kinematics_B.loop_ub_f] +=
            Ttree->data[forward_kinematics_B.b_jtilecol]
            .f1[forward_kinematics_B.d_m + 2] *
            forward_kinematics_B.a[forward_kinematics_B.b_kstr_c + 8];
          forward_kinematics_B.a_c[forward_kinematics_B.loop_ub_f] +=
            Ttree->data[forward_kinematics_B.b_jtilecol]
            .f1[forward_kinematics_B.d_m + 3] *
            forward_kinematics_B.a[forward_kinematics_B.b_kstr_c + 12];
        }
      }

      for (forward_kinematics_B.b_kstr_c = 0; forward_kinematics_B.b_kstr_c < 16;
           forward_kinematics_B.b_kstr_c++) {
        Ttree->data[forward_kinematics_B.b_jtilecol]
          .f1[forward_kinematics_B.b_kstr_c] =
          forward_kinematics_B.a_c[forward_kinematics_B.b_kstr_c];
      }
    }
  }

  forward_kinemati_emxFree_char_T(&switch_expression);
}

static void forwar_emxEnsureCapacity_real_T(emxArray_real_T_forward_kinem_T
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

static void forward_kinemati_emxFree_real_T(emxArray_real_T_forward_kinem_T
  **pEmxArray)
{
  if (*pEmxArray != (emxArray_real_T_forward_kinem_T *)NULL) {
    if (((*pEmxArray)->data != (real_T *)NULL) && (*pEmxArray)->canFreeData) {
      free((*pEmxArray)->data);
    }

    free((*pEmxArray)->size);
    free(*pEmxArray);
    *pEmxArray = (emxArray_real_T_forward_kinem_T *)NULL;
  }
}

static void forward_kin_emxFree_f_cell_wrap(emxArray_f_cell_wrap_forward__T
  **pEmxArray)
{
  if (*pEmxArray != (emxArray_f_cell_wrap_forward__T *)NULL) {
    if (((*pEmxArray)->data != (f_cell_wrap_forward_kinematic_T *)NULL) &&
        (*pEmxArray)->canFreeData) {
      free((*pEmxArray)->data);
    }

    free((*pEmxArray)->size);
    free(*pEmxArray);
    *pEmxArray = (emxArray_f_cell_wrap_forward__T *)NULL;
  }
}

static void RigidBodyTree_geometricJacobian(p_robotics_manip_internal_Rig_T *obj,
  const real_T Q[6], emxArray_real_T_forward_kinem_T *Jac)
{
  emxArray_f_cell_wrap_forward__T *Ttree;
  n_robotics_manip_internal_Rig_T *body;
  emxArray_real_T_forward_kinem_T *JacSlice;
  emxArray_char_T_forward_kinem_T *bname;
  emxArray_real_T_forward_kinem_T *b;
  static const char_T tmp[11] = { 'e', 'd', 'o', '_', 'l', 'i', 'n', 'k', '_',
    'e', 'e' };

  static const char_T tmp_0[5] = { 'f', 'i', 'x', 'e', 'd' };

  int32_T exitg1;
  boolean_T exitg2;
  forward_kin_emxInit_f_cell_wrap(&Ttree, 2);
  RigidBodyTree_forwardKinematics(obj, Q, Ttree);
  forward_kinematics_B.b_kstr_o = Jac->size[0] * Jac->size[1];
  Jac->size[0] = 6;
  Jac->size[1] = static_cast<int32_T>(obj->VelocityNumber);
  forwar_emxEnsureCapacity_real_T(Jac, forward_kinematics_B.b_kstr_o);
  forward_kinematics_B.loop_ub_m = 6 * static_cast<int32_T>(obj->VelocityNumber)
    - 1;
  for (forward_kinematics_B.b_kstr_o = 0; forward_kinematics_B.b_kstr_o <=
       forward_kinematics_B.loop_ub_m; forward_kinematics_B.b_kstr_o++) {
    Jac->data[forward_kinematics_B.b_kstr_o] = 0.0;
  }

  for (forward_kinematics_B.b_kstr_o = 0; forward_kinematics_B.b_kstr_o < 8;
       forward_kinematics_B.b_kstr_o++) {
    forward_kinematics_B.chainmask[forward_kinematics_B.b_kstr_o] = 0;
  }

  forward_kinemati_emxInit_char_T(&bname, 2);
  forward_kinematics_B.b_kstr_o = bname->size[0] * bname->size[1];
  bname->size[0] = 1;
  bname->size[1] = obj->Base.NameInternal->size[1];
  forwar_emxEnsureCapacity_char_T(bname, forward_kinematics_B.b_kstr_o);
  forward_kinematics_B.loop_ub_m = obj->Base.NameInternal->size[0] *
    obj->Base.NameInternal->size[1] - 1;
  for (forward_kinematics_B.b_kstr_o = 0; forward_kinematics_B.b_kstr_o <=
       forward_kinematics_B.loop_ub_m; forward_kinematics_B.b_kstr_o++) {
    bname->data[forward_kinematics_B.b_kstr_o] = obj->Base.NameInternal->
      data[forward_kinematics_B.b_kstr_o];
  }

  for (forward_kinematics_B.b_kstr_o = 0; forward_kinematics_B.b_kstr_o < 11;
       forward_kinematics_B.b_kstr_o++) {
    forward_kinematics_B.a_n[forward_kinematics_B.b_kstr_o] =
      tmp[forward_kinematics_B.b_kstr_o];
  }

  forward_kinematics_B.b_bool = false;
  if (11 == bname->size[1]) {
    forward_kinematics_B.b_kstr_o = 1;
    do {
      exitg1 = 0;
      if (forward_kinematics_B.b_kstr_o - 1 < 11) {
        forward_kinematics_B.kstr = forward_kinematics_B.b_kstr_o - 1;
        if (forward_kinematics_B.a_n[forward_kinematics_B.kstr] != bname->
            data[forward_kinematics_B.kstr]) {
          exitg1 = 1;
        } else {
          forward_kinematics_B.b_kstr_o++;
        }
      } else {
        forward_kinematics_B.b_bool = true;
        exitg1 = 1;
      }
    } while (exitg1 == 0);
  }

  if (forward_kinematics_B.b_bool) {
    memset(&forward_kinematics_B.T2inv[0], 0, sizeof(real_T) << 4U);
    forward_kinematics_B.T2inv[0] = 1.0;
    forward_kinematics_B.T2inv[5] = 1.0;
    forward_kinematics_B.T2inv[10] = 1.0;
    forward_kinematics_B.T2inv[15] = 1.0;
    memset(&forward_kinematics_B.T2_c[0], 0, sizeof(real_T) << 4U);
    forward_kinematics_B.T2_c[0] = 1.0;
    forward_kinematics_B.T2_c[5] = 1.0;
    forward_kinematics_B.T2_c[10] = 1.0;
    forward_kinematics_B.T2_c[15] = 1.0;
  } else {
    forward_kinematics_B.endeffectorIndex = -1.0;
    forward_kinematics_B.b_kstr_o = bname->size[0] * bname->size[1];
    bname->size[0] = 1;
    bname->size[1] = obj->Base.NameInternal->size[1];
    forwar_emxEnsureCapacity_char_T(bname, forward_kinematics_B.b_kstr_o);
    forward_kinematics_B.loop_ub_m = obj->Base.NameInternal->size[0] *
      obj->Base.NameInternal->size[1] - 1;
    for (forward_kinematics_B.b_kstr_o = 0; forward_kinematics_B.b_kstr_o <=
         forward_kinematics_B.loop_ub_m; forward_kinematics_B.b_kstr_o++) {
      bname->data[forward_kinematics_B.b_kstr_o] = obj->Base.NameInternal->
        data[forward_kinematics_B.b_kstr_o];
    }

    for (forward_kinematics_B.b_kstr_o = 0; forward_kinematics_B.b_kstr_o < 11;
         forward_kinematics_B.b_kstr_o++) {
      forward_kinematics_B.a_n[forward_kinematics_B.b_kstr_o] =
        tmp[forward_kinematics_B.b_kstr_o];
    }

    forward_kinematics_B.b_bool = false;
    if (bname->size[1] == 11) {
      forward_kinematics_B.b_kstr_o = 1;
      do {
        exitg1 = 0;
        if (forward_kinematics_B.b_kstr_o - 1 < 11) {
          forward_kinematics_B.kstr = forward_kinematics_B.b_kstr_o - 1;
          if (bname->data[forward_kinematics_B.kstr] !=
              forward_kinematics_B.a_n[forward_kinematics_B.kstr]) {
            exitg1 = 1;
          } else {
            forward_kinematics_B.b_kstr_o++;
          }
        } else {
          forward_kinematics_B.b_bool = true;
          exitg1 = 1;
        }
      } while (exitg1 == 0);
    }

    if (forward_kinematics_B.b_bool) {
      forward_kinematics_B.endeffectorIndex = 0.0;
    } else {
      forward_kinematics_B.idx_idx_1 = obj->NumBodies;
      forward_kinematics_B.b_i_c = 0;
      exitg2 = false;
      while ((!exitg2) && (forward_kinematics_B.b_i_c <= static_cast<int32_T>
                           (forward_kinematics_B.idx_idx_1) - 1)) {
        body = obj->Bodies[forward_kinematics_B.b_i_c];
        forward_kinematics_B.b_kstr_o = bname->size[0] * bname->size[1];
        bname->size[0] = 1;
        bname->size[1] = body->NameInternal->size[1];
        forwar_emxEnsureCapacity_char_T(bname, forward_kinematics_B.b_kstr_o);
        forward_kinematics_B.loop_ub_m = body->NameInternal->size[0] *
          body->NameInternal->size[1] - 1;
        for (forward_kinematics_B.b_kstr_o = 0; forward_kinematics_B.b_kstr_o <=
             forward_kinematics_B.loop_ub_m; forward_kinematics_B.b_kstr_o++) {
          bname->data[forward_kinematics_B.b_kstr_o] = body->NameInternal->
            data[forward_kinematics_B.b_kstr_o];
        }

        for (forward_kinematics_B.b_kstr_o = 0; forward_kinematics_B.b_kstr_o <
             11; forward_kinematics_B.b_kstr_o++) {
          forward_kinematics_B.a_n[forward_kinematics_B.b_kstr_o] =
            tmp[forward_kinematics_B.b_kstr_o];
        }

        forward_kinematics_B.b_bool = false;
        if (bname->size[1] == 11) {
          forward_kinematics_B.b_kstr_o = 1;
          do {
            exitg1 = 0;
            if (forward_kinematics_B.b_kstr_o - 1 < 11) {
              forward_kinematics_B.kstr = forward_kinematics_B.b_kstr_o - 1;
              if (bname->data[forward_kinematics_B.kstr] !=
                  forward_kinematics_B.a_n[forward_kinematics_B.kstr]) {
                exitg1 = 1;
              } else {
                forward_kinematics_B.b_kstr_o++;
              }
            } else {
              forward_kinematics_B.b_bool = true;
              exitg1 = 1;
            }
          } while (exitg1 == 0);
        }

        if (forward_kinematics_B.b_bool) {
          forward_kinematics_B.endeffectorIndex = static_cast<real_T>
            (forward_kinematics_B.b_i_c) + 1.0;
          exitg2 = true;
        } else {
          forward_kinematics_B.b_i_c++;
        }
      }
    }

    forward_kinematics_B.b_i_c = static_cast<int32_T>
      (forward_kinematics_B.endeffectorIndex) - 1;
    body = obj->Bodies[forward_kinematics_B.b_i_c];
    for (forward_kinematics_B.b_kstr_o = 0; forward_kinematics_B.b_kstr_o < 16;
         forward_kinematics_B.b_kstr_o++) {
      forward_kinematics_B.T2_c[forward_kinematics_B.b_kstr_o] = Ttree->
        data[forward_kinematics_B.b_i_c].f1[forward_kinematics_B.b_kstr_o];
    }

    for (forward_kinematics_B.b_kstr_o = 0; forward_kinematics_B.b_kstr_o < 3;
         forward_kinematics_B.b_kstr_o++) {
      forward_kinematics_B.R_m[3 * forward_kinematics_B.b_kstr_o] = Ttree->
        data[forward_kinematics_B.b_i_c].f1[forward_kinematics_B.b_kstr_o];
      forward_kinematics_B.R_m[3 * forward_kinematics_B.b_kstr_o + 1] =
        Ttree->data[forward_kinematics_B.b_i_c].f1[forward_kinematics_B.b_kstr_o
        + 4];
      forward_kinematics_B.R_m[3 * forward_kinematics_B.b_kstr_o + 2] =
        Ttree->data[forward_kinematics_B.b_i_c].f1[forward_kinematics_B.b_kstr_o
        + 8];
    }

    for (forward_kinematics_B.b_kstr_o = 0; forward_kinematics_B.b_kstr_o < 9;
         forward_kinematics_B.b_kstr_o++) {
      forward_kinematics_B.R_n[forward_kinematics_B.b_kstr_o] =
        -forward_kinematics_B.R_m[forward_kinematics_B.b_kstr_o];
    }

    for (forward_kinematics_B.b_kstr_o = 0; forward_kinematics_B.b_kstr_o < 3;
         forward_kinematics_B.b_kstr_o++) {
      forward_kinematics_B.endeffectorIndex = Ttree->
        data[forward_kinematics_B.b_i_c].f1[12] *
        forward_kinematics_B.R_n[forward_kinematics_B.b_kstr_o];
      forward_kinematics_B.loop_ub_m = forward_kinematics_B.b_kstr_o << 2;
      forward_kinematics_B.T2inv[forward_kinematics_B.loop_ub_m] =
        forward_kinematics_B.R_m[3 * forward_kinematics_B.b_kstr_o];
      forward_kinematics_B.endeffectorIndex +=
        forward_kinematics_B.R_n[forward_kinematics_B.b_kstr_o + 3] *
        Ttree->data[forward_kinematics_B.b_i_c].f1[13];
      forward_kinematics_B.T2inv[forward_kinematics_B.loop_ub_m + 1] =
        forward_kinematics_B.R_m[3 * forward_kinematics_B.b_kstr_o + 1];
      forward_kinematics_B.endeffectorIndex +=
        forward_kinematics_B.R_n[forward_kinematics_B.b_kstr_o + 6] *
        Ttree->data[forward_kinematics_B.b_i_c].f1[14];
      forward_kinematics_B.T2inv[forward_kinematics_B.loop_ub_m + 2] =
        forward_kinematics_B.R_m[3 * forward_kinematics_B.b_kstr_o + 2];
      forward_kinematics_B.T2inv[forward_kinematics_B.b_kstr_o + 12] =
        forward_kinematics_B.endeffectorIndex;
    }

    forward_kinematics_B.T2inv[3] = 0.0;
    forward_kinematics_B.T2inv[7] = 0.0;
    forward_kinematics_B.T2inv[11] = 0.0;
    forward_kinematics_B.T2inv[15] = 1.0;
    forward_kinematics_B.chainmask[forward_kinematics_B.b_i_c] = 1;
    while (body->ParentIndex > 0.0) {
      body = obj->Bodies[static_cast<int32_T>(body->ParentIndex) - 1];
      forward_kinematics_B.chainmask[static_cast<int32_T>(body->Index) - 1] = 1;
    }
  }

  forward_kinematics_B.idx_idx_1 = obj->NumBodies;
  forward_kinematics_B.c_g = static_cast<int32_T>(forward_kinematics_B.idx_idx_1)
    - 1;
  forward_kinemati_emxInit_real_T(&JacSlice, 2);
  forward_kinemati_emxInit_real_T(&b, 2);
  if (0 <= forward_kinematics_B.c_g) {
    for (forward_kinematics_B.b_kstr_o = 0; forward_kinematics_B.b_kstr_o < 5;
         forward_kinematics_B.b_kstr_o++) {
      forward_kinematics_B.b_i[forward_kinematics_B.b_kstr_o] =
        tmp_0[forward_kinematics_B.b_kstr_o];
    }
  }

  for (forward_kinematics_B.b_i_c = 0; forward_kinematics_B.b_i_c <=
       forward_kinematics_B.c_g; forward_kinematics_B.b_i_c++) {
    body = obj->Bodies[forward_kinematics_B.b_i_c];
    forward_kinematics_B.b_kstr_o = bname->size[0] * bname->size[1];
    bname->size[0] = 1;
    bname->size[1] = body->JointInternal.Type->size[1];
    forwar_emxEnsureCapacity_char_T(bname, forward_kinematics_B.b_kstr_o);
    forward_kinematics_B.loop_ub_m = body->JointInternal.Type->size[0] *
      body->JointInternal.Type->size[1] - 1;
    for (forward_kinematics_B.b_kstr_o = 0; forward_kinematics_B.b_kstr_o <=
         forward_kinematics_B.loop_ub_m; forward_kinematics_B.b_kstr_o++) {
      bname->data[forward_kinematics_B.b_kstr_o] = body->
        JointInternal.Type->data[forward_kinematics_B.b_kstr_o];
    }

    forward_kinematics_B.b_bool = false;
    if (bname->size[1] == 5) {
      forward_kinematics_B.b_kstr_o = 1;
      do {
        exitg1 = 0;
        if (forward_kinematics_B.b_kstr_o - 1 < 5) {
          forward_kinematics_B.kstr = forward_kinematics_B.b_kstr_o - 1;
          if (bname->data[forward_kinematics_B.kstr] !=
              forward_kinematics_B.b_i[forward_kinematics_B.kstr]) {
            exitg1 = 1;
          } else {
            forward_kinematics_B.b_kstr_o++;
          }
        } else {
          forward_kinematics_B.b_bool = true;
          exitg1 = 1;
        }
      } while (exitg1 == 0);
    }

    if ((!forward_kinematics_B.b_bool) &&
        (forward_kinematics_B.chainmask[forward_kinematics_B.b_i_c] != 0)) {
      for (forward_kinematics_B.b_kstr_o = 0; forward_kinematics_B.b_kstr_o < 16;
           forward_kinematics_B.b_kstr_o++) {
        forward_kinematics_B.T1_k[forward_kinematics_B.b_kstr_o] = Ttree->data[
          static_cast<int32_T>(body->Index) - 1]
          .f1[forward_kinematics_B.b_kstr_o];
      }

      for (forward_kinematics_B.b_kstr_o = 0; forward_kinematics_B.b_kstr_o < 16;
           forward_kinematics_B.b_kstr_o++) {
        forward_kinematics_B.Tdh[forward_kinematics_B.b_kstr_o] =
          body->
          JointInternal.ChildToJointTransform[forward_kinematics_B.b_kstr_o];
      }

      for (forward_kinematics_B.b_kstr_o = 0; forward_kinematics_B.b_kstr_o < 3;
           forward_kinematics_B.b_kstr_o++) {
        forward_kinematics_B.R_m[3 * forward_kinematics_B.b_kstr_o] =
          forward_kinematics_B.Tdh[forward_kinematics_B.b_kstr_o];
        forward_kinematics_B.R_m[3 * forward_kinematics_B.b_kstr_o + 1] =
          forward_kinematics_B.Tdh[forward_kinematics_B.b_kstr_o + 4];
        forward_kinematics_B.R_m[3 * forward_kinematics_B.b_kstr_o + 2] =
          forward_kinematics_B.Tdh[forward_kinematics_B.b_kstr_o + 8];
      }

      for (forward_kinematics_B.b_kstr_o = 0; forward_kinematics_B.b_kstr_o < 9;
           forward_kinematics_B.b_kstr_o++) {
        forward_kinematics_B.R_n[forward_kinematics_B.b_kstr_o] =
          -forward_kinematics_B.R_m[forward_kinematics_B.b_kstr_o];
      }

      for (forward_kinematics_B.b_kstr_o = 0; forward_kinematics_B.b_kstr_o < 3;
           forward_kinematics_B.b_kstr_o++) {
        forward_kinematics_B.R_d[forward_kinematics_B.b_kstr_o] =
          forward_kinematics_B.R_n[forward_kinematics_B.b_kstr_o + 6] *
          forward_kinematics_B.Tdh[14] +
          (forward_kinematics_B.R_n[forward_kinematics_B.b_kstr_o + 3] *
           forward_kinematics_B.Tdh[13] +
           forward_kinematics_B.R_n[forward_kinematics_B.b_kstr_o] *
           forward_kinematics_B.Tdh[12]);
      }

      for (forward_kinematics_B.b_kstr_o = 0; forward_kinematics_B.b_kstr_o < 4;
           forward_kinematics_B.b_kstr_o++) {
        for (forward_kinematics_B.kstr = 0; forward_kinematics_B.kstr < 4;
             forward_kinematics_B.kstr++) {
          forward_kinematics_B.n_l = forward_kinematics_B.kstr << 2;
          forward_kinematics_B.loop_ub_m = forward_kinematics_B.b_kstr_o +
            forward_kinematics_B.n_l;
          forward_kinematics_B.Tdh[forward_kinematics_B.loop_ub_m] = 0.0;
          forward_kinematics_B.Tdh[forward_kinematics_B.loop_ub_m] +=
            forward_kinematics_B.T1_k[forward_kinematics_B.n_l] *
            forward_kinematics_B.T2inv[forward_kinematics_B.b_kstr_o];
          forward_kinematics_B.Tdh[forward_kinematics_B.loop_ub_m] +=
            forward_kinematics_B.T1_k[forward_kinematics_B.n_l + 1] *
            forward_kinematics_B.T2inv[forward_kinematics_B.b_kstr_o + 4];
          forward_kinematics_B.Tdh[forward_kinematics_B.loop_ub_m] +=
            forward_kinematics_B.T1_k[forward_kinematics_B.n_l + 2] *
            forward_kinematics_B.T2inv[forward_kinematics_B.b_kstr_o + 8];
          forward_kinematics_B.Tdh[forward_kinematics_B.loop_ub_m] +=
            forward_kinematics_B.T1_k[forward_kinematics_B.n_l + 3] *
            forward_kinematics_B.T2inv[forward_kinematics_B.b_kstr_o + 12];
        }
      }

      for (forward_kinematics_B.b_kstr_o = 0; forward_kinematics_B.b_kstr_o < 3;
           forward_kinematics_B.b_kstr_o++) {
        forward_kinematics_B.kstr = forward_kinematics_B.b_kstr_o << 2;
        forward_kinematics_B.T1_k[forward_kinematics_B.kstr] =
          forward_kinematics_B.R_m[3 * forward_kinematics_B.b_kstr_o];
        forward_kinematics_B.T1_k[forward_kinematics_B.kstr + 1] =
          forward_kinematics_B.R_m[3 * forward_kinematics_B.b_kstr_o + 1];
        forward_kinematics_B.T1_k[forward_kinematics_B.kstr + 2] =
          forward_kinematics_B.R_m[3 * forward_kinematics_B.b_kstr_o + 2];
        forward_kinematics_B.T1_k[forward_kinematics_B.b_kstr_o + 12] =
          forward_kinematics_B.R_d[forward_kinematics_B.b_kstr_o];
      }

      forward_kinematics_B.T1_k[3] = 0.0;
      forward_kinematics_B.T1_k[7] = 0.0;
      forward_kinematics_B.T1_k[11] = 0.0;
      forward_kinematics_B.T1_k[15] = 1.0;
      for (forward_kinematics_B.b_kstr_o = 0; forward_kinematics_B.b_kstr_o < 4;
           forward_kinematics_B.b_kstr_o++) {
        for (forward_kinematics_B.kstr = 0; forward_kinematics_B.kstr < 4;
             forward_kinematics_B.kstr++) {
          forward_kinematics_B.loop_ub_m = forward_kinematics_B.kstr << 2;
          forward_kinematics_B.n_l = forward_kinematics_B.b_kstr_o +
            forward_kinematics_B.loop_ub_m;
          forward_kinematics_B.T[forward_kinematics_B.n_l] = 0.0;
          forward_kinematics_B.T[forward_kinematics_B.n_l] +=
            forward_kinematics_B.T1_k[forward_kinematics_B.loop_ub_m] *
            forward_kinematics_B.Tdh[forward_kinematics_B.b_kstr_o];
          forward_kinematics_B.T[forward_kinematics_B.n_l] +=
            forward_kinematics_B.T1_k[forward_kinematics_B.loop_ub_m + 1] *
            forward_kinematics_B.Tdh[forward_kinematics_B.b_kstr_o + 4];
          forward_kinematics_B.T[forward_kinematics_B.n_l] +=
            forward_kinematics_B.T1_k[forward_kinematics_B.loop_ub_m + 2] *
            forward_kinematics_B.Tdh[forward_kinematics_B.b_kstr_o + 8];
          forward_kinematics_B.T[forward_kinematics_B.n_l] +=
            forward_kinematics_B.T1_k[forward_kinematics_B.loop_ub_m + 3] *
            forward_kinematics_B.Tdh[forward_kinematics_B.b_kstr_o + 12];
        }
      }

      forward_kinematics_B.endeffectorIndex = obj->
        PositionDoFMap[forward_kinematics_B.b_i_c];
      forward_kinematics_B.idx_idx_1 = obj->
        PositionDoFMap[forward_kinematics_B.b_i_c + 8];
      forward_kinematics_B.R_m[0] = 0.0;
      forward_kinematics_B.R_m[3] = -forward_kinematics_B.T[14];
      forward_kinematics_B.R_m[6] = forward_kinematics_B.T[13];
      forward_kinematics_B.R_m[1] = forward_kinematics_B.T[14];
      forward_kinematics_B.R_m[4] = 0.0;
      forward_kinematics_B.R_m[7] = -forward_kinematics_B.T[12];
      forward_kinematics_B.R_m[2] = -forward_kinematics_B.T[13];
      forward_kinematics_B.R_m[5] = forward_kinematics_B.T[12];
      forward_kinematics_B.R_m[8] = 0.0;
      for (forward_kinematics_B.b_kstr_o = 0; forward_kinematics_B.b_kstr_o < 3;
           forward_kinematics_B.b_kstr_o++) {
        for (forward_kinematics_B.kstr = 0; forward_kinematics_B.kstr < 3;
             forward_kinematics_B.kstr++) {
          forward_kinematics_B.loop_ub_m = forward_kinematics_B.b_kstr_o + 3 *
            forward_kinematics_B.kstr;
          forward_kinematics_B.R_n[forward_kinematics_B.loop_ub_m] = 0.0;
          forward_kinematics_B.n_l = forward_kinematics_B.kstr << 2;
          forward_kinematics_B.R_n[forward_kinematics_B.loop_ub_m] +=
            forward_kinematics_B.T[forward_kinematics_B.n_l] *
            forward_kinematics_B.R_m[forward_kinematics_B.b_kstr_o];
          forward_kinematics_B.R_n[forward_kinematics_B.loop_ub_m] +=
            forward_kinematics_B.T[forward_kinematics_B.n_l + 1] *
            forward_kinematics_B.R_m[forward_kinematics_B.b_kstr_o + 3];
          forward_kinematics_B.R_n[forward_kinematics_B.loop_ub_m] +=
            forward_kinematics_B.T[forward_kinematics_B.n_l + 2] *
            forward_kinematics_B.R_m[forward_kinematics_B.b_kstr_o + 6];
          forward_kinematics_B.X[forward_kinematics_B.kstr + 6 *
            forward_kinematics_B.b_kstr_o] = forward_kinematics_B.T
            [(forward_kinematics_B.b_kstr_o << 2) + forward_kinematics_B.kstr];
          forward_kinematics_B.X[forward_kinematics_B.kstr + 6 *
            (forward_kinematics_B.b_kstr_o + 3)] = 0.0;
        }
      }

      for (forward_kinematics_B.b_kstr_o = 0; forward_kinematics_B.b_kstr_o < 3;
           forward_kinematics_B.b_kstr_o++) {
        forward_kinematics_B.X[6 * forward_kinematics_B.b_kstr_o + 3] =
          forward_kinematics_B.R_n[3 * forward_kinematics_B.b_kstr_o];
        forward_kinematics_B.kstr = forward_kinematics_B.b_kstr_o << 2;
        forward_kinematics_B.loop_ub_m = 6 * (forward_kinematics_B.b_kstr_o + 3);
        forward_kinematics_B.X[forward_kinematics_B.loop_ub_m + 3] =
          forward_kinematics_B.T[forward_kinematics_B.kstr];
        forward_kinematics_B.X[6 * forward_kinematics_B.b_kstr_o + 4] =
          forward_kinematics_B.R_n[3 * forward_kinematics_B.b_kstr_o + 1];
        forward_kinematics_B.X[forward_kinematics_B.loop_ub_m + 4] =
          forward_kinematics_B.T[forward_kinematics_B.kstr + 1];
        forward_kinematics_B.X[6 * forward_kinematics_B.b_kstr_o + 5] =
          forward_kinematics_B.R_n[3 * forward_kinematics_B.b_kstr_o + 2];
        forward_kinematics_B.X[forward_kinematics_B.loop_ub_m + 5] =
          forward_kinematics_B.T[forward_kinematics_B.kstr + 2];
      }

      forward_kinematics_B.b_kstr_o = b->size[0] * b->size[1];
      b->size[0] = 6;
      b->size[1] = body->JointInternal.MotionSubspace->size[1];
      forwar_emxEnsureCapacity_real_T(b, forward_kinematics_B.b_kstr_o);
      forward_kinematics_B.loop_ub_m = body->JointInternal.MotionSubspace->size
        [0] * body->JointInternal.MotionSubspace->size[1] - 1;
      for (forward_kinematics_B.b_kstr_o = 0; forward_kinematics_B.b_kstr_o <=
           forward_kinematics_B.loop_ub_m; forward_kinematics_B.b_kstr_o++) {
        b->data[forward_kinematics_B.b_kstr_o] =
          body->JointInternal.MotionSubspace->data[forward_kinematics_B.b_kstr_o];
      }

      forward_kinematics_B.n_l = b->size[1] - 1;
      forward_kinematics_B.b_kstr_o = JacSlice->size[0] * JacSlice->size[1];
      JacSlice->size[0] = 6;
      JacSlice->size[1] = b->size[1];
      forwar_emxEnsureCapacity_real_T(JacSlice, forward_kinematics_B.b_kstr_o);
      for (forward_kinematics_B.b_kstr_o = 0; forward_kinematics_B.b_kstr_o <=
           forward_kinematics_B.n_l; forward_kinematics_B.b_kstr_o++) {
        forward_kinematics_B.coffset_tmp = forward_kinematics_B.b_kstr_o * 6 - 1;
        for (forward_kinematics_B.kstr = 0; forward_kinematics_B.kstr < 6;
             forward_kinematics_B.kstr++) {
          forward_kinematics_B.s_j = 0.0;
          for (forward_kinematics_B.loop_ub_m = 0;
               forward_kinematics_B.loop_ub_m < 6;
               forward_kinematics_B.loop_ub_m++) {
            forward_kinematics_B.s_j +=
              forward_kinematics_B.X[forward_kinematics_B.loop_ub_m * 6 +
              forward_kinematics_B.kstr] * b->data
              [(forward_kinematics_B.coffset_tmp +
                forward_kinematics_B.loop_ub_m) + 1];
          }

          JacSlice->data[(forward_kinematics_B.coffset_tmp +
                          forward_kinematics_B.kstr) + 1] =
            forward_kinematics_B.s_j;
        }
      }

      if (forward_kinematics_B.endeffectorIndex > forward_kinematics_B.idx_idx_1)
      {
        forward_kinematics_B.n_l = 0;
      } else {
        forward_kinematics_B.n_l = static_cast<int32_T>
          (forward_kinematics_B.endeffectorIndex) - 1;
      }

      forward_kinematics_B.loop_ub_m = JacSlice->size[1];
      for (forward_kinematics_B.b_kstr_o = 0; forward_kinematics_B.b_kstr_o <
           forward_kinematics_B.loop_ub_m; forward_kinematics_B.b_kstr_o++) {
        for (forward_kinematics_B.kstr = 0; forward_kinematics_B.kstr < 6;
             forward_kinematics_B.kstr++) {
          Jac->data[forward_kinematics_B.kstr + 6 * (forward_kinematics_B.n_l +
            forward_kinematics_B.b_kstr_o)] = JacSlice->data[6 *
            forward_kinematics_B.b_kstr_o + forward_kinematics_B.kstr];
        }
      }
    }
  }

  forward_kinemati_emxFree_char_T(&bname);
  forward_kinemati_emxFree_real_T(&JacSlice);
  forward_kin_emxFree_f_cell_wrap(&Ttree);
  for (forward_kinematics_B.b_kstr_o = 0; forward_kinematics_B.b_kstr_o < 3;
       forward_kinematics_B.b_kstr_o++) {
    forward_kinematics_B.b_i_c = forward_kinematics_B.b_kstr_o << 2;
    forward_kinematics_B.X[6 * forward_kinematics_B.b_kstr_o] =
      forward_kinematics_B.T2_c[forward_kinematics_B.b_i_c];
    forward_kinematics_B.kstr = 6 * (forward_kinematics_B.b_kstr_o + 3);
    forward_kinematics_B.X[forward_kinematics_B.kstr] = 0.0;
    forward_kinematics_B.X[6 * forward_kinematics_B.b_kstr_o + 3] = 0.0;
    forward_kinematics_B.X[forward_kinematics_B.kstr + 3] =
      forward_kinematics_B.T2_c[forward_kinematics_B.b_i_c];
    forward_kinematics_B.endeffectorIndex =
      forward_kinematics_B.T2_c[forward_kinematics_B.b_i_c + 1];
    forward_kinematics_B.X[6 * forward_kinematics_B.b_kstr_o + 1] =
      forward_kinematics_B.endeffectorIndex;
    forward_kinematics_B.X[forward_kinematics_B.kstr + 1] = 0.0;
    forward_kinematics_B.X[6 * forward_kinematics_B.b_kstr_o + 4] = 0.0;
    forward_kinematics_B.X[forward_kinematics_B.kstr + 4] =
      forward_kinematics_B.endeffectorIndex;
    forward_kinematics_B.endeffectorIndex =
      forward_kinematics_B.T2_c[forward_kinematics_B.b_i_c + 2];
    forward_kinematics_B.X[6 * forward_kinematics_B.b_kstr_o + 2] =
      forward_kinematics_B.endeffectorIndex;
    forward_kinematics_B.X[forward_kinematics_B.kstr + 2] = 0.0;
    forward_kinematics_B.X[6 * forward_kinematics_B.b_kstr_o + 5] = 0.0;
    forward_kinematics_B.X[forward_kinematics_B.kstr + 5] =
      forward_kinematics_B.endeffectorIndex;
  }

  forward_kinematics_B.n_l = Jac->size[1];
  forward_kinematics_B.b_kstr_o = b->size[0] * b->size[1];
  b->size[0] = 6;
  b->size[1] = Jac->size[1];
  forwar_emxEnsureCapacity_real_T(b, forward_kinematics_B.b_kstr_o);
  forward_kinematics_B.loop_ub_m = Jac->size[0] * Jac->size[1] - 1;
  for (forward_kinematics_B.b_kstr_o = 0; forward_kinematics_B.b_kstr_o <=
       forward_kinematics_B.loop_ub_m; forward_kinematics_B.b_kstr_o++) {
    b->data[forward_kinematics_B.b_kstr_o] = Jac->
      data[forward_kinematics_B.b_kstr_o];
  }

  forward_kinematics_B.b_kstr_o = Jac->size[0] * Jac->size[1];
  Jac->size[0] = 6;
  Jac->size[1] = forward_kinematics_B.n_l;
  forwar_emxEnsureCapacity_real_T(Jac, forward_kinematics_B.b_kstr_o);
  for (forward_kinematics_B.b_kstr_o = 0; forward_kinematics_B.b_kstr_o <
       forward_kinematics_B.n_l; forward_kinematics_B.b_kstr_o++) {
    forward_kinematics_B.coffset_tmp = forward_kinematics_B.b_kstr_o * 6 - 1;
    for (forward_kinematics_B.b_i_c = 0; forward_kinematics_B.b_i_c < 6;
         forward_kinematics_B.b_i_c++) {
      forward_kinematics_B.s_j = 0.0;
      for (forward_kinematics_B.loop_ub_m = 0; forward_kinematics_B.loop_ub_m <
           6; forward_kinematics_B.loop_ub_m++) {
        forward_kinematics_B.s_j +=
          forward_kinematics_B.X[forward_kinematics_B.loop_ub_m * 6 +
          forward_kinematics_B.b_i_c] * b->data
          [(forward_kinematics_B.coffset_tmp + forward_kinematics_B.loop_ub_m) +
          1];
      }

      Jac->data[(forward_kinematics_B.coffset_tmp + forward_kinematics_B.b_i_c)
        + 1] = forward_kinematics_B.s_j;
    }
  }

  forward_kinemati_emxFree_real_T(&b);
}

static void rigidBodyJoint_get_JointAxis_e(const c_rigidBodyJoint_forward_ki_e_T
  *obj, real_T ax[3])
{
  static const char_T tmp[8] = { 'r', 'e', 'v', 'o', 'l', 'u', 't', 'e' };

  static const char_T tmp_0[9] = { 'p', 'r', 'i', 's', 'm', 'a', 't', 'i', 'c' };

  boolean_T guard1 = false;
  int32_T exitg1;
  for (forward_kinematics_B.b_kstr_p = 0; forward_kinematics_B.b_kstr_p < 8;
       forward_kinematics_B.b_kstr_p++) {
    forward_kinematics_B.b_bj[forward_kinematics_B.b_kstr_p] =
      tmp[forward_kinematics_B.b_kstr_p];
  }

  forward_kinematics_B.b_bool_i = false;
  if (obj->Type->size[1] == 8) {
    forward_kinematics_B.b_kstr_p = 1;
    do {
      exitg1 = 0;
      if (forward_kinematics_B.b_kstr_p - 1 < 8) {
        forward_kinematics_B.kstr_pc = forward_kinematics_B.b_kstr_p - 1;
        if (obj->Type->data[forward_kinematics_B.kstr_pc] !=
            forward_kinematics_B.b_bj[forward_kinematics_B.kstr_pc]) {
          exitg1 = 1;
        } else {
          forward_kinematics_B.b_kstr_p++;
        }
      } else {
        forward_kinematics_B.b_bool_i = true;
        exitg1 = 1;
      }
    } while (exitg1 == 0);
  }

  guard1 = false;
  if (forward_kinematics_B.b_bool_i) {
    guard1 = true;
  } else {
    for (forward_kinematics_B.b_kstr_p = 0; forward_kinematics_B.b_kstr_p < 9;
         forward_kinematics_B.b_kstr_p++) {
      forward_kinematics_B.b_l[forward_kinematics_B.b_kstr_p] =
        tmp_0[forward_kinematics_B.b_kstr_p];
    }

    forward_kinematics_B.b_bool_i = false;
    if (obj->Type->size[1] == 9) {
      forward_kinematics_B.b_kstr_p = 1;
      do {
        exitg1 = 0;
        if (forward_kinematics_B.b_kstr_p - 1 < 9) {
          forward_kinematics_B.kstr_pc = forward_kinematics_B.b_kstr_p - 1;
          if (obj->Type->data[forward_kinematics_B.kstr_pc] !=
              forward_kinematics_B.b_l[forward_kinematics_B.kstr_pc]) {
            exitg1 = 1;
          } else {
            forward_kinematics_B.b_kstr_p++;
          }
        } else {
          forward_kinematics_B.b_bool_i = true;
          exitg1 = 1;
        }
      } while (exitg1 == 0);
    }

    if (forward_kinematics_B.b_bool_i) {
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

static void RigidBodyTree_forwardKinemati_e(p_robotics_manip_internal_R_e_T *obj,
  const real_T qvec[6], emxArray_f_cell_wrap_forward__T *Ttree)
{
  n_robotics_manip_internal_R_e_T *body;
  emxArray_char_T_forward_kinem_T *switch_expression;
  static const int8_T tmp[16] = { 1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1
  };

  static const char_T tmp_0[5] = { 'f', 'i', 'x', 'e', 'd' };

  static const char_T tmp_1[8] = { 'r', 'e', 'v', 'o', 'l', 'u', 't', 'e' };

  int32_T exitg1;
  forward_kinematics_B.n_i = obj->NumBodies;
  for (forward_kinematics_B.b_kstr_cs = 0; forward_kinematics_B.b_kstr_cs < 16;
       forward_kinematics_B.b_kstr_cs++) {
    forward_kinematics_B.c_f1_b[forward_kinematics_B.b_kstr_cs] =
      tmp[forward_kinematics_B.b_kstr_cs];
  }

  forward_kinematics_B.b_kstr_cs = Ttree->size[0] * Ttree->size[1];
  Ttree->size[0] = 1;
  forward_kinematics_B.e_m = static_cast<int32_T>(forward_kinematics_B.n_i);
  Ttree->size[1] = forward_kinematics_B.e_m;
  f_emxEnsureCapacity_f_cell_wrap(Ttree, forward_kinematics_B.b_kstr_cs);
  if (forward_kinematics_B.e_m != 0) {
    forward_kinematics_B.ntilecols_m = forward_kinematics_B.e_m - 1;
    if (0 <= forward_kinematics_B.ntilecols_m) {
      memcpy(&forward_kinematics_B.expl_temp_m.f1[0],
             &forward_kinematics_B.c_f1_b[0], sizeof(real_T) << 4U);
    }

    for (forward_kinematics_B.b_jtilecol_h = 0;
         forward_kinematics_B.b_jtilecol_h <= forward_kinematics_B.ntilecols_m;
         forward_kinematics_B.b_jtilecol_h++) {
      Ttree->data[forward_kinematics_B.b_jtilecol_h] =
        forward_kinematics_B.expl_temp_m;
    }
  }

  forward_kinematics_B.k_o = 1.0;
  forward_kinematics_B.ntilecols_m = static_cast<int32_T>
    (forward_kinematics_B.n_i) - 1;
  forward_kinemati_emxInit_char_T(&switch_expression, 2);
  if (0 <= forward_kinematics_B.ntilecols_m) {
    for (forward_kinematics_B.b_kstr_cs = 0; forward_kinematics_B.b_kstr_cs < 5;
         forward_kinematics_B.b_kstr_cs++) {
      forward_kinematics_B.b_iz[forward_kinematics_B.b_kstr_cs] =
        tmp_0[forward_kinematics_B.b_kstr_cs];
    }
  }

  for (forward_kinematics_B.b_jtilecol_h = 0; forward_kinematics_B.b_jtilecol_h <=
       forward_kinematics_B.ntilecols_m; forward_kinematics_B.b_jtilecol_h++) {
    body = obj->Bodies[forward_kinematics_B.b_jtilecol_h];
    forward_kinematics_B.n_i = body->JointInternal.PositionNumber;
    forward_kinematics_B.n_i += forward_kinematics_B.k_o;
    if (forward_kinematics_B.k_o > forward_kinematics_B.n_i - 1.0) {
      forward_kinematics_B.e_m = 0;
      forward_kinematics_B.d_h = 0;
    } else {
      forward_kinematics_B.e_m = static_cast<int32_T>(forward_kinematics_B.k_o)
        - 1;
      forward_kinematics_B.d_h = static_cast<int32_T>(forward_kinematics_B.n_i -
        1.0);
    }

    forward_kinematics_B.b_kstr_cs = switch_expression->size[0] *
      switch_expression->size[1];
    switch_expression->size[0] = 1;
    switch_expression->size[1] = body->JointInternal.Type->size[1];
    forwar_emxEnsureCapacity_char_T(switch_expression,
      forward_kinematics_B.b_kstr_cs);
    forward_kinematics_B.loop_ub_k = body->JointInternal.Type->size[0] *
      body->JointInternal.Type->size[1] - 1;
    for (forward_kinematics_B.b_kstr_cs = 0; forward_kinematics_B.b_kstr_cs <=
         forward_kinematics_B.loop_ub_k; forward_kinematics_B.b_kstr_cs++) {
      switch_expression->data[forward_kinematics_B.b_kstr_cs] =
        body->JointInternal.Type->data[forward_kinematics_B.b_kstr_cs];
    }

    forward_kinematics_B.b_bool_f = false;
    if (switch_expression->size[1] == 5) {
      forward_kinematics_B.b_kstr_cs = 1;
      do {
        exitg1 = 0;
        if (forward_kinematics_B.b_kstr_cs - 1 < 5) {
          forward_kinematics_B.loop_ub_k = forward_kinematics_B.b_kstr_cs - 1;
          if (switch_expression->data[forward_kinematics_B.loop_ub_k] !=
              forward_kinematics_B.b_iz[forward_kinematics_B.loop_ub_k]) {
            exitg1 = 1;
          } else {
            forward_kinematics_B.b_kstr_cs++;
          }
        } else {
          forward_kinematics_B.b_bool_f = true;
          exitg1 = 1;
        }
      } while (exitg1 == 0);
    }

    if (forward_kinematics_B.b_bool_f) {
      forward_kinematics_B.b_kstr_cs = 0;
    } else {
      for (forward_kinematics_B.b_kstr_cs = 0; forward_kinematics_B.b_kstr_cs <
           8; forward_kinematics_B.b_kstr_cs++) {
        forward_kinematics_B.b_e[forward_kinematics_B.b_kstr_cs] =
          tmp_1[forward_kinematics_B.b_kstr_cs];
      }

      forward_kinematics_B.b_bool_f = false;
      if (switch_expression->size[1] == 8) {
        forward_kinematics_B.b_kstr_cs = 1;
        do {
          exitg1 = 0;
          if (forward_kinematics_B.b_kstr_cs - 1 < 8) {
            forward_kinematics_B.loop_ub_k = forward_kinematics_B.b_kstr_cs - 1;
            if (switch_expression->data[forward_kinematics_B.loop_ub_k] !=
                forward_kinematics_B.b_e[forward_kinematics_B.loop_ub_k]) {
              exitg1 = 1;
            } else {
              forward_kinematics_B.b_kstr_cs++;
            }
          } else {
            forward_kinematics_B.b_bool_f = true;
            exitg1 = 1;
          }
        } while (exitg1 == 0);
      }

      if (forward_kinematics_B.b_bool_f) {
        forward_kinematics_B.b_kstr_cs = 1;
      } else {
        forward_kinematics_B.b_kstr_cs = -1;
      }
    }

    switch (forward_kinematics_B.b_kstr_cs) {
     case 0:
      memset(&forward_kinematics_B.c_f1_b[0], 0, sizeof(real_T) << 4U);
      forward_kinematics_B.c_f1_b[0] = 1.0;
      forward_kinematics_B.c_f1_b[5] = 1.0;
      forward_kinematics_B.c_f1_b[10] = 1.0;
      forward_kinematics_B.c_f1_b[15] = 1.0;
      break;

     case 1:
      rigidBodyJoint_get_JointAxis_e(&body->JointInternal,
        forward_kinematics_B.v_d);
      forward_kinematics_B.d_h -= forward_kinematics_B.e_m;
      for (forward_kinematics_B.b_kstr_cs = 0; forward_kinematics_B.b_kstr_cs <
           forward_kinematics_B.d_h; forward_kinematics_B.b_kstr_cs++) {
        forward_kinematics_B.e_data_l[forward_kinematics_B.b_kstr_cs] =
          forward_kinematics_B.e_m + forward_kinematics_B.b_kstr_cs;
      }

      forward_kinematics_B.result_data_d[0] = forward_kinematics_B.v_d[0];
      forward_kinematics_B.result_data_d[1] = forward_kinematics_B.v_d[1];
      forward_kinematics_B.result_data_d[2] = forward_kinematics_B.v_d[2];
      if (0 <= (forward_kinematics_B.d_h != 0) - 1) {
        forward_kinematics_B.result_data_d[3] =
          qvec[forward_kinematics_B.e_data_l[0]];
      }

      forward_kinematics_B.k_o = 1.0 / sqrt((forward_kinematics_B.result_data_d
        [0] * forward_kinematics_B.result_data_d[0] +
        forward_kinematics_B.result_data_d[1] *
        forward_kinematics_B.result_data_d[1]) +
        forward_kinematics_B.result_data_d[2] *
        forward_kinematics_B.result_data_d[2]);
      forward_kinematics_B.v_d[0] = forward_kinematics_B.result_data_d[0] *
        forward_kinematics_B.k_o;
      forward_kinematics_B.v_d[1] = forward_kinematics_B.result_data_d[1] *
        forward_kinematics_B.k_o;
      forward_kinematics_B.v_d[2] = forward_kinematics_B.result_data_d[2] *
        forward_kinematics_B.k_o;
      forward_kinematics_B.k_o = cos(forward_kinematics_B.result_data_d[3]);
      forward_kinematics_B.sth_n = sin(forward_kinematics_B.result_data_d[3]);
      forward_kinematics_B.tempR_j[0] = forward_kinematics_B.v_d[0] *
        forward_kinematics_B.v_d[0] * (1.0 - forward_kinematics_B.k_o) +
        forward_kinematics_B.k_o;
      forward_kinematics_B.tempR_tmp_m = forward_kinematics_B.v_d[1] *
        forward_kinematics_B.v_d[0] * (1.0 - forward_kinematics_B.k_o);
      forward_kinematics_B.tempR_tmp_c = forward_kinematics_B.v_d[2] *
        forward_kinematics_B.sth_n;
      forward_kinematics_B.tempR_j[1] = forward_kinematics_B.tempR_tmp_m -
        forward_kinematics_B.tempR_tmp_c;
      forward_kinematics_B.tempR_tmp_md = forward_kinematics_B.v_d[2] *
        forward_kinematics_B.v_d[0] * (1.0 - forward_kinematics_B.k_o);
      forward_kinematics_B.tempR_tmp_m3 = forward_kinematics_B.v_d[1] *
        forward_kinematics_B.sth_n;
      forward_kinematics_B.tempR_j[2] = forward_kinematics_B.tempR_tmp_md +
        forward_kinematics_B.tempR_tmp_m3;
      forward_kinematics_B.tempR_j[3] = forward_kinematics_B.tempR_tmp_m +
        forward_kinematics_B.tempR_tmp_c;
      forward_kinematics_B.tempR_j[4] = forward_kinematics_B.v_d[1] *
        forward_kinematics_B.v_d[1] * (1.0 - forward_kinematics_B.k_o) +
        forward_kinematics_B.k_o;
      forward_kinematics_B.tempR_tmp_m = forward_kinematics_B.v_d[2] *
        forward_kinematics_B.v_d[1] * (1.0 - forward_kinematics_B.k_o);
      forward_kinematics_B.tempR_tmp_c = forward_kinematics_B.v_d[0] *
        forward_kinematics_B.sth_n;
      forward_kinematics_B.tempR_j[5] = forward_kinematics_B.tempR_tmp_m -
        forward_kinematics_B.tempR_tmp_c;
      forward_kinematics_B.tempR_j[6] = forward_kinematics_B.tempR_tmp_md -
        forward_kinematics_B.tempR_tmp_m3;
      forward_kinematics_B.tempR_j[7] = forward_kinematics_B.tempR_tmp_m +
        forward_kinematics_B.tempR_tmp_c;
      forward_kinematics_B.tempR_j[8] = forward_kinematics_B.v_d[2] *
        forward_kinematics_B.v_d[2] * (1.0 - forward_kinematics_B.k_o) +
        forward_kinematics_B.k_o;
      for (forward_kinematics_B.b_kstr_cs = 0; forward_kinematics_B.b_kstr_cs <
           3; forward_kinematics_B.b_kstr_cs++) {
        forward_kinematics_B.e_m = forward_kinematics_B.b_kstr_cs + 1;
        forward_kinematics_B.R_l[forward_kinematics_B.e_m - 1] =
          forward_kinematics_B.tempR_j[(forward_kinematics_B.e_m - 1) * 3];
        forward_kinematics_B.e_m = forward_kinematics_B.b_kstr_cs + 1;
        forward_kinematics_B.R_l[forward_kinematics_B.e_m + 2] =
          forward_kinematics_B.tempR_j[(forward_kinematics_B.e_m - 1) * 3 + 1];
        forward_kinematics_B.e_m = forward_kinematics_B.b_kstr_cs + 1;
        forward_kinematics_B.R_l[forward_kinematics_B.e_m + 5] =
          forward_kinematics_B.tempR_j[(forward_kinematics_B.e_m - 1) * 3 + 2];
      }

      memset(&forward_kinematics_B.c_f1_b[0], 0, sizeof(real_T) << 4U);
      for (forward_kinematics_B.b_kstr_cs = 0; forward_kinematics_B.b_kstr_cs <
           3; forward_kinematics_B.b_kstr_cs++) {
        forward_kinematics_B.d_h = forward_kinematics_B.b_kstr_cs << 2;
        forward_kinematics_B.c_f1_b[forward_kinematics_B.d_h] =
          forward_kinematics_B.R_l[3 * forward_kinematics_B.b_kstr_cs];
        forward_kinematics_B.c_f1_b[forward_kinematics_B.d_h + 1] =
          forward_kinematics_B.R_l[3 * forward_kinematics_B.b_kstr_cs + 1];
        forward_kinematics_B.c_f1_b[forward_kinematics_B.d_h + 2] =
          forward_kinematics_B.R_l[3 * forward_kinematics_B.b_kstr_cs + 2];
      }

      forward_kinematics_B.c_f1_b[15] = 1.0;
      break;

     default:
      rigidBodyJoint_get_JointAxis_e(&body->JointInternal,
        forward_kinematics_B.v_d);
      memset(&forward_kinematics_B.tempR_j[0], 0, 9U * sizeof(real_T));
      forward_kinematics_B.tempR_j[0] = 1.0;
      forward_kinematics_B.tempR_j[4] = 1.0;
      forward_kinematics_B.tempR_j[8] = 1.0;
      for (forward_kinematics_B.b_kstr_cs = 0; forward_kinematics_B.b_kstr_cs <
           3; forward_kinematics_B.b_kstr_cs++) {
        forward_kinematics_B.d_h = forward_kinematics_B.b_kstr_cs << 2;
        forward_kinematics_B.c_f1_b[forward_kinematics_B.d_h] =
          forward_kinematics_B.tempR_j[3 * forward_kinematics_B.b_kstr_cs];
        forward_kinematics_B.c_f1_b[forward_kinematics_B.d_h + 1] =
          forward_kinematics_B.tempR_j[3 * forward_kinematics_B.b_kstr_cs + 1];
        forward_kinematics_B.c_f1_b[forward_kinematics_B.d_h + 2] =
          forward_kinematics_B.tempR_j[3 * forward_kinematics_B.b_kstr_cs + 2];
        forward_kinematics_B.c_f1_b[forward_kinematics_B.b_kstr_cs + 12] =
          forward_kinematics_B.v_d[forward_kinematics_B.b_kstr_cs] *
          qvec[forward_kinematics_B.e_m];
      }

      forward_kinematics_B.c_f1_b[3] = 0.0;
      forward_kinematics_B.c_f1_b[7] = 0.0;
      forward_kinematics_B.c_f1_b[11] = 0.0;
      forward_kinematics_B.c_f1_b[15] = 1.0;
      break;
    }

    for (forward_kinematics_B.b_kstr_cs = 0; forward_kinematics_B.b_kstr_cs < 16;
         forward_kinematics_B.b_kstr_cs++) {
      forward_kinematics_B.a_p[forward_kinematics_B.b_kstr_cs] =
        body->
        JointInternal.JointToParentTransform[forward_kinematics_B.b_kstr_cs];
    }

    for (forward_kinematics_B.b_kstr_cs = 0; forward_kinematics_B.b_kstr_cs < 16;
         forward_kinematics_B.b_kstr_cs++) {
      forward_kinematics_B.b_c[forward_kinematics_B.b_kstr_cs] =
        body->JointInternal.ChildToJointTransform[forward_kinematics_B.b_kstr_cs];
    }

    for (forward_kinematics_B.b_kstr_cs = 0; forward_kinematics_B.b_kstr_cs < 4;
         forward_kinematics_B.b_kstr_cs++) {
      for (forward_kinematics_B.e_m = 0; forward_kinematics_B.e_m < 4;
           forward_kinematics_B.e_m++) {
        forward_kinematics_B.d_h = forward_kinematics_B.e_m << 2;
        forward_kinematics_B.loop_ub_k = forward_kinematics_B.b_kstr_cs +
          forward_kinematics_B.d_h;
        forward_kinematics_B.a_f[forward_kinematics_B.loop_ub_k] = 0.0;
        forward_kinematics_B.a_f[forward_kinematics_B.loop_ub_k] +=
          forward_kinematics_B.c_f1_b[forward_kinematics_B.d_h] *
          forward_kinematics_B.a_p[forward_kinematics_B.b_kstr_cs];
        forward_kinematics_B.a_f[forward_kinematics_B.loop_ub_k] +=
          forward_kinematics_B.c_f1_b[forward_kinematics_B.d_h + 1] *
          forward_kinematics_B.a_p[forward_kinematics_B.b_kstr_cs + 4];
        forward_kinematics_B.a_f[forward_kinematics_B.loop_ub_k] +=
          forward_kinematics_B.c_f1_b[forward_kinematics_B.d_h + 2] *
          forward_kinematics_B.a_p[forward_kinematics_B.b_kstr_cs + 8];
        forward_kinematics_B.a_f[forward_kinematics_B.loop_ub_k] +=
          forward_kinematics_B.c_f1_b[forward_kinematics_B.d_h + 3] *
          forward_kinematics_B.a_p[forward_kinematics_B.b_kstr_cs + 12];
      }

      for (forward_kinematics_B.e_m = 0; forward_kinematics_B.e_m < 4;
           forward_kinematics_B.e_m++) {
        forward_kinematics_B.d_h = forward_kinematics_B.e_m << 2;
        forward_kinematics_B.loop_ub_k = forward_kinematics_B.b_kstr_cs +
          forward_kinematics_B.d_h;
        Ttree->data[forward_kinematics_B.b_jtilecol_h]
          .f1[forward_kinematics_B.loop_ub_k] = 0.0;
        Ttree->data[forward_kinematics_B.b_jtilecol_h]
          .f1[forward_kinematics_B.loop_ub_k] +=
          forward_kinematics_B.b_c[forward_kinematics_B.d_h] *
          forward_kinematics_B.a_f[forward_kinematics_B.b_kstr_cs];
        Ttree->data[forward_kinematics_B.b_jtilecol_h]
          .f1[forward_kinematics_B.loop_ub_k] +=
          forward_kinematics_B.b_c[forward_kinematics_B.d_h + 1] *
          forward_kinematics_B.a_f[forward_kinematics_B.b_kstr_cs + 4];
        Ttree->data[forward_kinematics_B.b_jtilecol_h]
          .f1[forward_kinematics_B.loop_ub_k] +=
          forward_kinematics_B.b_c[forward_kinematics_B.d_h + 2] *
          forward_kinematics_B.a_f[forward_kinematics_B.b_kstr_cs + 8];
        Ttree->data[forward_kinematics_B.b_jtilecol_h]
          .f1[forward_kinematics_B.loop_ub_k] +=
          forward_kinematics_B.b_c[forward_kinematics_B.d_h + 3] *
          forward_kinematics_B.a_f[forward_kinematics_B.b_kstr_cs + 12];
      }
    }

    forward_kinematics_B.k_o = forward_kinematics_B.n_i;
    if (body->ParentIndex > 0.0) {
      for (forward_kinematics_B.b_kstr_cs = 0; forward_kinematics_B.b_kstr_cs <
           16; forward_kinematics_B.b_kstr_cs++) {
        forward_kinematics_B.a_p[forward_kinematics_B.b_kstr_cs] = Ttree->data[
          static_cast<int32_T>(body->ParentIndex) - 1]
          .f1[forward_kinematics_B.b_kstr_cs];
      }

      for (forward_kinematics_B.b_kstr_cs = 0; forward_kinematics_B.b_kstr_cs <
           4; forward_kinematics_B.b_kstr_cs++) {
        for (forward_kinematics_B.e_m = 0; forward_kinematics_B.e_m < 4;
             forward_kinematics_B.e_m++) {
          forward_kinematics_B.d_h = forward_kinematics_B.e_m << 2;
          forward_kinematics_B.loop_ub_k = forward_kinematics_B.b_kstr_cs +
            forward_kinematics_B.d_h;
          forward_kinematics_B.a_f[forward_kinematics_B.loop_ub_k] = 0.0;
          forward_kinematics_B.a_f[forward_kinematics_B.loop_ub_k] +=
            Ttree->data[forward_kinematics_B.b_jtilecol_h]
            .f1[forward_kinematics_B.d_h] *
            forward_kinematics_B.a_p[forward_kinematics_B.b_kstr_cs];
          forward_kinematics_B.a_f[forward_kinematics_B.loop_ub_k] +=
            Ttree->data[forward_kinematics_B.b_jtilecol_h]
            .f1[forward_kinematics_B.d_h + 1] *
            forward_kinematics_B.a_p[forward_kinematics_B.b_kstr_cs + 4];
          forward_kinematics_B.a_f[forward_kinematics_B.loop_ub_k] +=
            Ttree->data[forward_kinematics_B.b_jtilecol_h]
            .f1[forward_kinematics_B.d_h + 2] *
            forward_kinematics_B.a_p[forward_kinematics_B.b_kstr_cs + 8];
          forward_kinematics_B.a_f[forward_kinematics_B.loop_ub_k] +=
            Ttree->data[forward_kinematics_B.b_jtilecol_h]
            .f1[forward_kinematics_B.d_h + 3] *
            forward_kinematics_B.a_p[forward_kinematics_B.b_kstr_cs + 12];
        }
      }

      for (forward_kinematics_B.b_kstr_cs = 0; forward_kinematics_B.b_kstr_cs <
           16; forward_kinematics_B.b_kstr_cs++) {
        Ttree->data[forward_kinematics_B.b_jtilecol_h]
          .f1[forward_kinematics_B.b_kstr_cs] =
          forward_kinematics_B.a_f[forward_kinematics_B.b_kstr_cs];
      }
    }
  }

  forward_kinemati_emxFree_char_T(&switch_expression);
}

// Function for MATLAB Function: '<S1>/MATLAB Function'
static boolean_T forward_kinematics_anyNonFinite(const real_T x[16])
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

static real_T forward_kinematic_rt_hypotd_snf(real_T u0, real_T u1)
{
  real_T y;
  forward_kinematics_B.a_ny = fabs(u0);
  y = fabs(u1);
  if (forward_kinematics_B.a_ny < y) {
    forward_kinematics_B.a_ny /= y;
    y *= sqrt(forward_kinematics_B.a_ny * forward_kinematics_B.a_ny + 1.0);
  } else if (forward_kinematics_B.a_ny > y) {
    y /= forward_kinematics_B.a_ny;
    y = sqrt(y * y + 1.0) * forward_kinematics_B.a_ny;
  } else {
    if (!rtIsNaN(y)) {
      y = forward_kinematics_B.a_ny * 1.4142135623730951;
    }
  }

  return y;
}

// Function for MATLAB Function: '<S1>/MATLAB Function'
static real_T forward_kinematics_xzlangeM(const creal_T x[16])
{
  real_T y;
  real_T absxk;
  int32_T k;
  boolean_T exitg1;
  y = 0.0;
  k = 0;
  exitg1 = false;
  while ((!exitg1) && (k < 16)) {
    absxk = forward_kinematic_rt_hypotd_snf(x[k].re, x[k].im);
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

// Function for MATLAB Function: '<S1>/MATLAB Function'
static void forward_kinematics_xzlascl(real_T cfrom, real_T cto, creal_T A[16])
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

// Function for MATLAB Function: '<S1>/MATLAB Function'
static real_T forward_kinematics_xzlanhs(const creal_T A[16], int32_T ilo,
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

// Function for MATLAB Function: '<S1>/MATLAB Function'
static void forward_kinematics_xzlartg_k(const creal_T f, const creal_T g,
  real_T *cs, creal_T *sn)
{
  real_T gs_im;
  boolean_T guard1 = false;
  forward_kinematics_B.f2s_e = fabs(f.re);
  forward_kinematics_B.di_a = fabs(f.im);
  forward_kinematics_B.scale_p5 = forward_kinematics_B.f2s_e;
  if (forward_kinematics_B.di_a > forward_kinematics_B.f2s_e) {
    forward_kinematics_B.scale_p5 = forward_kinematics_B.di_a;
  }

  forward_kinematics_B.gs_re_o = fabs(g.re);
  gs_im = fabs(g.im);
  if (gs_im > forward_kinematics_B.gs_re_o) {
    forward_kinematics_B.gs_re_o = gs_im;
  }

  if (forward_kinematics_B.gs_re_o > forward_kinematics_B.scale_p5) {
    forward_kinematics_B.scale_p5 = forward_kinematics_B.gs_re_o;
  }

  forward_kinematics_B.fs_re_i = f.re;
  forward_kinematics_B.fs_im_l = f.im;
  forward_kinematics_B.gs_re_o = g.re;
  gs_im = g.im;
  guard1 = false;
  if (forward_kinematics_B.scale_p5 >= 7.4428285367870146E+137) {
    do {
      forward_kinematics_B.fs_re_i *= 1.3435752215134178E-138;
      forward_kinematics_B.fs_im_l *= 1.3435752215134178E-138;
      forward_kinematics_B.gs_re_o *= 1.3435752215134178E-138;
      gs_im *= 1.3435752215134178E-138;
      forward_kinematics_B.scale_p5 *= 1.3435752215134178E-138;
    } while (!(forward_kinematics_B.scale_p5 < 7.4428285367870146E+137));

    guard1 = true;
  } else if (forward_kinematics_B.scale_p5 <= 1.3435752215134178E-138) {
    if ((g.re == 0.0) && (g.im == 0.0)) {
      *cs = 1.0;
      sn->re = 0.0;
      sn->im = 0.0;
    } else {
      do {
        forward_kinematics_B.fs_re_i *= 7.4428285367870146E+137;
        forward_kinematics_B.fs_im_l *= 7.4428285367870146E+137;
        forward_kinematics_B.gs_re_o *= 7.4428285367870146E+137;
        gs_im *= 7.4428285367870146E+137;
        forward_kinematics_B.scale_p5 *= 7.4428285367870146E+137;
      } while (!(forward_kinematics_B.scale_p5 > 1.3435752215134178E-138));

      guard1 = true;
    }
  } else {
    guard1 = true;
  }

  if (guard1) {
    forward_kinematics_B.scale_p5 = forward_kinematics_B.fs_re_i *
      forward_kinematics_B.fs_re_i + forward_kinematics_B.fs_im_l *
      forward_kinematics_B.fs_im_l;
    forward_kinematics_B.g2_a = forward_kinematics_B.gs_re_o *
      forward_kinematics_B.gs_re_o + gs_im * gs_im;
    forward_kinematics_B.x_a = forward_kinematics_B.g2_a;
    if (1.0 > forward_kinematics_B.g2_a) {
      forward_kinematics_B.x_a = 1.0;
    }

    if (forward_kinematics_B.scale_p5 <= forward_kinematics_B.x_a *
        2.0041683600089728E-292) {
      if ((f.re == 0.0) && (f.im == 0.0)) {
        *cs = 0.0;
        forward_kinematics_B.f2s_e = forward_kinematic_rt_hypotd_snf
          (forward_kinematics_B.gs_re_o, gs_im);
        sn->re = forward_kinematics_B.gs_re_o / forward_kinematics_B.f2s_e;
        sn->im = -gs_im / forward_kinematics_B.f2s_e;
      } else {
        forward_kinematics_B.scale_p5 = sqrt(forward_kinematics_B.g2_a);
        *cs = forward_kinematic_rt_hypotd_snf(forward_kinematics_B.fs_re_i,
          forward_kinematics_B.fs_im_l) / forward_kinematics_B.scale_p5;
        if (forward_kinematics_B.di_a > forward_kinematics_B.f2s_e) {
          forward_kinematics_B.f2s_e = forward_kinematics_B.di_a;
        }

        if (forward_kinematics_B.f2s_e > 1.0) {
          forward_kinematics_B.f2s_e = forward_kinematic_rt_hypotd_snf(f.re,
            f.im);
          forward_kinematics_B.fs_re_i = f.re / forward_kinematics_B.f2s_e;
          forward_kinematics_B.fs_im_l = f.im / forward_kinematics_B.f2s_e;
        } else {
          forward_kinematics_B.fs_re_i = 7.4428285367870146E+137 * f.re;
          forward_kinematics_B.di_a = 7.4428285367870146E+137 * f.im;
          forward_kinematics_B.f2s_e = forward_kinematic_rt_hypotd_snf
            (forward_kinematics_B.fs_re_i, forward_kinematics_B.di_a);
          forward_kinematics_B.fs_re_i /= forward_kinematics_B.f2s_e;
          forward_kinematics_B.fs_im_l = forward_kinematics_B.di_a /
            forward_kinematics_B.f2s_e;
        }

        forward_kinematics_B.gs_re_o /= forward_kinematics_B.scale_p5;
        gs_im = -gs_im / forward_kinematics_B.scale_p5;
        sn->re = forward_kinematics_B.fs_re_i * forward_kinematics_B.gs_re_o -
          forward_kinematics_B.fs_im_l * gs_im;
        sn->im = forward_kinematics_B.fs_re_i * gs_im +
          forward_kinematics_B.fs_im_l * forward_kinematics_B.gs_re_o;
      }
    } else {
      forward_kinematics_B.f2s_e = sqrt(forward_kinematics_B.g2_a /
        forward_kinematics_B.scale_p5 + 1.0);
      forward_kinematics_B.fs_re_i *= forward_kinematics_B.f2s_e;
      forward_kinematics_B.fs_im_l *= forward_kinematics_B.f2s_e;
      *cs = 1.0 / forward_kinematics_B.f2s_e;
      forward_kinematics_B.f2s_e = forward_kinematics_B.scale_p5 +
        forward_kinematics_B.g2_a;
      forward_kinematics_B.fs_re_i /= forward_kinematics_B.f2s_e;
      forward_kinematics_B.fs_im_l /= forward_kinematics_B.f2s_e;
      sn->re = forward_kinematics_B.fs_re_i * forward_kinematics_B.gs_re_o -
        forward_kinematics_B.fs_im_l * -gs_im;
      sn->im = forward_kinematics_B.fs_re_i * -gs_im +
        forward_kinematics_B.fs_im_l * forward_kinematics_B.gs_re_o;
    }
  }
}

// Function for MATLAB Function: '<S1>/MATLAB Function'
static void forward_kinematics_xzlartg(const creal_T f, const creal_T g, real_T *
  cs, creal_T *sn, creal_T *r)
{
  int32_T count;
  int32_T rescaledir;
  boolean_T guard1 = false;
  forward_kinematics_B.f2s = fabs(f.re);
  forward_kinematics_B.di = fabs(f.im);
  forward_kinematics_B.scale_o = forward_kinematics_B.f2s;
  if (forward_kinematics_B.di > forward_kinematics_B.f2s) {
    forward_kinematics_B.scale_o = forward_kinematics_B.di;
  }

  forward_kinematics_B.gs_re = fabs(g.re);
  forward_kinematics_B.gs_im = fabs(g.im);
  if (forward_kinematics_B.gs_im > forward_kinematics_B.gs_re) {
    forward_kinematics_B.gs_re = forward_kinematics_B.gs_im;
  }

  if (forward_kinematics_B.gs_re > forward_kinematics_B.scale_o) {
    forward_kinematics_B.scale_o = forward_kinematics_B.gs_re;
  }

  forward_kinematics_B.fs_re = f.re;
  forward_kinematics_B.fs_im = f.im;
  forward_kinematics_B.gs_re = g.re;
  forward_kinematics_B.gs_im = g.im;
  count = -1;
  rescaledir = 0;
  guard1 = false;
  if (forward_kinematics_B.scale_o >= 7.4428285367870146E+137) {
    do {
      count++;
      forward_kinematics_B.fs_re *= 1.3435752215134178E-138;
      forward_kinematics_B.fs_im *= 1.3435752215134178E-138;
      forward_kinematics_B.gs_re *= 1.3435752215134178E-138;
      forward_kinematics_B.gs_im *= 1.3435752215134178E-138;
      forward_kinematics_B.scale_o *= 1.3435752215134178E-138;
    } while (!(forward_kinematics_B.scale_o < 7.4428285367870146E+137));

    rescaledir = 1;
    guard1 = true;
  } else if (forward_kinematics_B.scale_o <= 1.3435752215134178E-138) {
    if ((g.re == 0.0) && (g.im == 0.0)) {
      *cs = 1.0;
      sn->re = 0.0;
      sn->im = 0.0;
      *r = f;
    } else {
      do {
        count++;
        forward_kinematics_B.fs_re *= 7.4428285367870146E+137;
        forward_kinematics_B.fs_im *= 7.4428285367870146E+137;
        forward_kinematics_B.gs_re *= 7.4428285367870146E+137;
        forward_kinematics_B.gs_im *= 7.4428285367870146E+137;
        forward_kinematics_B.scale_o *= 7.4428285367870146E+137;
      } while (!(forward_kinematics_B.scale_o > 1.3435752215134178E-138));

      rescaledir = -1;
      guard1 = true;
    }
  } else {
    guard1 = true;
  }

  if (guard1) {
    forward_kinematics_B.scale_o = forward_kinematics_B.fs_re *
      forward_kinematics_B.fs_re + forward_kinematics_B.fs_im *
      forward_kinematics_B.fs_im;
    forward_kinematics_B.g2 = forward_kinematics_B.gs_re *
      forward_kinematics_B.gs_re + forward_kinematics_B.gs_im *
      forward_kinematics_B.gs_im;
    forward_kinematics_B.x = forward_kinematics_B.g2;
    if (1.0 > forward_kinematics_B.g2) {
      forward_kinematics_B.x = 1.0;
    }

    if (forward_kinematics_B.scale_o <= forward_kinematics_B.x *
        2.0041683600089728E-292) {
      if ((f.re == 0.0) && (f.im == 0.0)) {
        *cs = 0.0;
        r->re = forward_kinematic_rt_hypotd_snf(g.re, g.im);
        r->im = 0.0;
        forward_kinematics_B.f2s = forward_kinematic_rt_hypotd_snf
          (forward_kinematics_B.gs_re, forward_kinematics_B.gs_im);
        sn->re = forward_kinematics_B.gs_re / forward_kinematics_B.f2s;
        sn->im = -forward_kinematics_B.gs_im / forward_kinematics_B.f2s;
      } else {
        forward_kinematics_B.scale_o = sqrt(forward_kinematics_B.g2);
        *cs = forward_kinematic_rt_hypotd_snf(forward_kinematics_B.fs_re,
          forward_kinematics_B.fs_im) / forward_kinematics_B.scale_o;
        if (forward_kinematics_B.di > forward_kinematics_B.f2s) {
          forward_kinematics_B.f2s = forward_kinematics_B.di;
        }

        if (forward_kinematics_B.f2s > 1.0) {
          forward_kinematics_B.f2s = forward_kinematic_rt_hypotd_snf(f.re, f.im);
          forward_kinematics_B.fs_re = f.re / forward_kinematics_B.f2s;
          forward_kinematics_B.fs_im = f.im / forward_kinematics_B.f2s;
        } else {
          forward_kinematics_B.fs_re = 7.4428285367870146E+137 * f.re;
          forward_kinematics_B.di = 7.4428285367870146E+137 * f.im;
          forward_kinematics_B.f2s = forward_kinematic_rt_hypotd_snf
            (forward_kinematics_B.fs_re, forward_kinematics_B.di);
          forward_kinematics_B.fs_re /= forward_kinematics_B.f2s;
          forward_kinematics_B.fs_im = forward_kinematics_B.di /
            forward_kinematics_B.f2s;
        }

        forward_kinematics_B.gs_re /= forward_kinematics_B.scale_o;
        forward_kinematics_B.gs_im = -forward_kinematics_B.gs_im /
          forward_kinematics_B.scale_o;
        sn->re = forward_kinematics_B.fs_re * forward_kinematics_B.gs_re -
          forward_kinematics_B.fs_im * forward_kinematics_B.gs_im;
        sn->im = forward_kinematics_B.fs_re * forward_kinematics_B.gs_im +
          forward_kinematics_B.fs_im * forward_kinematics_B.gs_re;
        r->re = (sn->re * g.re - sn->im * g.im) + *cs * f.re;
        r->im = (sn->re * g.im + sn->im * g.re) + *cs * f.im;
      }
    } else {
      forward_kinematics_B.f2s = sqrt(forward_kinematics_B.g2 /
        forward_kinematics_B.scale_o + 1.0);
      r->re = forward_kinematics_B.f2s * forward_kinematics_B.fs_re;
      r->im = forward_kinematics_B.f2s * forward_kinematics_B.fs_im;
      *cs = 1.0 / forward_kinematics_B.f2s;
      forward_kinematics_B.f2s = forward_kinematics_B.scale_o +
        forward_kinematics_B.g2;
      forward_kinematics_B.fs_re = r->re / forward_kinematics_B.f2s;
      forward_kinematics_B.f2s = r->im / forward_kinematics_B.f2s;
      sn->re = forward_kinematics_B.fs_re * forward_kinematics_B.gs_re -
        forward_kinematics_B.f2s * -forward_kinematics_B.gs_im;
      sn->im = forward_kinematics_B.fs_re * -forward_kinematics_B.gs_im +
        forward_kinematics_B.f2s * forward_kinematics_B.gs_re;
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

// Function for MATLAB Function: '<S1>/MATLAB Function'
static void forward_kinematics_xzhgeqz(creal_T A[16], int32_T ilo, int32_T ihi,
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
  forward_kinematics_B.eshift_re = 0.0;
  forward_kinematics_B.eshift_im = 0.0;
  forward_kinematics_B.ctemp.re = 0.0;
  forward_kinematics_B.ctemp.im = 0.0;
  forward_kinematics_B.anorm = forward_kinematics_xzlanhs(A, ilo, ihi);
  forward_kinematics_B.shift_re = 2.2204460492503131E-16 *
    forward_kinematics_B.anorm;
  forward_kinematics_B.b_atol = 2.2250738585072014E-308;
  if (forward_kinematics_B.shift_re > 2.2250738585072014E-308) {
    forward_kinematics_B.b_atol = forward_kinematics_B.shift_re;
  }

  forward_kinematics_B.shift_re = 2.2250738585072014E-308;
  if (forward_kinematics_B.anorm > 2.2250738585072014E-308) {
    forward_kinematics_B.shift_re = forward_kinematics_B.anorm;
  }

  forward_kinematics_B.anorm = 1.0 / forward_kinematics_B.shift_re;
  failed = true;
  forward_kinematics_B.ilast = ihi;
  while (forward_kinematics_B.ilast + 1 < 5) {
    alpha1[forward_kinematics_B.ilast] = A[(forward_kinematics_B.ilast << 2) +
      forward_kinematics_B.ilast];
    forward_kinematics_B.ilast++;
  }

  guard1 = false;
  guard2 = false;
  if (ihi >= ilo) {
    forward_kinematics_B.ifirst = ilo;
    forward_kinematics_B.istart = ilo;
    forward_kinematics_B.ilast = ihi - 1;
    forward_kinematics_B.ilastm1 = ihi - 2;
    forward_kinematics_B.iiter = 0;
    goto60 = false;
    goto70 = false;
    goto90 = false;
    forward_kinematics_B.jiter = 0;
    do {
      exitg1 = 0;
      if (forward_kinematics_B.jiter <= ((ihi - ilo) + 1) * 30 - 1) {
        if (forward_kinematics_B.ilast + 1 == ilo) {
          goto60 = true;
        } else {
          forward_kinematics_B.jp1 = (forward_kinematics_B.ilastm1 << 2) +
            forward_kinematics_B.ilast;
          if (fabs(A[forward_kinematics_B.jp1].re) + fabs
              (A[forward_kinematics_B.jp1].im) <= forward_kinematics_B.b_atol) {
            A[forward_kinematics_B.jp1].re = 0.0;
            A[forward_kinematics_B.jp1].im = 0.0;
            goto60 = true;
          } else {
            forward_kinematics_B.j = forward_kinematics_B.ilastm1;
            guard3 = false;
            exitg2 = false;
            while ((!exitg2) && (forward_kinematics_B.j + 1 >= ilo)) {
              if (forward_kinematics_B.j + 1 == ilo) {
                guard3 = true;
                exitg2 = true;
              } else {
                forward_kinematics_B.jp1 = ((forward_kinematics_B.j - 1) << 2) +
                  forward_kinematics_B.j;
                if (fabs(A[forward_kinematics_B.jp1].re) + fabs
                    (A[forward_kinematics_B.jp1].im) <=
                    forward_kinematics_B.b_atol) {
                  A[forward_kinematics_B.jp1].re = 0.0;
                  A[forward_kinematics_B.jp1].im = 0.0;
                  guard3 = true;
                  exitg2 = true;
                } else {
                  forward_kinematics_B.j--;
                  guard3 = false;
                }
              }
            }

            if (guard3) {
              forward_kinematics_B.ifirst = forward_kinematics_B.j + 1;
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
          for (forward_kinematics_B.jp1 = 0; forward_kinematics_B.jp1 < 16;
               forward_kinematics_B.jp1++) {
            Z[forward_kinematics_B.jp1].re = (rtNaN);
            Z[forward_kinematics_B.jp1].im = 0.0;
          }

          *info = 1;
          exitg1 = 1;
        } else if (goto60) {
          goto60 = false;
          alpha1[forward_kinematics_B.ilast] = A[(forward_kinematics_B.ilast <<
            2) + forward_kinematics_B.ilast];
          forward_kinematics_B.ilast = forward_kinematics_B.ilastm1;
          forward_kinematics_B.ilastm1--;
          if (forward_kinematics_B.ilast + 1 < ilo) {
            failed = false;
            guard2 = true;
            exitg1 = 1;
          } else {
            forward_kinematics_B.iiter = 0;
            forward_kinematics_B.eshift_re = 0.0;
            forward_kinematics_B.eshift_im = 0.0;
            forward_kinematics_B.jiter++;
          }
        } else {
          if (goto70) {
            goto70 = false;
            forward_kinematics_B.iiter++;
            if (forward_kinematics_B.iiter - div_nzp_s32
                (forward_kinematics_B.iiter, 10) * 10 != 0) {
              forward_kinematics_B.j = (forward_kinematics_B.ilastm1 << 2) +
                forward_kinematics_B.ilastm1;
              forward_kinematics_B.ar = A[forward_kinematics_B.j].re *
                forward_kinematics_B.anorm;
              forward_kinematics_B.ai = A[forward_kinematics_B.j].im *
                forward_kinematics_B.anorm;
              if (forward_kinematics_B.ai == 0.0) {
                forward_kinematics_B.shift_re = forward_kinematics_B.ar / 0.5;
                forward_kinematics_B.shift_im = 0.0;
              } else if (forward_kinematics_B.ar == 0.0) {
                forward_kinematics_B.shift_re = 0.0;
                forward_kinematics_B.shift_im = forward_kinematics_B.ai / 0.5;
              } else {
                forward_kinematics_B.shift_re = forward_kinematics_B.ar / 0.5;
                forward_kinematics_B.shift_im = forward_kinematics_B.ai / 0.5;
              }

              forward_kinematics_B.j = (forward_kinematics_B.ilast << 2) +
                forward_kinematics_B.ilast;
              forward_kinematics_B.ar = A[forward_kinematics_B.j].re *
                forward_kinematics_B.anorm;
              forward_kinematics_B.ai = A[forward_kinematics_B.j].im *
                forward_kinematics_B.anorm;
              if (forward_kinematics_B.ai == 0.0) {
                forward_kinematics_B.ad22.re = forward_kinematics_B.ar / 0.5;
                forward_kinematics_B.ad22.im = 0.0;
              } else if (forward_kinematics_B.ar == 0.0) {
                forward_kinematics_B.ad22.re = 0.0;
                forward_kinematics_B.ad22.im = forward_kinematics_B.ai / 0.5;
              } else {
                forward_kinematics_B.ad22.re = forward_kinematics_B.ar / 0.5;
                forward_kinematics_B.ad22.im = forward_kinematics_B.ai / 0.5;
              }

              forward_kinematics_B.t1_re = (forward_kinematics_B.shift_re +
                forward_kinematics_B.ad22.re) * 0.5;
              forward_kinematics_B.t1_im = (forward_kinematics_B.shift_im +
                forward_kinematics_B.ad22.im) * 0.5;
              forward_kinematics_B.j = (forward_kinematics_B.ilast << 2) +
                forward_kinematics_B.ilastm1;
              forward_kinematics_B.ar = A[forward_kinematics_B.j].re *
                forward_kinematics_B.anorm;
              forward_kinematics_B.ai = A[forward_kinematics_B.j].im *
                forward_kinematics_B.anorm;
              if (forward_kinematics_B.ai == 0.0) {
                forward_kinematics_B.absxr = forward_kinematics_B.ar / 0.5;
                forward_kinematics_B.absxi = 0.0;
              } else if (forward_kinematics_B.ar == 0.0) {
                forward_kinematics_B.absxr = 0.0;
                forward_kinematics_B.absxi = forward_kinematics_B.ai / 0.5;
              } else {
                forward_kinematics_B.absxr = forward_kinematics_B.ar / 0.5;
                forward_kinematics_B.absxi = forward_kinematics_B.ai / 0.5;
              }

              forward_kinematics_B.j = (forward_kinematics_B.ilastm1 << 2) +
                forward_kinematics_B.ilast;
              forward_kinematics_B.ar = A[forward_kinematics_B.j].re *
                forward_kinematics_B.anorm;
              forward_kinematics_B.ai = A[forward_kinematics_B.j].im *
                forward_kinematics_B.anorm;
              if (forward_kinematics_B.ai == 0.0) {
                forward_kinematics_B.ar /= 0.5;
                forward_kinematics_B.ai = 0.0;
              } else if (forward_kinematics_B.ar == 0.0) {
                forward_kinematics_B.ar = 0.0;
                forward_kinematics_B.ai /= 0.5;
              } else {
                forward_kinematics_B.ar /= 0.5;
                forward_kinematics_B.ai /= 0.5;
              }

              forward_kinematics_B.shift_im_j = forward_kinematics_B.shift_re *
                forward_kinematics_B.ad22.im + forward_kinematics_B.shift_im *
                forward_kinematics_B.ad22.re;
              forward_kinematics_B.shift_re = ((forward_kinematics_B.t1_re *
                forward_kinematics_B.t1_re - forward_kinematics_B.t1_im *
                forward_kinematics_B.t1_im) + (forward_kinematics_B.absxr *
                forward_kinematics_B.ar - forward_kinematics_B.absxi *
                forward_kinematics_B.ai)) - (forward_kinematics_B.shift_re *
                forward_kinematics_B.ad22.re - forward_kinematics_B.shift_im *
                forward_kinematics_B.ad22.im);
              forward_kinematics_B.shift_im = forward_kinematics_B.t1_re *
                forward_kinematics_B.t1_im;
              forward_kinematics_B.shift_im = ((forward_kinematics_B.shift_im +
                forward_kinematics_B.shift_im) + (forward_kinematics_B.absxr *
                forward_kinematics_B.ai + forward_kinematics_B.absxi *
                forward_kinematics_B.ar)) - forward_kinematics_B.shift_im_j;
              if (forward_kinematics_B.shift_im == 0.0) {
                if (forward_kinematics_B.shift_re < 0.0) {
                  forward_kinematics_B.absxr = 0.0;
                  forward_kinematics_B.absxi = sqrt
                    (-forward_kinematics_B.shift_re);
                } else {
                  forward_kinematics_B.absxr = sqrt
                    (forward_kinematics_B.shift_re);
                  forward_kinematics_B.absxi = 0.0;
                }
              } else if (forward_kinematics_B.shift_re == 0.0) {
                if (forward_kinematics_B.shift_im < 0.0) {
                  forward_kinematics_B.absxr = sqrt
                    (-forward_kinematics_B.shift_im / 2.0);
                  forward_kinematics_B.absxi = -forward_kinematics_B.absxr;
                } else {
                  forward_kinematics_B.absxr = sqrt
                    (forward_kinematics_B.shift_im / 2.0);
                  forward_kinematics_B.absxi = forward_kinematics_B.absxr;
                }
              } else if (rtIsNaN(forward_kinematics_B.shift_re)) {
                forward_kinematics_B.absxr = forward_kinematics_B.shift_re;
                forward_kinematics_B.absxi = forward_kinematics_B.shift_re;
              } else if (rtIsNaN(forward_kinematics_B.shift_im)) {
                forward_kinematics_B.absxr = forward_kinematics_B.shift_im;
                forward_kinematics_B.absxi = forward_kinematics_B.shift_im;
              } else if (rtIsInf(forward_kinematics_B.shift_im)) {
                forward_kinematics_B.absxr = fabs(forward_kinematics_B.shift_im);
                forward_kinematics_B.absxi = forward_kinematics_B.shift_im;
              } else if (rtIsInf(forward_kinematics_B.shift_re)) {
                if (forward_kinematics_B.shift_re < 0.0) {
                  forward_kinematics_B.absxr = 0.0;
                  forward_kinematics_B.absxi = forward_kinematics_B.shift_im *
                    -forward_kinematics_B.shift_re;
                } else {
                  forward_kinematics_B.absxr = forward_kinematics_B.shift_re;
                  forward_kinematics_B.absxi = 0.0;
                }
              } else {
                forward_kinematics_B.absxr = fabs(forward_kinematics_B.shift_re);
                forward_kinematics_B.absxi = fabs(forward_kinematics_B.shift_im);
                if ((forward_kinematics_B.absxr > 4.4942328371557893E+307) ||
                    (forward_kinematics_B.absxi > 4.4942328371557893E+307)) {
                  forward_kinematics_B.absxr *= 0.5;
                  forward_kinematics_B.absxi *= 0.5;
                  forward_kinematics_B.absxi = forward_kinematic_rt_hypotd_snf
                    (forward_kinematics_B.absxr, forward_kinematics_B.absxi);
                  if (forward_kinematics_B.absxi > forward_kinematics_B.absxr) {
                    forward_kinematics_B.absxr = sqrt(forward_kinematics_B.absxr
                      / forward_kinematics_B.absxi + 1.0) * sqrt
                      (forward_kinematics_B.absxi);
                  } else {
                    forward_kinematics_B.absxr = sqrt(forward_kinematics_B.absxi)
                      * 1.4142135623730951;
                  }
                } else {
                  forward_kinematics_B.absxr = sqrt
                    ((forward_kinematic_rt_hypotd_snf(forward_kinematics_B.absxr,
                       forward_kinematics_B.absxi) + forward_kinematics_B.absxr)
                     * 0.5);
                }

                if (forward_kinematics_B.shift_re > 0.0) {
                  forward_kinematics_B.absxi = forward_kinematics_B.shift_im /
                    forward_kinematics_B.absxr * 0.5;
                } else {
                  if (forward_kinematics_B.shift_im < 0.0) {
                    forward_kinematics_B.absxi = -forward_kinematics_B.absxr;
                  } else {
                    forward_kinematics_B.absxi = forward_kinematics_B.absxr;
                  }

                  forward_kinematics_B.absxr = forward_kinematics_B.shift_im /
                    forward_kinematics_B.absxi * 0.5;
                }
              }

              if ((forward_kinematics_B.t1_re - forward_kinematics_B.ad22.re) *
                  forward_kinematics_B.absxr + (forward_kinematics_B.t1_im -
                   forward_kinematics_B.ad22.im) * forward_kinematics_B.absxi <=
                  0.0) {
                forward_kinematics_B.shift_re = forward_kinematics_B.t1_re +
                  forward_kinematics_B.absxr;
                forward_kinematics_B.shift_im = forward_kinematics_B.t1_im +
                  forward_kinematics_B.absxi;
              } else {
                forward_kinematics_B.shift_re = forward_kinematics_B.t1_re -
                  forward_kinematics_B.absxr;
                forward_kinematics_B.shift_im = forward_kinematics_B.t1_im -
                  forward_kinematics_B.absxi;
              }
            } else {
              forward_kinematics_B.j = (forward_kinematics_B.ilastm1 << 2) +
                forward_kinematics_B.ilast;
              forward_kinematics_B.ar = A[forward_kinematics_B.j].re *
                forward_kinematics_B.anorm;
              forward_kinematics_B.ai = A[forward_kinematics_B.j].im *
                forward_kinematics_B.anorm;
              if (forward_kinematics_B.ai == 0.0) {
                forward_kinematics_B.absxr = forward_kinematics_B.ar / 0.5;
                forward_kinematics_B.absxi = 0.0;
              } else if (forward_kinematics_B.ar == 0.0) {
                forward_kinematics_B.absxr = 0.0;
                forward_kinematics_B.absxi = forward_kinematics_B.ai / 0.5;
              } else {
                forward_kinematics_B.absxr = forward_kinematics_B.ar / 0.5;
                forward_kinematics_B.absxi = forward_kinematics_B.ai / 0.5;
              }

              forward_kinematics_B.eshift_re += forward_kinematics_B.absxr;
              forward_kinematics_B.eshift_im += forward_kinematics_B.absxi;
              forward_kinematics_B.shift_re = forward_kinematics_B.eshift_re;
              forward_kinematics_B.shift_im = forward_kinematics_B.eshift_im;
            }

            forward_kinematics_B.j = forward_kinematics_B.ilastm1;
            forward_kinematics_B.jp1 = forward_kinematics_B.ilastm1 + 1;
            exitg2 = false;
            while ((!exitg2) && (forward_kinematics_B.j + 1 >
                                 forward_kinematics_B.ifirst)) {
              forward_kinematics_B.istart = forward_kinematics_B.j + 1;
              forward_kinematics_B.ctemp_tmp_tmp = forward_kinematics_B.j << 2;
              forward_kinematics_B.ctemp_tmp =
                forward_kinematics_B.ctemp_tmp_tmp + forward_kinematics_B.j;
              forward_kinematics_B.ctemp.re = A[forward_kinematics_B.ctemp_tmp].
                re * forward_kinematics_B.anorm - forward_kinematics_B.shift_re *
                0.5;
              forward_kinematics_B.ctemp.im = A[forward_kinematics_B.ctemp_tmp].
                im * forward_kinematics_B.anorm - forward_kinematics_B.shift_im *
                0.5;
              forward_kinematics_B.t1_re = fabs(forward_kinematics_B.ctemp.re) +
                fabs(forward_kinematics_B.ctemp.im);
              forward_kinematics_B.jp1 += forward_kinematics_B.ctemp_tmp_tmp;
              forward_kinematics_B.t1_im = (fabs(A[forward_kinematics_B.jp1].re)
                + fabs(A[forward_kinematics_B.jp1].im)) *
                forward_kinematics_B.anorm;
              forward_kinematics_B.absxr = forward_kinematics_B.t1_re;
              if (forward_kinematics_B.t1_im > forward_kinematics_B.t1_re) {
                forward_kinematics_B.absxr = forward_kinematics_B.t1_im;
              }

              if ((forward_kinematics_B.absxr < 1.0) &&
                  (forward_kinematics_B.absxr != 0.0)) {
                forward_kinematics_B.t1_re /= forward_kinematics_B.absxr;
                forward_kinematics_B.t1_im /= forward_kinematics_B.absxr;
              }

              forward_kinematics_B.jp1 = ((forward_kinematics_B.j - 1) << 2) +
                forward_kinematics_B.j;
              if ((fabs(A[forward_kinematics_B.jp1].re) + fabs
                   (A[forward_kinematics_B.jp1].im)) *
                  forward_kinematics_B.t1_im <= forward_kinematics_B.t1_re *
                  forward_kinematics_B.b_atol) {
                goto90 = true;
                exitg2 = true;
              } else {
                forward_kinematics_B.jp1 = forward_kinematics_B.j;
                forward_kinematics_B.j--;
              }
            }

            if (!goto90) {
              forward_kinematics_B.istart = forward_kinematics_B.ifirst;
              forward_kinematics_B.ctemp_tmp = (((forward_kinematics_B.ifirst -
                1) << 2) + forward_kinematics_B.ifirst) - 1;
              forward_kinematics_B.ctemp.re = A[forward_kinematics_B.ctemp_tmp].
                re * forward_kinematics_B.anorm - forward_kinematics_B.shift_re *
                0.5;
              forward_kinematics_B.ctemp.im = A[forward_kinematics_B.ctemp_tmp].
                im * forward_kinematics_B.anorm - forward_kinematics_B.shift_im *
                0.5;
            }

            goto90 = false;
            forward_kinematics_B.j = ((forward_kinematics_B.istart - 1) << 2) +
              forward_kinematics_B.istart;
            forward_kinematics_B.ascale.re = A[forward_kinematics_B.j].re *
              forward_kinematics_B.anorm;
            forward_kinematics_B.ascale.im = A[forward_kinematics_B.j].im *
              forward_kinematics_B.anorm;
            forward_kinematics_xzlartg_k(forward_kinematics_B.ctemp,
              forward_kinematics_B.ascale, &forward_kinematics_B.t1_re,
              &forward_kinematics_B.ad22);
            forward_kinematics_B.j = forward_kinematics_B.istart;
            forward_kinematics_B.jp1 = forward_kinematics_B.istart - 2;
            while (forward_kinematics_B.j < forward_kinematics_B.ilast + 1) {
              if (forward_kinematics_B.j > forward_kinematics_B.istart) {
                forward_kinematics_xzlartg(A[(forward_kinematics_B.j +
                  (forward_kinematics_B.jp1 << 2)) - 1],
                  A[forward_kinematics_B.j + (forward_kinematics_B.jp1 << 2)],
                  &forward_kinematics_B.t1_re, &forward_kinematics_B.ad22, &A
                  [(forward_kinematics_B.j + (forward_kinematics_B.jp1 << 2)) -
                  1]);
                forward_kinematics_B.jp1 = forward_kinematics_B.j +
                  (forward_kinematics_B.jp1 << 2);
                A[forward_kinematics_B.jp1].re = 0.0;
                A[forward_kinematics_B.jp1].im = 0.0;
              }

              forward_kinematics_B.ctemp_tmp = forward_kinematics_B.j - 1;
              while (forward_kinematics_B.ctemp_tmp + 1 < 5) {
                forward_kinematics_B.jp1 = (forward_kinematics_B.ctemp_tmp << 2)
                  + forward_kinematics_B.j;
                forward_kinematics_B.ctemp_tmp_tmp = forward_kinematics_B.jp1 -
                  1;
                forward_kinematics_B.shift_re =
                  A[forward_kinematics_B.ctemp_tmp_tmp].re *
                  forward_kinematics_B.t1_re + (A[forward_kinematics_B.jp1].re *
                  forward_kinematics_B.ad22.re - A[forward_kinematics_B.jp1].im *
                  forward_kinematics_B.ad22.im);
                forward_kinematics_B.shift_im =
                  A[forward_kinematics_B.ctemp_tmp_tmp].im *
                  forward_kinematics_B.t1_re + (A[forward_kinematics_B.jp1].im *
                  forward_kinematics_B.ad22.re + A[forward_kinematics_B.jp1].re *
                  forward_kinematics_B.ad22.im);
                forward_kinematics_B.t1_im =
                  A[forward_kinematics_B.ctemp_tmp_tmp].im;
                forward_kinematics_B.absxr =
                  A[forward_kinematics_B.ctemp_tmp_tmp].re;
                A[forward_kinematics_B.jp1].re = A[forward_kinematics_B.jp1].re *
                  forward_kinematics_B.t1_re -
                  (A[forward_kinematics_B.ctemp_tmp_tmp].re *
                   forward_kinematics_B.ad22.re +
                   A[forward_kinematics_B.ctemp_tmp_tmp].im *
                   forward_kinematics_B.ad22.im);
                A[forward_kinematics_B.jp1].im = A[forward_kinematics_B.jp1].im *
                  forward_kinematics_B.t1_re - (forward_kinematics_B.ad22.re *
                  forward_kinematics_B.t1_im - forward_kinematics_B.ad22.im *
                  forward_kinematics_B.absxr);
                A[forward_kinematics_B.ctemp_tmp_tmp].re =
                  forward_kinematics_B.shift_re;
                A[forward_kinematics_B.ctemp_tmp_tmp].im =
                  forward_kinematics_B.shift_im;
                forward_kinematics_B.ctemp_tmp++;
              }

              forward_kinematics_B.ad22.re = -forward_kinematics_B.ad22.re;
              forward_kinematics_B.ad22.im = -forward_kinematics_B.ad22.im;
              forward_kinematics_B.ctemp_tmp = forward_kinematics_B.j;
              if (forward_kinematics_B.ilast + 1 < forward_kinematics_B.j + 2) {
                forward_kinematics_B.ctemp_tmp = forward_kinematics_B.ilast - 1;
              }

              forward_kinematics_B.i_l = 0;
              while (forward_kinematics_B.i_l + 1 <=
                     forward_kinematics_B.ctemp_tmp + 2) {
                forward_kinematics_B.jp1 = ((forward_kinematics_B.j - 1) << 2) +
                  forward_kinematics_B.i_l;
                forward_kinematics_B.ctemp_tmp_tmp = (forward_kinematics_B.j <<
                  2) + forward_kinematics_B.i_l;
                forward_kinematics_B.shift_re = (A[forward_kinematics_B.jp1].re *
                  forward_kinematics_B.ad22.re - A[forward_kinematics_B.jp1].im *
                  forward_kinematics_B.ad22.im) +
                  A[forward_kinematics_B.ctemp_tmp_tmp].re *
                  forward_kinematics_B.t1_re;
                forward_kinematics_B.shift_im = (A[forward_kinematics_B.jp1].im *
                  forward_kinematics_B.ad22.re + A[forward_kinematics_B.jp1].re *
                  forward_kinematics_B.ad22.im) +
                  A[forward_kinematics_B.ctemp_tmp_tmp].im *
                  forward_kinematics_B.t1_re;
                forward_kinematics_B.t1_im =
                  A[forward_kinematics_B.ctemp_tmp_tmp].im;
                forward_kinematics_B.absxr =
                  A[forward_kinematics_B.ctemp_tmp_tmp].re;
                A[forward_kinematics_B.jp1].re = A[forward_kinematics_B.jp1].re *
                  forward_kinematics_B.t1_re -
                  (A[forward_kinematics_B.ctemp_tmp_tmp].re *
                   forward_kinematics_B.ad22.re +
                   A[forward_kinematics_B.ctemp_tmp_tmp].im *
                   forward_kinematics_B.ad22.im);
                A[forward_kinematics_B.jp1].im = A[forward_kinematics_B.jp1].im *
                  forward_kinematics_B.t1_re - (forward_kinematics_B.ad22.re *
                  forward_kinematics_B.t1_im - forward_kinematics_B.ad22.im *
                  forward_kinematics_B.absxr);
                A[forward_kinematics_B.ctemp_tmp_tmp].re =
                  forward_kinematics_B.shift_re;
                A[forward_kinematics_B.ctemp_tmp_tmp].im =
                  forward_kinematics_B.shift_im;
                forward_kinematics_B.i_l++;
              }

              forward_kinematics_B.jp1 = (forward_kinematics_B.j - 1) << 2;
              forward_kinematics_B.ctemp_tmp_tmp = forward_kinematics_B.j << 2;
              forward_kinematics_B.shift_re = (Z[forward_kinematics_B.jp1].re *
                forward_kinematics_B.ad22.re - Z[forward_kinematics_B.jp1].im *
                forward_kinematics_B.ad22.im) +
                Z[forward_kinematics_B.ctemp_tmp_tmp].re *
                forward_kinematics_B.t1_re;
              forward_kinematics_B.shift_im = (Z[forward_kinematics_B.jp1].im *
                forward_kinematics_B.ad22.re + Z[forward_kinematics_B.jp1].re *
                forward_kinematics_B.ad22.im) +
                Z[forward_kinematics_B.ctemp_tmp_tmp].im *
                forward_kinematics_B.t1_re;
              forward_kinematics_B.t1_im = Z[forward_kinematics_B.ctemp_tmp_tmp]
                .im;
              forward_kinematics_B.absxr = Z[forward_kinematics_B.ctemp_tmp_tmp]
                .re;
              Z[forward_kinematics_B.jp1].re = Z[forward_kinematics_B.jp1].re *
                forward_kinematics_B.t1_re -
                (Z[forward_kinematics_B.ctemp_tmp_tmp].re *
                 forward_kinematics_B.ad22.re +
                 Z[forward_kinematics_B.ctemp_tmp_tmp].im *
                 forward_kinematics_B.ad22.im);
              Z[forward_kinematics_B.jp1].im = Z[forward_kinematics_B.jp1].im *
                forward_kinematics_B.t1_re - (forward_kinematics_B.ad22.re *
                forward_kinematics_B.t1_im - forward_kinematics_B.ad22.im *
                forward_kinematics_B.absxr);
              Z[forward_kinematics_B.ctemp_tmp_tmp].re =
                forward_kinematics_B.shift_re;
              Z[forward_kinematics_B.ctemp_tmp_tmp].im =
                forward_kinematics_B.shift_im;
              forward_kinematics_B.ctemp_tmp = forward_kinematics_B.jp1 + 1;
              forward_kinematics_B.i_l = forward_kinematics_B.ctemp_tmp_tmp + 1;
              forward_kinematics_B.shift_re = (Z[forward_kinematics_B.ctemp_tmp]
                .re * forward_kinematics_B.ad22.re -
                Z[forward_kinematics_B.ctemp_tmp].im *
                forward_kinematics_B.ad22.im) + Z[forward_kinematics_B.i_l].re *
                forward_kinematics_B.t1_re;
              forward_kinematics_B.shift_im = (Z[forward_kinematics_B.ctemp_tmp]
                .im * forward_kinematics_B.ad22.re +
                Z[forward_kinematics_B.ctemp_tmp].re *
                forward_kinematics_B.ad22.im) + Z[forward_kinematics_B.i_l].im *
                forward_kinematics_B.t1_re;
              forward_kinematics_B.t1_im = Z[forward_kinematics_B.i_l].im;
              forward_kinematics_B.absxr = Z[forward_kinematics_B.i_l].re;
              Z[forward_kinematics_B.ctemp_tmp].re =
                Z[forward_kinematics_B.ctemp_tmp].re *
                forward_kinematics_B.t1_re - (Z[forward_kinematics_B.i_l].re *
                forward_kinematics_B.ad22.re + Z[forward_kinematics_B.i_l].im *
                forward_kinematics_B.ad22.im);
              Z[forward_kinematics_B.ctemp_tmp].im =
                Z[forward_kinematics_B.ctemp_tmp].im *
                forward_kinematics_B.t1_re - (forward_kinematics_B.ad22.re *
                forward_kinematics_B.t1_im - forward_kinematics_B.ad22.im *
                forward_kinematics_B.absxr);
              Z[forward_kinematics_B.i_l].re = forward_kinematics_B.shift_re;
              Z[forward_kinematics_B.i_l].im = forward_kinematics_B.shift_im;
              forward_kinematics_B.ctemp_tmp = forward_kinematics_B.jp1 + 2;
              forward_kinematics_B.i_l = forward_kinematics_B.ctemp_tmp_tmp + 2;
              forward_kinematics_B.shift_re = (Z[forward_kinematics_B.ctemp_tmp]
                .re * forward_kinematics_B.ad22.re -
                Z[forward_kinematics_B.ctemp_tmp].im *
                forward_kinematics_B.ad22.im) + Z[forward_kinematics_B.i_l].re *
                forward_kinematics_B.t1_re;
              forward_kinematics_B.shift_im = (Z[forward_kinematics_B.ctemp_tmp]
                .im * forward_kinematics_B.ad22.re +
                Z[forward_kinematics_B.ctemp_tmp].re *
                forward_kinematics_B.ad22.im) + Z[forward_kinematics_B.i_l].im *
                forward_kinematics_B.t1_re;
              forward_kinematics_B.t1_im = Z[forward_kinematics_B.i_l].im;
              forward_kinematics_B.absxr = Z[forward_kinematics_B.i_l].re;
              Z[forward_kinematics_B.ctemp_tmp].re =
                Z[forward_kinematics_B.ctemp_tmp].re *
                forward_kinematics_B.t1_re - (Z[forward_kinematics_B.i_l].re *
                forward_kinematics_B.ad22.re + Z[forward_kinematics_B.i_l].im *
                forward_kinematics_B.ad22.im);
              Z[forward_kinematics_B.ctemp_tmp].im =
                Z[forward_kinematics_B.ctemp_tmp].im *
                forward_kinematics_B.t1_re - (forward_kinematics_B.ad22.re *
                forward_kinematics_B.t1_im - forward_kinematics_B.ad22.im *
                forward_kinematics_B.absxr);
              Z[forward_kinematics_B.i_l].re = forward_kinematics_B.shift_re;
              Z[forward_kinematics_B.i_l].im = forward_kinematics_B.shift_im;
              forward_kinematics_B.jp1 += 3;
              forward_kinematics_B.ctemp_tmp_tmp += 3;
              forward_kinematics_B.shift_re = (Z[forward_kinematics_B.jp1].re *
                forward_kinematics_B.ad22.re - Z[forward_kinematics_B.jp1].im *
                forward_kinematics_B.ad22.im) +
                Z[forward_kinematics_B.ctemp_tmp_tmp].re *
                forward_kinematics_B.t1_re;
              forward_kinematics_B.shift_im = (Z[forward_kinematics_B.jp1].im *
                forward_kinematics_B.ad22.re + Z[forward_kinematics_B.jp1].re *
                forward_kinematics_B.ad22.im) +
                Z[forward_kinematics_B.ctemp_tmp_tmp].im *
                forward_kinematics_B.t1_re;
              forward_kinematics_B.t1_im = Z[forward_kinematics_B.ctemp_tmp_tmp]
                .im;
              forward_kinematics_B.absxr = Z[forward_kinematics_B.ctemp_tmp_tmp]
                .re;
              Z[forward_kinematics_B.jp1].re = Z[forward_kinematics_B.jp1].re *
                forward_kinematics_B.t1_re -
                (Z[forward_kinematics_B.ctemp_tmp_tmp].re *
                 forward_kinematics_B.ad22.re +
                 Z[forward_kinematics_B.ctemp_tmp_tmp].im *
                 forward_kinematics_B.ad22.im);
              Z[forward_kinematics_B.jp1].im = Z[forward_kinematics_B.jp1].im *
                forward_kinematics_B.t1_re - (forward_kinematics_B.ad22.re *
                forward_kinematics_B.t1_im - forward_kinematics_B.ad22.im *
                forward_kinematics_B.absxr);
              Z[forward_kinematics_B.ctemp_tmp_tmp].re =
                forward_kinematics_B.shift_re;
              Z[forward_kinematics_B.ctemp_tmp_tmp].im =
                forward_kinematics_B.shift_im;
              forward_kinematics_B.jp1 = forward_kinematics_B.j - 1;
              forward_kinematics_B.j++;
            }
          }

          forward_kinematics_B.jiter++;
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
      *info = forward_kinematics_B.ilast + 1;
      forward_kinematics_B.ifirst = 0;
      while (forward_kinematics_B.ifirst <= forward_kinematics_B.ilast) {
        alpha1[forward_kinematics_B.ifirst].re = (rtNaN);
        alpha1[forward_kinematics_B.ifirst].im = 0.0;
        beta1[forward_kinematics_B.ifirst].re = (rtNaN);
        beta1[forward_kinematics_B.ifirst].im = 0.0;
        forward_kinematics_B.ifirst++;
      }

      for (forward_kinematics_B.jp1 = 0; forward_kinematics_B.jp1 < 16;
           forward_kinematics_B.jp1++) {
        Z[forward_kinematics_B.jp1].re = (rtNaN);
        Z[forward_kinematics_B.jp1].im = 0.0;
      }
    } else {
      guard1 = true;
    }
  }

  if (guard1) {
    forward_kinematics_B.ilast = 0;
    while (forward_kinematics_B.ilast <= ilo - 2) {
      alpha1[forward_kinematics_B.ilast] = A[(forward_kinematics_B.ilast << 2) +
        forward_kinematics_B.ilast];
      forward_kinematics_B.ilast++;
    }
  }
}

// Function for MATLAB Function: '<S1>/MATLAB Function'
static void forward_kinematics_xztgevc(const creal_T A[16], creal_T V[16])
{
  boolean_T lscalea;
  boolean_T lscaleb;
  int32_T work2_idx_1_re_tmp;
  int32_T d_re_tmp_tmp;
  forward_kinematics_B.rworka[0] = 0.0;
  forward_kinematics_B.rworka[2] = 0.0;
  forward_kinematics_B.rworka[3] = 0.0;
  forward_kinematics_B.anorm_c = fabs(A[0].re) + fabs(A[0].im);
  forward_kinematics_B.rworka[1] = fabs(A[4].re) + fabs(A[4].im);
  forward_kinematics_B.ascale_c = (fabs(A[5].re) + fabs(A[5].im)) +
    forward_kinematics_B.rworka[1];
  if (forward_kinematics_B.ascale_c > forward_kinematics_B.anorm_c) {
    forward_kinematics_B.anorm_c = forward_kinematics_B.ascale_c;
  }

  forward_kinematics_B.i_ol = 0;
  while (forward_kinematics_B.i_ol <= 1) {
    forward_kinematics_B.rworka[2] += fabs(A[forward_kinematics_B.i_ol + 8].re)
      + fabs(A[forward_kinematics_B.i_ol + 8].im);
    forward_kinematics_B.i_ol++;
  }

  forward_kinematics_B.ascale_c = (fabs(A[10].re) + fabs(A[10].im)) +
    forward_kinematics_B.rworka[2];
  if (forward_kinematics_B.ascale_c > forward_kinematics_B.anorm_c) {
    forward_kinematics_B.anorm_c = forward_kinematics_B.ascale_c;
  }

  forward_kinematics_B.i_ol = 0;
  while (forward_kinematics_B.i_ol <= 2) {
    forward_kinematics_B.rworka[3] += fabs(A[forward_kinematics_B.i_ol + 12].re)
      + fabs(A[forward_kinematics_B.i_ol + 12].im);
    forward_kinematics_B.i_ol++;
  }

  forward_kinematics_B.ascale_c = (fabs(A[15].re) + fabs(A[15].im)) +
    forward_kinematics_B.rworka[3];
  if (forward_kinematics_B.ascale_c > forward_kinematics_B.anorm_c) {
    forward_kinematics_B.anorm_c = forward_kinematics_B.ascale_c;
  }

  forward_kinematics_B.ascale_c = forward_kinematics_B.anorm_c;
  if (2.2250738585072014E-308 > forward_kinematics_B.anorm_c) {
    forward_kinematics_B.ascale_c = 2.2250738585072014E-308;
  }

  forward_kinematics_B.ascale_c = 1.0 / forward_kinematics_B.ascale_c;
  for (forward_kinematics_B.i_ol = 0; forward_kinematics_B.i_ol < 4;
       forward_kinematics_B.i_ol++) {
    forward_kinematics_B.c_x_tmp_tmp = (3 - forward_kinematics_B.i_ol) << 2;
    forward_kinematics_B.c_x_tmp = (forward_kinematics_B.c_x_tmp_tmp -
      forward_kinematics_B.i_ol) + 3;
    forward_kinematics_B.salpha_re = (fabs(A[forward_kinematics_B.c_x_tmp].re) +
      fabs(A[forward_kinematics_B.c_x_tmp].im)) * forward_kinematics_B.ascale_c;
    if (1.0 > forward_kinematics_B.salpha_re) {
      forward_kinematics_B.salpha_re = 1.0;
    }

    forward_kinematics_B.temp = 1.0 / forward_kinematics_B.salpha_re;
    forward_kinematics_B.salpha_re = A[forward_kinematics_B.c_x_tmp].re *
      forward_kinematics_B.temp * forward_kinematics_B.ascale_c;
    forward_kinematics_B.salpha_im = A[forward_kinematics_B.c_x_tmp].im *
      forward_kinematics_B.temp * forward_kinematics_B.ascale_c;
    forward_kinematics_B.acoeff = forward_kinematics_B.temp *
      forward_kinematics_B.ascale_c;
    lscalea = ((forward_kinematics_B.temp >= 2.2250738585072014E-308) &&
               (forward_kinematics_B.acoeff < 4.0083367200179456E-292));
    forward_kinematics_B.dmin = fabs(forward_kinematics_B.salpha_re) + fabs
      (forward_kinematics_B.salpha_im);
    lscaleb = ((forward_kinematics_B.dmin >= 2.2250738585072014E-308) &&
               (forward_kinematics_B.dmin < 4.0083367200179456E-292));
    forward_kinematics_B.scale_p = 1.0;
    if (lscalea) {
      forward_kinematics_B.scale_p = forward_kinematics_B.anorm_c;
      if (2.4948003869184E+291 < forward_kinematics_B.anorm_c) {
        forward_kinematics_B.scale_p = 2.4948003869184E+291;
      }

      forward_kinematics_B.scale_p *= 4.0083367200179456E-292 /
        forward_kinematics_B.temp;
    }

    if (lscaleb) {
      forward_kinematics_B.work2_idx_2_im = 4.0083367200179456E-292 /
        forward_kinematics_B.dmin;
      if (forward_kinematics_B.work2_idx_2_im > forward_kinematics_B.scale_p) {
        forward_kinematics_B.scale_p = forward_kinematics_B.work2_idx_2_im;
      }
    }

    if (lscalea || lscaleb) {
      forward_kinematics_B.work2_idx_2_im = forward_kinematics_B.acoeff;
      if (1.0 > forward_kinematics_B.acoeff) {
        forward_kinematics_B.work2_idx_2_im = 1.0;
      }

      if (forward_kinematics_B.dmin > forward_kinematics_B.work2_idx_2_im) {
        forward_kinematics_B.work2_idx_2_im = forward_kinematics_B.dmin;
      }

      forward_kinematics_B.dmin = 1.0 / (2.2250738585072014E-308 *
        forward_kinematics_B.work2_idx_2_im);
      if (forward_kinematics_B.dmin < forward_kinematics_B.scale_p) {
        forward_kinematics_B.scale_p = forward_kinematics_B.dmin;
      }

      if (lscalea) {
        forward_kinematics_B.acoeff = forward_kinematics_B.scale_p *
          forward_kinematics_B.temp * forward_kinematics_B.ascale_c;
      } else {
        forward_kinematics_B.acoeff *= forward_kinematics_B.scale_p;
      }

      forward_kinematics_B.salpha_re *= forward_kinematics_B.scale_p;
      forward_kinematics_B.salpha_im *= forward_kinematics_B.scale_p;
    }

    memset(&forward_kinematics_B.work1[0], 0, sizeof(creal_T) << 2U);
    forward_kinematics_B.work1[3 - forward_kinematics_B.i_ol].re = 1.0;
    forward_kinematics_B.work1[3 - forward_kinematics_B.i_ol].im = 0.0;
    forward_kinematics_B.dmin = 2.2204460492503131E-16 *
      forward_kinematics_B.acoeff * forward_kinematics_B.anorm_c;
    forward_kinematics_B.temp = (fabs(forward_kinematics_B.salpha_re) + fabs
      (forward_kinematics_B.salpha_im)) * 2.2204460492503131E-16;
    if (forward_kinematics_B.temp > forward_kinematics_B.dmin) {
      forward_kinematics_B.dmin = forward_kinematics_B.temp;
    }

    if (2.2250738585072014E-308 > forward_kinematics_B.dmin) {
      forward_kinematics_B.dmin = 2.2250738585072014E-308;
    }

    forward_kinematics_B.c_x_tmp = 0;
    while (forward_kinematics_B.c_x_tmp <= 2 - forward_kinematics_B.i_ol) {
      forward_kinematics_B.d_re_tmp = forward_kinematics_B.c_x_tmp_tmp +
        forward_kinematics_B.c_x_tmp;
      forward_kinematics_B.work1[forward_kinematics_B.c_x_tmp].re =
        A[forward_kinematics_B.d_re_tmp].re * forward_kinematics_B.acoeff;
      forward_kinematics_B.work1[forward_kinematics_B.c_x_tmp].im =
        A[forward_kinematics_B.d_re_tmp].im * forward_kinematics_B.acoeff;
      forward_kinematics_B.c_x_tmp++;
    }

    forward_kinematics_B.work1[3 - forward_kinematics_B.i_ol].re = 1.0;
    forward_kinematics_B.work1[3 - forward_kinematics_B.i_ol].im = 0.0;
    forward_kinematics_B.c_x_tmp = static_cast<int32_T>(((-1.0 - ((-static_cast<
      real_T>(forward_kinematics_B.i_ol) + 4.0) - 1.0)) + 1.0) / -1.0);
    forward_kinematics_B.c_j_b = 0;
    while (forward_kinematics_B.c_j_b <= forward_kinematics_B.c_x_tmp - 1) {
      work2_idx_1_re_tmp = 2 - (forward_kinematics_B.i_ol +
        forward_kinematics_B.c_j_b);
      d_re_tmp_tmp = work2_idx_1_re_tmp << 2;
      forward_kinematics_B.d_re_tmp = d_re_tmp_tmp + work2_idx_1_re_tmp;
      forward_kinematics_B.work2_idx_3_re = A[forward_kinematics_B.d_re_tmp].re *
        forward_kinematics_B.acoeff - forward_kinematics_B.salpha_re;
      forward_kinematics_B.scale_p = A[forward_kinematics_B.d_re_tmp].im *
        forward_kinematics_B.acoeff - forward_kinematics_B.salpha_im;
      if (fabs(forward_kinematics_B.work2_idx_3_re) + fabs
          (forward_kinematics_B.scale_p) <= forward_kinematics_B.dmin) {
        forward_kinematics_B.work2_idx_3_re = forward_kinematics_B.dmin;
        forward_kinematics_B.scale_p = 0.0;
      }

      forward_kinematics_B.work2_idx_2_im = fabs
        (forward_kinematics_B.work2_idx_3_re);
      forward_kinematics_B.work2_idx_3_im = fabs(forward_kinematics_B.scale_p);
      forward_kinematics_B.temp = forward_kinematics_B.work2_idx_2_im +
        forward_kinematics_B.work2_idx_3_im;
      if (forward_kinematics_B.temp < 1.0) {
        forward_kinematics_B.f_y = fabs
          (forward_kinematics_B.work1[work2_idx_1_re_tmp].re) + fabs
          (forward_kinematics_B.work1[work2_idx_1_re_tmp].im);
        if (forward_kinematics_B.f_y >= forward_kinematics_B.temp *
            1.1235582092889474E+307) {
          forward_kinematics_B.temp = 1.0 / forward_kinematics_B.f_y;
          forward_kinematics_B.d_re_tmp = 0;
          while (forward_kinematics_B.d_re_tmp <= 3 - forward_kinematics_B.i_ol)
          {
            forward_kinematics_B.work1[forward_kinematics_B.d_re_tmp].re *=
              forward_kinematics_B.temp;
            forward_kinematics_B.work1[forward_kinematics_B.d_re_tmp].im *=
              forward_kinematics_B.temp;
            forward_kinematics_B.d_re_tmp++;
          }
        }
      }

      if (forward_kinematics_B.scale_p == 0.0) {
        if (-forward_kinematics_B.work1[work2_idx_1_re_tmp].im == 0.0) {
          forward_kinematics_B.temp =
            -forward_kinematics_B.work1[work2_idx_1_re_tmp].re /
            forward_kinematics_B.work2_idx_3_re;
          forward_kinematics_B.scale_p = 0.0;
        } else if (-forward_kinematics_B.work1[work2_idx_1_re_tmp].re == 0.0) {
          forward_kinematics_B.temp = 0.0;
          forward_kinematics_B.scale_p =
            -forward_kinematics_B.work1[work2_idx_1_re_tmp].im /
            forward_kinematics_B.work2_idx_3_re;
        } else {
          forward_kinematics_B.temp =
            -forward_kinematics_B.work1[work2_idx_1_re_tmp].re /
            forward_kinematics_B.work2_idx_3_re;
          forward_kinematics_B.scale_p =
            -forward_kinematics_B.work1[work2_idx_1_re_tmp].im /
            forward_kinematics_B.work2_idx_3_re;
        }
      } else if (forward_kinematics_B.work2_idx_3_re == 0.0) {
        if (-forward_kinematics_B.work1[work2_idx_1_re_tmp].re == 0.0) {
          forward_kinematics_B.temp =
            -forward_kinematics_B.work1[work2_idx_1_re_tmp].im /
            forward_kinematics_B.scale_p;
          forward_kinematics_B.scale_p = 0.0;
        } else if (-forward_kinematics_B.work1[work2_idx_1_re_tmp].im == 0.0) {
          forward_kinematics_B.temp = 0.0;
          forward_kinematics_B.scale_p =
            -(-forward_kinematics_B.work1[work2_idx_1_re_tmp].re /
              forward_kinematics_B.scale_p);
        } else {
          forward_kinematics_B.temp =
            -forward_kinematics_B.work1[work2_idx_1_re_tmp].im /
            forward_kinematics_B.scale_p;
          forward_kinematics_B.scale_p =
            -(-forward_kinematics_B.work1[work2_idx_1_re_tmp].re /
              forward_kinematics_B.scale_p);
        }
      } else if (forward_kinematics_B.work2_idx_2_im >
                 forward_kinematics_B.work2_idx_3_im) {
        forward_kinematics_B.work2_idx_2_im = forward_kinematics_B.scale_p /
          forward_kinematics_B.work2_idx_3_re;
        forward_kinematics_B.scale_p = forward_kinematics_B.work2_idx_2_im *
          forward_kinematics_B.scale_p + forward_kinematics_B.work2_idx_3_re;
        forward_kinematics_B.temp = (forward_kinematics_B.work2_idx_2_im *
          -forward_kinematics_B.work1[work2_idx_1_re_tmp].im +
          -forward_kinematics_B.work1[work2_idx_1_re_tmp].re) /
          forward_kinematics_B.scale_p;
        forward_kinematics_B.scale_p =
          (-forward_kinematics_B.work1[work2_idx_1_re_tmp].im -
           forward_kinematics_B.work2_idx_2_im *
           -forward_kinematics_B.work1[work2_idx_1_re_tmp].re) /
          forward_kinematics_B.scale_p;
      } else if (forward_kinematics_B.work2_idx_3_im ==
                 forward_kinematics_B.work2_idx_2_im) {
        forward_kinematics_B.work2_idx_3_re =
          forward_kinematics_B.work2_idx_3_re > 0.0 ? 0.5 : -0.5;
        forward_kinematics_B.scale_p = forward_kinematics_B.scale_p > 0.0 ? 0.5 :
          -0.5;
        forward_kinematics_B.temp =
          (-forward_kinematics_B.work1[work2_idx_1_re_tmp].re *
           forward_kinematics_B.work2_idx_3_re +
           -forward_kinematics_B.work1[work2_idx_1_re_tmp].im *
           forward_kinematics_B.scale_p) / forward_kinematics_B.work2_idx_2_im;
        forward_kinematics_B.scale_p =
          (-forward_kinematics_B.work1[work2_idx_1_re_tmp].im *
           forward_kinematics_B.work2_idx_3_re -
           -forward_kinematics_B.work1[work2_idx_1_re_tmp].re *
           forward_kinematics_B.scale_p) / forward_kinematics_B.work2_idx_2_im;
      } else {
        forward_kinematics_B.work2_idx_2_im =
          forward_kinematics_B.work2_idx_3_re / forward_kinematics_B.scale_p;
        forward_kinematics_B.scale_p += forward_kinematics_B.work2_idx_2_im *
          forward_kinematics_B.work2_idx_3_re;
        forward_kinematics_B.temp = (forward_kinematics_B.work2_idx_2_im *
          -forward_kinematics_B.work1[work2_idx_1_re_tmp].re +
          -forward_kinematics_B.work1[work2_idx_1_re_tmp].im) /
          forward_kinematics_B.scale_p;
        forward_kinematics_B.scale_p = (forward_kinematics_B.work2_idx_2_im *
          -forward_kinematics_B.work1[work2_idx_1_re_tmp].im -
          (-forward_kinematics_B.work1[work2_idx_1_re_tmp].re)) /
          forward_kinematics_B.scale_p;
      }

      forward_kinematics_B.work1[work2_idx_1_re_tmp].re =
        forward_kinematics_B.temp;
      forward_kinematics_B.work1[work2_idx_1_re_tmp].im =
        forward_kinematics_B.scale_p;
      if (work2_idx_1_re_tmp + 1 > 1) {
        if (fabs(forward_kinematics_B.work1[work2_idx_1_re_tmp].re) + fabs
            (forward_kinematics_B.work1[work2_idx_1_re_tmp].im) > 1.0) {
          forward_kinematics_B.temp = 1.0 / (fabs
            (forward_kinematics_B.work1[work2_idx_1_re_tmp].re) + fabs
            (forward_kinematics_B.work1[work2_idx_1_re_tmp].im));
          if (forward_kinematics_B.acoeff *
              forward_kinematics_B.rworka[work2_idx_1_re_tmp] >=
              1.1235582092889474E+307 * forward_kinematics_B.temp) {
            forward_kinematics_B.d_re_tmp = 0;
            while (forward_kinematics_B.d_re_tmp <= 3 -
                   forward_kinematics_B.i_ol) {
              forward_kinematics_B.work1[forward_kinematics_B.d_re_tmp].re *=
                forward_kinematics_B.temp;
              forward_kinematics_B.work1[forward_kinematics_B.d_re_tmp].im *=
                forward_kinematics_B.temp;
              forward_kinematics_B.d_re_tmp++;
            }
          }
        }

        forward_kinematics_B.work2_idx_3_re = forward_kinematics_B.acoeff *
          forward_kinematics_B.work1[work2_idx_1_re_tmp].re;
        forward_kinematics_B.scale_p = forward_kinematics_B.acoeff *
          forward_kinematics_B.work1[work2_idx_1_re_tmp].im;
        forward_kinematics_B.e_jr = 0;
        while (forward_kinematics_B.e_jr <= work2_idx_1_re_tmp - 1) {
          forward_kinematics_B.d_re_tmp = d_re_tmp_tmp +
            forward_kinematics_B.e_jr;
          forward_kinematics_B.work1[forward_kinematics_B.e_jr].re +=
            A[forward_kinematics_B.d_re_tmp].re *
            forward_kinematics_B.work2_idx_3_re -
            A[forward_kinematics_B.d_re_tmp].im * forward_kinematics_B.scale_p;
          forward_kinematics_B.work1[forward_kinematics_B.e_jr].im +=
            A[forward_kinematics_B.d_re_tmp].im *
            forward_kinematics_B.work2_idx_3_re +
            A[forward_kinematics_B.d_re_tmp].re * forward_kinematics_B.scale_p;
          forward_kinematics_B.e_jr++;
        }
      }

      forward_kinematics_B.c_j_b++;
    }

    forward_kinematics_B.salpha_re = 0.0;
    forward_kinematics_B.salpha_im = 0.0;
    forward_kinematics_B.acoeff = 0.0;
    forward_kinematics_B.dmin = 0.0;
    forward_kinematics_B.scale_p = 0.0;
    forward_kinematics_B.work2_idx_2_im = 0.0;
    forward_kinematics_B.work2_idx_3_re = 0.0;
    forward_kinematics_B.work2_idx_3_im = 0.0;
    forward_kinematics_B.c_x_tmp = 0;
    while (forward_kinematics_B.c_x_tmp <= 3 - forward_kinematics_B.i_ol) {
      forward_kinematics_B.c_j_b = forward_kinematics_B.c_x_tmp << 2;
      forward_kinematics_B.salpha_re += V[forward_kinematics_B.c_j_b].re *
        forward_kinematics_B.work1[forward_kinematics_B.c_x_tmp].re -
        V[forward_kinematics_B.c_j_b].im *
        forward_kinematics_B.work1[forward_kinematics_B.c_x_tmp].im;
      forward_kinematics_B.salpha_im += V[forward_kinematics_B.c_j_b].re *
        forward_kinematics_B.work1[forward_kinematics_B.c_x_tmp].im +
        V[forward_kinematics_B.c_j_b].im *
        forward_kinematics_B.work1[forward_kinematics_B.c_x_tmp].re;
      work2_idx_1_re_tmp = forward_kinematics_B.c_j_b + 1;
      forward_kinematics_B.acoeff += V[work2_idx_1_re_tmp].re *
        forward_kinematics_B.work1[forward_kinematics_B.c_x_tmp].re -
        V[work2_idx_1_re_tmp].im *
        forward_kinematics_B.work1[forward_kinematics_B.c_x_tmp].im;
      forward_kinematics_B.dmin += V[work2_idx_1_re_tmp].re *
        forward_kinematics_B.work1[forward_kinematics_B.c_x_tmp].im +
        V[work2_idx_1_re_tmp].im *
        forward_kinematics_B.work1[forward_kinematics_B.c_x_tmp].re;
      work2_idx_1_re_tmp = forward_kinematics_B.c_j_b + 2;
      forward_kinematics_B.scale_p += V[work2_idx_1_re_tmp].re *
        forward_kinematics_B.work1[forward_kinematics_B.c_x_tmp].re -
        V[work2_idx_1_re_tmp].im *
        forward_kinematics_B.work1[forward_kinematics_B.c_x_tmp].im;
      forward_kinematics_B.work2_idx_2_im += V[work2_idx_1_re_tmp].re *
        forward_kinematics_B.work1[forward_kinematics_B.c_x_tmp].im +
        V[work2_idx_1_re_tmp].im *
        forward_kinematics_B.work1[forward_kinematics_B.c_x_tmp].re;
      forward_kinematics_B.c_j_b += 3;
      forward_kinematics_B.work2_idx_3_re += V[forward_kinematics_B.c_j_b].re *
        forward_kinematics_B.work1[forward_kinematics_B.c_x_tmp].re -
        V[forward_kinematics_B.c_j_b].im *
        forward_kinematics_B.work1[forward_kinematics_B.c_x_tmp].im;
      forward_kinematics_B.work2_idx_3_im += V[forward_kinematics_B.c_j_b].re *
        forward_kinematics_B.work1[forward_kinematics_B.c_x_tmp].im +
        V[forward_kinematics_B.c_j_b].im *
        forward_kinematics_B.work1[forward_kinematics_B.c_x_tmp].re;
      forward_kinematics_B.c_x_tmp++;
    }

    forward_kinematics_B.temp = fabs(forward_kinematics_B.salpha_re) + fabs
      (forward_kinematics_B.salpha_im);
    forward_kinematics_B.f_y = fabs(forward_kinematics_B.acoeff) + fabs
      (forward_kinematics_B.dmin);
    if (forward_kinematics_B.f_y > forward_kinematics_B.temp) {
      forward_kinematics_B.temp = forward_kinematics_B.f_y;
    }

    forward_kinematics_B.f_y = fabs(forward_kinematics_B.scale_p) + fabs
      (forward_kinematics_B.work2_idx_2_im);
    if (forward_kinematics_B.f_y > forward_kinematics_B.temp) {
      forward_kinematics_B.temp = forward_kinematics_B.f_y;
    }

    forward_kinematics_B.f_y = fabs(forward_kinematics_B.work2_idx_3_re) + fabs
      (forward_kinematics_B.work2_idx_3_im);
    if (forward_kinematics_B.f_y > forward_kinematics_B.temp) {
      forward_kinematics_B.temp = forward_kinematics_B.f_y;
    }

    if (forward_kinematics_B.temp > 2.2250738585072014E-308) {
      forward_kinematics_B.temp = 1.0 / forward_kinematics_B.temp;
      V[forward_kinematics_B.c_x_tmp_tmp].re = forward_kinematics_B.temp *
        forward_kinematics_B.salpha_re;
      V[forward_kinematics_B.c_x_tmp_tmp].im = forward_kinematics_B.temp *
        forward_kinematics_B.salpha_im;
      forward_kinematics_B.d_re_tmp = ((3 - forward_kinematics_B.i_ol) << 2) + 1;
      V[forward_kinematics_B.d_re_tmp].re = forward_kinematics_B.temp *
        forward_kinematics_B.acoeff;
      V[forward_kinematics_B.d_re_tmp].im = forward_kinematics_B.temp *
        forward_kinematics_B.dmin;
      forward_kinematics_B.d_re_tmp = ((3 - forward_kinematics_B.i_ol) << 2) + 2;
      V[forward_kinematics_B.d_re_tmp].re = forward_kinematics_B.temp *
        forward_kinematics_B.scale_p;
      V[forward_kinematics_B.d_re_tmp].im = forward_kinematics_B.temp *
        forward_kinematics_B.work2_idx_2_im;
      forward_kinematics_B.d_re_tmp = ((3 - forward_kinematics_B.i_ol) << 2) + 3;
      V[forward_kinematics_B.d_re_tmp].re = forward_kinematics_B.temp *
        forward_kinematics_B.work2_idx_3_re;
      V[forward_kinematics_B.d_re_tmp].im = forward_kinematics_B.temp *
        forward_kinematics_B.work2_idx_3_im;
    } else {
      V[forward_kinematics_B.c_x_tmp_tmp].re = 0.0;
      V[forward_kinematics_B.c_x_tmp_tmp].im = 0.0;
      forward_kinematics_B.d_re_tmp = forward_kinematics_B.c_x_tmp_tmp + 1;
      V[forward_kinematics_B.d_re_tmp].re = 0.0;
      V[forward_kinematics_B.d_re_tmp].im = 0.0;
      forward_kinematics_B.d_re_tmp = forward_kinematics_B.c_x_tmp_tmp + 2;
      V[forward_kinematics_B.d_re_tmp].re = 0.0;
      V[forward_kinematics_B.d_re_tmp].im = 0.0;
      forward_kinematics_B.d_re_tmp = forward_kinematics_B.c_x_tmp_tmp + 3;
      V[forward_kinematics_B.d_re_tmp].re = 0.0;
      V[forward_kinematics_B.d_re_tmp].im = 0.0;
    }
  }
}

// Function for MATLAB Function: '<S1>/MATLAB Function'
static void forward_kinematics_xzggev(creal_T A[16], int32_T *info, creal_T
  alpha1[4], creal_T beta1[4], creal_T V[16])
{
  boolean_T ilascl;
  boolean_T found;
  int32_T exitg1;
  int32_T exitg2;
  boolean_T exitg3;
  boolean_T exitg4;
  *info = 0;
  forward_kinematics_B.anrm = forward_kinematics_xzlangeM(A);
  if (rtIsInf(forward_kinematics_B.anrm) || rtIsNaN(forward_kinematics_B.anrm))
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
    for (forward_kinematics_B.k_h = 0; forward_kinematics_B.k_h < 16;
         forward_kinematics_B.k_h++) {
      V[forward_kinematics_B.k_h].re = (rtNaN);
      V[forward_kinematics_B.k_h].im = 0.0;
    }
  } else {
    ilascl = false;
    forward_kinematics_B.anrmto = forward_kinematics_B.anrm;
    if ((forward_kinematics_B.anrm > 0.0) && (forward_kinematics_B.anrm <
         6.7178761075670888E-139)) {
      forward_kinematics_B.anrmto = 6.7178761075670888E-139;
      ilascl = true;
      forward_kinematics_xzlascl(forward_kinematics_B.anrm,
        forward_kinematics_B.anrmto, A);
    } else {
      if (forward_kinematics_B.anrm > 1.4885657073574029E+138) {
        forward_kinematics_B.anrmto = 1.4885657073574029E+138;
        ilascl = true;
        forward_kinematics_xzlascl(forward_kinematics_B.anrm,
          forward_kinematics_B.anrmto, A);
      }
    }

    forward_kinematics_B.rscale[0] = 1;
    forward_kinematics_B.rscale[1] = 1;
    forward_kinematics_B.rscale[2] = 1;
    forward_kinematics_B.rscale[3] = 1;
    forward_kinematics_B.c_i = 0;
    forward_kinematics_B.ihi = 4;
    do {
      exitg2 = 0;
      forward_kinematics_B.i_o = 0;
      forward_kinematics_B.jcol = 0;
      found = false;
      forward_kinematics_B.ii = forward_kinematics_B.ihi;
      exitg3 = false;
      while ((!exitg3) && (forward_kinematics_B.ii > 0)) {
        forward_kinematics_B.nzcount = 0;
        forward_kinematics_B.i_o = forward_kinematics_B.ii;
        forward_kinematics_B.jcol = forward_kinematics_B.ihi;
        forward_kinematics_B.jj = 0;
        exitg4 = false;
        while ((!exitg4) && (forward_kinematics_B.jj <= forward_kinematics_B.ihi
                             - 1)) {
          forward_kinematics_B.k_h = ((forward_kinematics_B.jj << 2) +
            forward_kinematics_B.ii) - 1;
          if ((A[forward_kinematics_B.k_h].re != 0.0) ||
              (A[forward_kinematics_B.k_h].im != 0.0) ||
              (forward_kinematics_B.jj + 1 == forward_kinematics_B.ii)) {
            if (forward_kinematics_B.nzcount == 0) {
              forward_kinematics_B.jcol = forward_kinematics_B.jj + 1;
              forward_kinematics_B.nzcount = 1;
              forward_kinematics_B.jj++;
            } else {
              forward_kinematics_B.nzcount = 2;
              exitg4 = true;
            }
          } else {
            forward_kinematics_B.jj++;
          }
        }

        if (forward_kinematics_B.nzcount < 2) {
          found = true;
          exitg3 = true;
        } else {
          forward_kinematics_B.ii--;
        }
      }

      if (!found) {
        exitg2 = 2;
      } else {
        if (forward_kinematics_B.i_o != forward_kinematics_B.ihi) {
          forward_kinematics_B.atmp_re = A[forward_kinematics_B.i_o - 1].re;
          forward_kinematics_B.atmp_im = A[forward_kinematics_B.i_o - 1].im;
          A[forward_kinematics_B.i_o - 1] = A[forward_kinematics_B.ihi - 1];
          A[forward_kinematics_B.ihi - 1].re = forward_kinematics_B.atmp_re;
          A[forward_kinematics_B.ihi - 1].im = forward_kinematics_B.atmp_im;
          forward_kinematics_B.atmp_re = A[forward_kinematics_B.i_o + 3].re;
          forward_kinematics_B.atmp_im = A[forward_kinematics_B.i_o + 3].im;
          A[forward_kinematics_B.i_o + 3] = A[forward_kinematics_B.ihi + 3];
          A[forward_kinematics_B.ihi + 3].re = forward_kinematics_B.atmp_re;
          A[forward_kinematics_B.ihi + 3].im = forward_kinematics_B.atmp_im;
          forward_kinematics_B.atmp_re = A[forward_kinematics_B.i_o + 7].re;
          forward_kinematics_B.atmp_im = A[forward_kinematics_B.i_o + 7].im;
          A[forward_kinematics_B.i_o + 7] = A[forward_kinematics_B.ihi + 7];
          A[forward_kinematics_B.ihi + 7].re = forward_kinematics_B.atmp_re;
          A[forward_kinematics_B.ihi + 7].im = forward_kinematics_B.atmp_im;
          forward_kinematics_B.atmp_re = A[forward_kinematics_B.i_o + 11].re;
          forward_kinematics_B.atmp_im = A[forward_kinematics_B.i_o + 11].im;
          A[forward_kinematics_B.i_o + 11] = A[forward_kinematics_B.ihi + 11];
          A[forward_kinematics_B.ihi + 11].re = forward_kinematics_B.atmp_re;
          A[forward_kinematics_B.ihi + 11].im = forward_kinematics_B.atmp_im;
        }

        if (forward_kinematics_B.jcol != forward_kinematics_B.ihi) {
          forward_kinematics_B.ii = 0;
          while (forward_kinematics_B.ii <= forward_kinematics_B.ihi - 1) {
            forward_kinematics_B.i_o = ((forward_kinematics_B.jcol - 1) << 2) +
              forward_kinematics_B.ii;
            forward_kinematics_B.atmp_re = A[forward_kinematics_B.i_o].re;
            forward_kinematics_B.atmp_im = A[forward_kinematics_B.i_o].im;
            forward_kinematics_B.k_h = ((forward_kinematics_B.ihi - 1) << 2) +
              forward_kinematics_B.ii;
            A[forward_kinematics_B.i_o] = A[forward_kinematics_B.k_h];
            A[forward_kinematics_B.k_h].re = forward_kinematics_B.atmp_re;
            A[forward_kinematics_B.k_h].im = forward_kinematics_B.atmp_im;
            forward_kinematics_B.ii++;
          }
        }

        forward_kinematics_B.rscale[forward_kinematics_B.ihi - 1] =
          forward_kinematics_B.jcol;
        forward_kinematics_B.ihi--;
        if (forward_kinematics_B.ihi == 1) {
          forward_kinematics_B.rscale[0] = 1;
          exitg2 = 1;
        }
      }
    } while (exitg2 == 0);

    if (exitg2 == 1) {
    } else {
      do {
        exitg1 = 0;
        forward_kinematics_B.ii = 0;
        forward_kinematics_B.jcol = 0;
        found = false;
        forward_kinematics_B.i_o = forward_kinematics_B.c_i + 1;
        exitg3 = false;
        while ((!exitg3) && (forward_kinematics_B.i_o <=
                             forward_kinematics_B.ihi)) {
          forward_kinematics_B.nzcount = 0;
          forward_kinematics_B.ii = forward_kinematics_B.ihi;
          forward_kinematics_B.jcol = forward_kinematics_B.i_o;
          forward_kinematics_B.jj = forward_kinematics_B.c_i + 1;
          exitg4 = false;
          while ((!exitg4) && (forward_kinematics_B.jj <=
                               forward_kinematics_B.ihi)) {
            forward_kinematics_B.k_h = (((forward_kinematics_B.i_o - 1) << 2) +
              forward_kinematics_B.jj) - 1;
            if ((A[forward_kinematics_B.k_h].re != 0.0) ||
                (A[forward_kinematics_B.k_h].im != 0.0) ||
                (forward_kinematics_B.jj == forward_kinematics_B.i_o)) {
              if (forward_kinematics_B.nzcount == 0) {
                forward_kinematics_B.ii = forward_kinematics_B.jj;
                forward_kinematics_B.nzcount = 1;
                forward_kinematics_B.jj++;
              } else {
                forward_kinematics_B.nzcount = 2;
                exitg4 = true;
              }
            } else {
              forward_kinematics_B.jj++;
            }
          }

          if (forward_kinematics_B.nzcount < 2) {
            found = true;
            exitg3 = true;
          } else {
            forward_kinematics_B.i_o++;
          }
        }

        if (!found) {
          exitg1 = 1;
        } else {
          if (forward_kinematics_B.c_i + 1 != forward_kinematics_B.ii) {
            forward_kinematics_B.nzcount = forward_kinematics_B.c_i;
            while (forward_kinematics_B.nzcount + 1 < 5) {
              forward_kinematics_B.k_h = forward_kinematics_B.nzcount << 2;
              forward_kinematics_B.i_o = (forward_kinematics_B.k_h +
                forward_kinematics_B.ii) - 1;
              forward_kinematics_B.atmp_re = A[forward_kinematics_B.i_o].re;
              forward_kinematics_B.atmp_im = A[forward_kinematics_B.i_o].im;
              forward_kinematics_B.k_h += forward_kinematics_B.c_i;
              A[forward_kinematics_B.i_o] = A[forward_kinematics_B.k_h];
              A[forward_kinematics_B.k_h].re = forward_kinematics_B.atmp_re;
              A[forward_kinematics_B.k_h].im = forward_kinematics_B.atmp_im;
              forward_kinematics_B.nzcount++;
            }
          }

          if (forward_kinematics_B.c_i + 1 != forward_kinematics_B.jcol) {
            forward_kinematics_B.ii = 0;
            while (forward_kinematics_B.ii <= forward_kinematics_B.ihi - 1) {
              forward_kinematics_B.i_o = ((forward_kinematics_B.jcol - 1) << 2)
                + forward_kinematics_B.ii;
              forward_kinematics_B.atmp_re = A[forward_kinematics_B.i_o].re;
              forward_kinematics_B.atmp_im = A[forward_kinematics_B.i_o].im;
              forward_kinematics_B.k_h = (forward_kinematics_B.c_i << 2) +
                forward_kinematics_B.ii;
              A[forward_kinematics_B.i_o] = A[forward_kinematics_B.k_h];
              A[forward_kinematics_B.k_h].re = forward_kinematics_B.atmp_re;
              A[forward_kinematics_B.k_h].im = forward_kinematics_B.atmp_im;
              forward_kinematics_B.ii++;
            }
          }

          forward_kinematics_B.rscale[forward_kinematics_B.c_i] =
            forward_kinematics_B.jcol;
          forward_kinematics_B.c_i++;
          if (forward_kinematics_B.c_i + 1 == forward_kinematics_B.ihi) {
            forward_kinematics_B.rscale[forward_kinematics_B.c_i] =
              forward_kinematics_B.c_i + 1;
            exitg1 = 1;
          }
        }
      } while (exitg1 == 0);
    }

    for (forward_kinematics_B.k_h = 0; forward_kinematics_B.k_h < 16;
         forward_kinematics_B.k_h++) {
      forward_kinematics_B.b_I[forward_kinematics_B.k_h] = 0;
    }

    forward_kinematics_B.b_I[0] = 1;
    forward_kinematics_B.b_I[5] = 1;
    forward_kinematics_B.b_I[10] = 1;
    forward_kinematics_B.b_I[15] = 1;
    for (forward_kinematics_B.k_h = 0; forward_kinematics_B.k_h < 16;
         forward_kinematics_B.k_h++) {
      V[forward_kinematics_B.k_h].re =
        forward_kinematics_B.b_I[forward_kinematics_B.k_h];
      V[forward_kinematics_B.k_h].im = 0.0;
    }

    if (forward_kinematics_B.ihi >= forward_kinematics_B.c_i + 3) {
      forward_kinematics_B.jcol = forward_kinematics_B.c_i;
      while (forward_kinematics_B.jcol + 1 < forward_kinematics_B.ihi - 1) {
        forward_kinematics_B.ii = forward_kinematics_B.ihi - 1;
        while (forward_kinematics_B.ii + 1 > forward_kinematics_B.jcol + 2) {
          forward_kinematics_xzlartg(A[(forward_kinematics_B.ii +
            (forward_kinematics_B.jcol << 2)) - 1], A[forward_kinematics_B.ii +
            (forward_kinematics_B.jcol << 2)], &forward_kinematics_B.mul,
            &forward_kinematics_B.s, &A[(forward_kinematics_B.ii +
            (forward_kinematics_B.jcol << 2)) - 1]);
          forward_kinematics_B.k_h = forward_kinematics_B.ii +
            (forward_kinematics_B.jcol << 2);
          A[forward_kinematics_B.k_h].re = 0.0;
          A[forward_kinematics_B.k_h].im = 0.0;
          forward_kinematics_B.nzcount = forward_kinematics_B.jcol + 1;
          while (forward_kinematics_B.nzcount + 1 < 5) {
            forward_kinematics_B.i_o = (forward_kinematics_B.nzcount << 2) +
              forward_kinematics_B.ii;
            forward_kinematics_B.k_h = forward_kinematics_B.i_o - 1;
            forward_kinematics_B.atmp_re = A[forward_kinematics_B.k_h].re *
              forward_kinematics_B.mul + (A[forward_kinematics_B.i_o].re *
              forward_kinematics_B.s.re - A[forward_kinematics_B.i_o].im *
              forward_kinematics_B.s.im);
            forward_kinematics_B.atmp_im = A[forward_kinematics_B.k_h].im *
              forward_kinematics_B.mul + (A[forward_kinematics_B.i_o].im *
              forward_kinematics_B.s.re + A[forward_kinematics_B.i_o].re *
              forward_kinematics_B.s.im);
            forward_kinematics_B.d = A[forward_kinematics_B.k_h].im;
            forward_kinematics_B.d1 = A[forward_kinematics_B.k_h].re;
            A[forward_kinematics_B.i_o].re = A[forward_kinematics_B.i_o].re *
              forward_kinematics_B.mul - (A[forward_kinematics_B.k_h].re *
              forward_kinematics_B.s.re + A[forward_kinematics_B.k_h].im *
              forward_kinematics_B.s.im);
            A[forward_kinematics_B.i_o].im = A[forward_kinematics_B.i_o].im *
              forward_kinematics_B.mul - (forward_kinematics_B.s.re *
              forward_kinematics_B.d - forward_kinematics_B.s.im *
              forward_kinematics_B.d1);
            A[forward_kinematics_B.k_h].re = forward_kinematics_B.atmp_re;
            A[forward_kinematics_B.k_h].im = forward_kinematics_B.atmp_im;
            forward_kinematics_B.nzcount++;
          }

          forward_kinematics_B.s.re = -forward_kinematics_B.s.re;
          forward_kinematics_B.s.im = -forward_kinematics_B.s.im;
          forward_kinematics_B.nzcount = 0;
          while (forward_kinematics_B.nzcount + 1 <= forward_kinematics_B.ihi) {
            forward_kinematics_B.i_o = ((forward_kinematics_B.ii - 1) << 2) +
              forward_kinematics_B.nzcount;
            forward_kinematics_B.k_h = (forward_kinematics_B.ii << 2) +
              forward_kinematics_B.nzcount;
            forward_kinematics_B.atmp_re = (A[forward_kinematics_B.i_o].re *
              forward_kinematics_B.s.re - A[forward_kinematics_B.i_o].im *
              forward_kinematics_B.s.im) + A[forward_kinematics_B.k_h].re *
              forward_kinematics_B.mul;
            forward_kinematics_B.atmp_im = (A[forward_kinematics_B.i_o].im *
              forward_kinematics_B.s.re + A[forward_kinematics_B.i_o].re *
              forward_kinematics_B.s.im) + A[forward_kinematics_B.k_h].im *
              forward_kinematics_B.mul;
            forward_kinematics_B.d = A[forward_kinematics_B.k_h].im;
            forward_kinematics_B.d1 = A[forward_kinematics_B.k_h].re;
            A[forward_kinematics_B.i_o].re = A[forward_kinematics_B.i_o].re *
              forward_kinematics_B.mul - (A[forward_kinematics_B.k_h].re *
              forward_kinematics_B.s.re + A[forward_kinematics_B.k_h].im *
              forward_kinematics_B.s.im);
            A[forward_kinematics_B.i_o].im = A[forward_kinematics_B.i_o].im *
              forward_kinematics_B.mul - (forward_kinematics_B.s.re *
              forward_kinematics_B.d - forward_kinematics_B.s.im *
              forward_kinematics_B.d1);
            A[forward_kinematics_B.k_h].re = forward_kinematics_B.atmp_re;
            A[forward_kinematics_B.k_h].im = forward_kinematics_B.atmp_im;
            forward_kinematics_B.nzcount++;
          }

          forward_kinematics_B.i_o = (forward_kinematics_B.ii - 1) << 2;
          forward_kinematics_B.k_h = forward_kinematics_B.ii << 2;
          forward_kinematics_B.atmp_re = (V[forward_kinematics_B.i_o].re *
            forward_kinematics_B.s.re - V[forward_kinematics_B.i_o].im *
            forward_kinematics_B.s.im) + V[forward_kinematics_B.k_h].re *
            forward_kinematics_B.mul;
          forward_kinematics_B.atmp_im = (V[forward_kinematics_B.i_o].im *
            forward_kinematics_B.s.re + V[forward_kinematics_B.i_o].re *
            forward_kinematics_B.s.im) + V[forward_kinematics_B.k_h].im *
            forward_kinematics_B.mul;
          forward_kinematics_B.d = V[forward_kinematics_B.k_h].re;
          V[forward_kinematics_B.i_o].re = V[forward_kinematics_B.i_o].re *
            forward_kinematics_B.mul - (V[forward_kinematics_B.k_h].re *
            forward_kinematics_B.s.re + V[forward_kinematics_B.k_h].im *
            forward_kinematics_B.s.im);
          V[forward_kinematics_B.i_o].im = V[forward_kinematics_B.i_o].im *
            forward_kinematics_B.mul - (V[forward_kinematics_B.k_h].im *
            forward_kinematics_B.s.re - forward_kinematics_B.s.im *
            forward_kinematics_B.d);
          V[forward_kinematics_B.k_h].re = forward_kinematics_B.atmp_re;
          V[forward_kinematics_B.k_h].im = forward_kinematics_B.atmp_im;
          forward_kinematics_B.nzcount = forward_kinematics_B.i_o + 1;
          forward_kinematics_B.jj = forward_kinematics_B.k_h + 1;
          forward_kinematics_B.atmp_re = (V[forward_kinematics_B.nzcount].re *
            forward_kinematics_B.s.re - V[forward_kinematics_B.nzcount].im *
            forward_kinematics_B.s.im) + V[forward_kinematics_B.jj].re *
            forward_kinematics_B.mul;
          forward_kinematics_B.atmp_im = (V[forward_kinematics_B.nzcount].im *
            forward_kinematics_B.s.re + V[forward_kinematics_B.nzcount].re *
            forward_kinematics_B.s.im) + V[forward_kinematics_B.jj].im *
            forward_kinematics_B.mul;
          forward_kinematics_B.d = V[forward_kinematics_B.jj].re;
          V[forward_kinematics_B.nzcount].re = V[forward_kinematics_B.nzcount].
            re * forward_kinematics_B.mul - (V[forward_kinematics_B.jj].re *
            forward_kinematics_B.s.re + V[forward_kinematics_B.jj].im *
            forward_kinematics_B.s.im);
          V[forward_kinematics_B.nzcount].im = V[forward_kinematics_B.nzcount].
            im * forward_kinematics_B.mul - (V[forward_kinematics_B.jj].im *
            forward_kinematics_B.s.re - forward_kinematics_B.s.im *
            forward_kinematics_B.d);
          V[forward_kinematics_B.jj].re = forward_kinematics_B.atmp_re;
          V[forward_kinematics_B.jj].im = forward_kinematics_B.atmp_im;
          forward_kinematics_B.nzcount = forward_kinematics_B.i_o + 2;
          forward_kinematics_B.jj = forward_kinematics_B.k_h + 2;
          forward_kinematics_B.atmp_re = (V[forward_kinematics_B.nzcount].re *
            forward_kinematics_B.s.re - V[forward_kinematics_B.nzcount].im *
            forward_kinematics_B.s.im) + V[forward_kinematics_B.jj].re *
            forward_kinematics_B.mul;
          forward_kinematics_B.atmp_im = (V[forward_kinematics_B.nzcount].im *
            forward_kinematics_B.s.re + V[forward_kinematics_B.nzcount].re *
            forward_kinematics_B.s.im) + V[forward_kinematics_B.jj].im *
            forward_kinematics_B.mul;
          forward_kinematics_B.d = V[forward_kinematics_B.jj].re;
          V[forward_kinematics_B.nzcount].re = V[forward_kinematics_B.nzcount].
            re * forward_kinematics_B.mul - (V[forward_kinematics_B.jj].re *
            forward_kinematics_B.s.re + V[forward_kinematics_B.jj].im *
            forward_kinematics_B.s.im);
          V[forward_kinematics_B.nzcount].im = V[forward_kinematics_B.nzcount].
            im * forward_kinematics_B.mul - (V[forward_kinematics_B.jj].im *
            forward_kinematics_B.s.re - forward_kinematics_B.s.im *
            forward_kinematics_B.d);
          V[forward_kinematics_B.jj].re = forward_kinematics_B.atmp_re;
          V[forward_kinematics_B.jj].im = forward_kinematics_B.atmp_im;
          forward_kinematics_B.i_o += 3;
          forward_kinematics_B.k_h += 3;
          forward_kinematics_B.atmp_re = (V[forward_kinematics_B.i_o].re *
            forward_kinematics_B.s.re - V[forward_kinematics_B.i_o].im *
            forward_kinematics_B.s.im) + V[forward_kinematics_B.k_h].re *
            forward_kinematics_B.mul;
          forward_kinematics_B.atmp_im = (V[forward_kinematics_B.i_o].im *
            forward_kinematics_B.s.re + V[forward_kinematics_B.i_o].re *
            forward_kinematics_B.s.im) + V[forward_kinematics_B.k_h].im *
            forward_kinematics_B.mul;
          forward_kinematics_B.d = V[forward_kinematics_B.k_h].re;
          V[forward_kinematics_B.i_o].re = V[forward_kinematics_B.i_o].re *
            forward_kinematics_B.mul - (V[forward_kinematics_B.k_h].re *
            forward_kinematics_B.s.re + V[forward_kinematics_B.k_h].im *
            forward_kinematics_B.s.im);
          V[forward_kinematics_B.i_o].im = V[forward_kinematics_B.i_o].im *
            forward_kinematics_B.mul - (V[forward_kinematics_B.k_h].im *
            forward_kinematics_B.s.re - forward_kinematics_B.s.im *
            forward_kinematics_B.d);
          V[forward_kinematics_B.k_h].re = forward_kinematics_B.atmp_re;
          V[forward_kinematics_B.k_h].im = forward_kinematics_B.atmp_im;
          forward_kinematics_B.ii--;
        }

        forward_kinematics_B.jcol++;
      }
    }

    forward_kinematics_xzhgeqz(A, forward_kinematics_B.c_i + 1,
      forward_kinematics_B.ihi, V, info, alpha1, beta1);
    if (*info == 0) {
      forward_kinematics_xztgevc(A, V);
      if (forward_kinematics_B.c_i + 1 > 1) {
        forward_kinematics_B.c_i--;
        while (forward_kinematics_B.c_i + 1 >= 1) {
          forward_kinematics_B.k_h =
            forward_kinematics_B.rscale[forward_kinematics_B.c_i] - 1;
          if (forward_kinematics_B.c_i + 1 !=
              forward_kinematics_B.rscale[forward_kinematics_B.c_i]) {
            forward_kinematics_B.atmp_re = V[forward_kinematics_B.c_i].re;
            forward_kinematics_B.atmp_im = V[forward_kinematics_B.c_i].im;
            V[forward_kinematics_B.c_i] = V[forward_kinematics_B.k_h];
            V[forward_kinematics_B.k_h].re = forward_kinematics_B.atmp_re;
            V[forward_kinematics_B.k_h].im = forward_kinematics_B.atmp_im;
            forward_kinematics_B.atmp_re = V[forward_kinematics_B.c_i + 4].re;
            forward_kinematics_B.atmp_im = V[forward_kinematics_B.c_i + 4].im;
            V[forward_kinematics_B.c_i + 4] = V[forward_kinematics_B.k_h + 4];
            V[forward_kinematics_B.k_h + 4].re = forward_kinematics_B.atmp_re;
            V[forward_kinematics_B.k_h + 4].im = forward_kinematics_B.atmp_im;
            forward_kinematics_B.atmp_re = V[forward_kinematics_B.c_i + 8].re;
            forward_kinematics_B.atmp_im = V[forward_kinematics_B.c_i + 8].im;
            V[forward_kinematics_B.c_i + 8] = V[forward_kinematics_B.k_h + 8];
            V[forward_kinematics_B.k_h + 8].re = forward_kinematics_B.atmp_re;
            V[forward_kinematics_B.k_h + 8].im = forward_kinematics_B.atmp_im;
            forward_kinematics_B.atmp_re = V[forward_kinematics_B.c_i + 12].re;
            forward_kinematics_B.atmp_im = V[forward_kinematics_B.c_i + 12].im;
            V[forward_kinematics_B.c_i + 12] = V[forward_kinematics_B.k_h + 12];
            V[forward_kinematics_B.k_h + 12].re = forward_kinematics_B.atmp_re;
            V[forward_kinematics_B.k_h + 12].im = forward_kinematics_B.atmp_im;
          }

          forward_kinematics_B.c_i--;
        }
      }

      if (forward_kinematics_B.ihi < 4) {
        while (forward_kinematics_B.ihi + 1 < 5) {
          forward_kinematics_B.k_h =
            forward_kinematics_B.rscale[forward_kinematics_B.ihi] - 1;
          if (forward_kinematics_B.ihi + 1 !=
              forward_kinematics_B.rscale[forward_kinematics_B.ihi]) {
            forward_kinematics_B.atmp_re = V[forward_kinematics_B.ihi].re;
            forward_kinematics_B.atmp_im = V[forward_kinematics_B.ihi].im;
            V[forward_kinematics_B.ihi] = V[forward_kinematics_B.k_h];
            V[forward_kinematics_B.k_h].re = forward_kinematics_B.atmp_re;
            V[forward_kinematics_B.k_h].im = forward_kinematics_B.atmp_im;
            forward_kinematics_B.atmp_re = V[forward_kinematics_B.ihi + 4].re;
            forward_kinematics_B.atmp_im = V[forward_kinematics_B.ihi + 4].im;
            V[forward_kinematics_B.ihi + 4] = V[forward_kinematics_B.k_h + 4];
            V[forward_kinematics_B.k_h + 4].re = forward_kinematics_B.atmp_re;
            V[forward_kinematics_B.k_h + 4].im = forward_kinematics_B.atmp_im;
            forward_kinematics_B.atmp_re = V[forward_kinematics_B.ihi + 8].re;
            forward_kinematics_B.atmp_im = V[forward_kinematics_B.ihi + 8].im;
            V[forward_kinematics_B.ihi + 8] = V[forward_kinematics_B.k_h + 8];
            V[forward_kinematics_B.k_h + 8].re = forward_kinematics_B.atmp_re;
            V[forward_kinematics_B.k_h + 8].im = forward_kinematics_B.atmp_im;
            forward_kinematics_B.atmp_re = V[forward_kinematics_B.ihi + 12].re;
            forward_kinematics_B.atmp_im = V[forward_kinematics_B.ihi + 12].im;
            V[forward_kinematics_B.ihi + 12] = V[forward_kinematics_B.k_h + 12];
            V[forward_kinematics_B.k_h + 12].re = forward_kinematics_B.atmp_re;
            V[forward_kinematics_B.k_h + 12].im = forward_kinematics_B.atmp_im;
          }

          forward_kinematics_B.ihi++;
        }
      }

      for (forward_kinematics_B.ihi = 0; forward_kinematics_B.ihi < 4;
           forward_kinematics_B.ihi++) {
        forward_kinematics_B.c_i = forward_kinematics_B.ihi << 2;
        forward_kinematics_B.atmp_re = fabs(V[forward_kinematics_B.c_i].re) +
          fabs(V[forward_kinematics_B.c_i].im);
        forward_kinematics_B.k_h = forward_kinematics_B.c_i + 1;
        forward_kinematics_B.atmp_im = fabs(V[forward_kinematics_B.k_h].re) +
          fabs(V[forward_kinematics_B.k_h].im);
        if (forward_kinematics_B.atmp_im > forward_kinematics_B.atmp_re) {
          forward_kinematics_B.atmp_re = forward_kinematics_B.atmp_im;
        }

        forward_kinematics_B.i_o = forward_kinematics_B.c_i + 2;
        forward_kinematics_B.atmp_im = fabs(V[forward_kinematics_B.i_o].re) +
          fabs(V[forward_kinematics_B.i_o].im);
        if (forward_kinematics_B.atmp_im > forward_kinematics_B.atmp_re) {
          forward_kinematics_B.atmp_re = forward_kinematics_B.atmp_im;
        }

        forward_kinematics_B.jcol = forward_kinematics_B.c_i + 3;
        forward_kinematics_B.atmp_im = fabs(V[forward_kinematics_B.jcol].re) +
          fabs(V[forward_kinematics_B.jcol].im);
        if (forward_kinematics_B.atmp_im > forward_kinematics_B.atmp_re) {
          forward_kinematics_B.atmp_re = forward_kinematics_B.atmp_im;
        }

        if (forward_kinematics_B.atmp_re >= 6.7178761075670888E-139) {
          forward_kinematics_B.atmp_re = 1.0 / forward_kinematics_B.atmp_re;
          V[forward_kinematics_B.c_i].re *= forward_kinematics_B.atmp_re;
          V[forward_kinematics_B.c_i].im *= forward_kinematics_B.atmp_re;
          V[forward_kinematics_B.k_h].re *= forward_kinematics_B.atmp_re;
          V[forward_kinematics_B.k_h].im *= forward_kinematics_B.atmp_re;
          V[forward_kinematics_B.i_o].re *= forward_kinematics_B.atmp_re;
          V[forward_kinematics_B.i_o].im *= forward_kinematics_B.atmp_re;
          V[forward_kinematics_B.jcol].re *= forward_kinematics_B.atmp_re;
          V[forward_kinematics_B.jcol].im *= forward_kinematics_B.atmp_re;
        }
      }

      if (ilascl) {
        ilascl = true;
        while (ilascl) {
          forward_kinematics_B.atmp_re = forward_kinematics_B.anrmto *
            2.0041683600089728E-292;
          forward_kinematics_B.atmp_im = forward_kinematics_B.anrm /
            4.9896007738368E+291;
          if ((fabs(forward_kinematics_B.atmp_re) > fabs
               (forward_kinematics_B.anrm)) && (forward_kinematics_B.anrm != 0.0))
          {
            forward_kinematics_B.mul = 2.0041683600089728E-292;
            forward_kinematics_B.anrmto = forward_kinematics_B.atmp_re;
          } else if (fabs(forward_kinematics_B.atmp_im) > fabs
                     (forward_kinematics_B.anrmto)) {
            forward_kinematics_B.mul = 4.9896007738368E+291;
            forward_kinematics_B.anrm = forward_kinematics_B.atmp_im;
          } else {
            forward_kinematics_B.mul = forward_kinematics_B.anrm /
              forward_kinematics_B.anrmto;
            ilascl = false;
          }

          alpha1[0].re *= forward_kinematics_B.mul;
          alpha1[0].im *= forward_kinematics_B.mul;
          alpha1[1].re *= forward_kinematics_B.mul;
          alpha1[1].im *= forward_kinematics_B.mul;
          alpha1[2].re *= forward_kinematics_B.mul;
          alpha1[2].im *= forward_kinematics_B.mul;
          alpha1[3].re *= forward_kinematics_B.mul;
          alpha1[3].im *= forward_kinematics_B.mul;
        }
      }
    }
  }
}

// Function for MATLAB Function: '<S1>/MATLAB Function'
static real_T forward_kinematics_xnrm2(int32_T n, const real_T x[16], int32_T
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

// Function for MATLAB Function: '<S1>/MATLAB Function'
static void forward_kinematics_xzlarf(int32_T m, int32_T n, int32_T iv0, real_T
  tau, real_T C[16], int32_T ic0, real_T work[4])
{
  int32_T d;
  int32_T b_ia;
  int32_T jy;
  int32_T exitg1;
  boolean_T exitg2;
  if (tau != 0.0) {
    forward_kinematics_B.lastv = m;
    forward_kinematics_B.lastc_a = iv0 + m;
    while ((forward_kinematics_B.lastv > 0) && (C[forward_kinematics_B.lastc_a -
            2] == 0.0)) {
      forward_kinematics_B.lastv--;
      forward_kinematics_B.lastc_a--;
    }

    forward_kinematics_B.lastc_a = n - 1;
    exitg2 = false;
    while ((!exitg2) && (forward_kinematics_B.lastc_a + 1 > 0)) {
      forward_kinematics_B.coltop = (forward_kinematics_B.lastc_a << 2) + ic0;
      jy = forward_kinematics_B.coltop;
      do {
        exitg1 = 0;
        if (jy <= (forward_kinematics_B.coltop + forward_kinematics_B.lastv) - 1)
        {
          if (C[jy - 1] != 0.0) {
            exitg1 = 1;
          } else {
            jy++;
          }
        } else {
          forward_kinematics_B.lastc_a--;
          exitg1 = 2;
        }
      } while (exitg1 == 0);

      if (exitg1 == 1) {
        exitg2 = true;
      }
    }
  } else {
    forward_kinematics_B.lastv = 0;
    forward_kinematics_B.lastc_a = -1;
  }

  if (forward_kinematics_B.lastv > 0) {
    if (forward_kinematics_B.lastc_a + 1 != 0) {
      forward_kinematics_B.coltop = 0;
      while (forward_kinematics_B.coltop <= forward_kinematics_B.lastc_a) {
        work[forward_kinematics_B.coltop] = 0.0;
        forward_kinematics_B.coltop++;
      }

      forward_kinematics_B.coltop = 0;
      jy = (forward_kinematics_B.lastc_a << 2) + ic0;
      forward_kinematics_B.iac_e = ic0;
      while (forward_kinematics_B.iac_e <= jy) {
        forward_kinematics_B.ix_g = iv0;
        forward_kinematics_B.c = 0.0;
        d = (forward_kinematics_B.iac_e + forward_kinematics_B.lastv) - 1;
        for (b_ia = forward_kinematics_B.iac_e; b_ia <= d; b_ia++) {
          forward_kinematics_B.c += C[b_ia - 1] * C[forward_kinematics_B.ix_g -
            1];
          forward_kinematics_B.ix_g++;
        }

        work[forward_kinematics_B.coltop] += forward_kinematics_B.c;
        forward_kinematics_B.coltop++;
        forward_kinematics_B.iac_e += 4;
      }
    }

    if (!(-tau == 0.0)) {
      forward_kinematics_B.coltop = ic0 - 1;
      jy = 0;
      forward_kinematics_B.iac_e = 0;
      while (forward_kinematics_B.iac_e <= forward_kinematics_B.lastc_a) {
        if (work[jy] != 0.0) {
          forward_kinematics_B.c = work[jy] * -tau;
          forward_kinematics_B.ix_g = iv0;
          d = forward_kinematics_B.lastv + forward_kinematics_B.coltop;
          for (b_ia = forward_kinematics_B.coltop; b_ia < d; b_ia++) {
            C[b_ia] += C[forward_kinematics_B.ix_g - 1] * forward_kinematics_B.c;
            forward_kinematics_B.ix_g++;
          }
        }

        jy++;
        forward_kinematics_B.coltop += 4;
        forward_kinematics_B.iac_e++;
      }
    }
  }
}

// Function for MATLAB Function: '<S1>/MATLAB Function'
static void forward_kinematics_xgehrd(real_T a[16], real_T tau[3])
{
  int32_T exitg1;
  boolean_T exitg2;
  forward_kinematics_B.work_g[0] = 0.0;
  forward_kinematics_B.work_g[1] = 0.0;
  forward_kinematics_B.work_g[2] = 0.0;
  forward_kinematics_B.work_g[3] = 0.0;
  forward_kinematics_B.alpha1 = a[1];
  tau[0] = 0.0;
  forward_kinematics_B.xnorm = forward_kinematics_xnrm2(2, a, 3);
  if (forward_kinematics_B.xnorm != 0.0) {
    forward_kinematics_B.xnorm = forward_kinematic_rt_hypotd_snf(a[1],
      forward_kinematics_B.xnorm);
    if (a[1] >= 0.0) {
      forward_kinematics_B.xnorm = -forward_kinematics_B.xnorm;
    }

    if (fabs(forward_kinematics_B.xnorm) < 1.0020841800044864E-292) {
      forward_kinematics_B.knt = -1;
      do {
        forward_kinematics_B.knt++;
        forward_kinematics_B.lastc = 3;
        while (forward_kinematics_B.lastc <= 4) {
          a[forward_kinematics_B.lastc - 1] *= 9.9792015476736E+291;
          forward_kinematics_B.lastc++;
        }

        forward_kinematics_B.xnorm *= 9.9792015476736E+291;
        forward_kinematics_B.alpha1 *= 9.9792015476736E+291;
      } while (!(fabs(forward_kinematics_B.xnorm) >= 1.0020841800044864E-292));

      forward_kinematics_B.xnorm = forward_kinematic_rt_hypotd_snf
        (forward_kinematics_B.alpha1, forward_kinematics_xnrm2(2, a, 3));
      if (forward_kinematics_B.alpha1 >= 0.0) {
        forward_kinematics_B.xnorm = -forward_kinematics_B.xnorm;
      }

      tau[0] = (forward_kinematics_B.xnorm - forward_kinematics_B.alpha1) /
        forward_kinematics_B.xnorm;
      forward_kinematics_B.alpha1 = 1.0 / (forward_kinematics_B.alpha1 -
        forward_kinematics_B.xnorm);
      forward_kinematics_B.lastc = 3;
      while (forward_kinematics_B.lastc <= 4) {
        a[forward_kinematics_B.lastc - 1] *= forward_kinematics_B.alpha1;
        forward_kinematics_B.lastc++;
      }

      forward_kinematics_B.lastc = 0;
      while (forward_kinematics_B.lastc <= forward_kinematics_B.knt) {
        forward_kinematics_B.xnorm *= 1.0020841800044864E-292;
        forward_kinematics_B.lastc++;
      }

      forward_kinematics_B.alpha1 = forward_kinematics_B.xnorm;
    } else {
      tau[0] = (forward_kinematics_B.xnorm - a[1]) / forward_kinematics_B.xnorm;
      forward_kinematics_B.alpha1 = 1.0 / (a[1] - forward_kinematics_B.xnorm);
      forward_kinematics_B.knt = 3;
      while (forward_kinematics_B.knt <= 4) {
        a[forward_kinematics_B.knt - 1] *= forward_kinematics_B.alpha1;
        forward_kinematics_B.knt++;
      }

      forward_kinematics_B.alpha1 = forward_kinematics_B.xnorm;
    }
  }

  a[1] = 1.0;
  if (tau[0] != 0.0) {
    forward_kinematics_B.knt = 2;
    forward_kinematics_B.lastc = 3;
    while ((forward_kinematics_B.knt + 1 > 0) && (a[forward_kinematics_B.lastc] ==
            0.0)) {
      forward_kinematics_B.knt--;
      forward_kinematics_B.lastc--;
    }

    forward_kinematics_B.lastc = 4;
    exitg2 = false;
    while ((!exitg2) && (forward_kinematics_B.lastc > 0)) {
      forward_kinematics_B.rowleft = forward_kinematics_B.lastc + 4;
      forward_kinematics_B.jy = forward_kinematics_B.rowleft;
      do {
        exitg1 = 0;
        if (forward_kinematics_B.jy <= (forward_kinematics_B.knt << 2) +
            forward_kinematics_B.rowleft) {
          if (a[forward_kinematics_B.jy - 1] != 0.0) {
            exitg1 = 1;
          } else {
            forward_kinematics_B.jy += 4;
          }
        } else {
          forward_kinematics_B.lastc--;
          exitg1 = 2;
        }
      } while (exitg1 == 0);

      if (exitg1 == 1) {
        exitg2 = true;
      }
    }
  } else {
    forward_kinematics_B.knt = -1;
    forward_kinematics_B.lastc = 0;
  }

  if (forward_kinematics_B.knt + 1 > 0) {
    if (forward_kinematics_B.lastc != 0) {
      forward_kinematics_B.rowleft = 0;
      while (forward_kinematics_B.rowleft <= forward_kinematics_B.lastc - 1) {
        forward_kinematics_B.work_g[forward_kinematics_B.rowleft] = 0.0;
        forward_kinematics_B.rowleft++;
      }

      forward_kinematics_B.rowleft = 1;
      forward_kinematics_B.jy = (forward_kinematics_B.knt << 2) + 5;
      forward_kinematics_B.iac = 5;
      while (forward_kinematics_B.iac <= forward_kinematics_B.jy) {
        forward_kinematics_B.b_ix = 0;
        forward_kinematics_B.g = (forward_kinematics_B.iac +
          forward_kinematics_B.lastc) - 1;
        forward_kinematics_B.b_ia = forward_kinematics_B.iac;
        while (forward_kinematics_B.b_ia <= forward_kinematics_B.g) {
          forward_kinematics_B.work_g[forward_kinematics_B.b_ix] +=
            a[forward_kinematics_B.b_ia - 1] * a[forward_kinematics_B.rowleft];
          forward_kinematics_B.b_ix++;
          forward_kinematics_B.b_ia++;
        }

        forward_kinematics_B.rowleft++;
        forward_kinematics_B.iac += 4;
      }
    }

    if (!(-tau[0] == 0.0)) {
      forward_kinematics_B.rowleft = 4;
      forward_kinematics_B.jy = 1;
      forward_kinematics_B.iac = 0;
      while (forward_kinematics_B.iac <= forward_kinematics_B.knt) {
        if (a[forward_kinematics_B.jy] != 0.0) {
          forward_kinematics_B.xnorm = a[forward_kinematics_B.jy] * -tau[0];
          forward_kinematics_B.b_ix = 0;
          forward_kinematics_B.g = forward_kinematics_B.lastc +
            forward_kinematics_B.rowleft;
          forward_kinematics_B.b_ia = forward_kinematics_B.rowleft;
          while (forward_kinematics_B.b_ia + 1 <= forward_kinematics_B.g) {
            a[forward_kinematics_B.b_ia] +=
              forward_kinematics_B.work_g[forward_kinematics_B.b_ix] *
              forward_kinematics_B.xnorm;
            forward_kinematics_B.b_ix++;
            forward_kinematics_B.b_ia++;
          }
        }

        forward_kinematics_B.jy++;
        forward_kinematics_B.rowleft += 4;
        forward_kinematics_B.iac++;
      }
    }
  }

  forward_kinematics_xzlarf(3, 3, 2, tau[0], a, 6, forward_kinematics_B.work_g);
  a[1] = forward_kinematics_B.alpha1;
  forward_kinematics_B.alpha1 = a[6];
  tau[1] = 0.0;
  forward_kinematics_B.xnorm = forward_kinematics_xnrm2(1, a, 8);
  if (forward_kinematics_B.xnorm != 0.0) {
    forward_kinematics_B.xnorm = forward_kinematic_rt_hypotd_snf(a[6],
      forward_kinematics_B.xnorm);
    if (a[6] >= 0.0) {
      forward_kinematics_B.xnorm = -forward_kinematics_B.xnorm;
    }

    if (fabs(forward_kinematics_B.xnorm) < 1.0020841800044864E-292) {
      forward_kinematics_B.knt = -1;
      do {
        forward_kinematics_B.knt++;
        a[7] *= 9.9792015476736E+291;
        forward_kinematics_B.xnorm *= 9.9792015476736E+291;
        forward_kinematics_B.alpha1 *= 9.9792015476736E+291;
      } while (!(fabs(forward_kinematics_B.xnorm) >= 1.0020841800044864E-292));

      forward_kinematics_B.xnorm = forward_kinematic_rt_hypotd_snf
        (forward_kinematics_B.alpha1, forward_kinematics_xnrm2(1, a, 8));
      if (forward_kinematics_B.alpha1 >= 0.0) {
        forward_kinematics_B.xnorm = -forward_kinematics_B.xnorm;
      }

      tau[1] = (forward_kinematics_B.xnorm - forward_kinematics_B.alpha1) /
        forward_kinematics_B.xnorm;
      forward_kinematics_B.alpha1 = 1.0 / (forward_kinematics_B.alpha1 -
        forward_kinematics_B.xnorm);
      a[7] *= forward_kinematics_B.alpha1;
      forward_kinematics_B.lastc = 0;
      while (forward_kinematics_B.lastc <= forward_kinematics_B.knt) {
        forward_kinematics_B.xnorm *= 1.0020841800044864E-292;
        forward_kinematics_B.lastc++;
      }

      forward_kinematics_B.alpha1 = forward_kinematics_B.xnorm;
    } else {
      tau[1] = (forward_kinematics_B.xnorm - a[6]) / forward_kinematics_B.xnorm;
      a[7] *= 1.0 / (a[6] - forward_kinematics_B.xnorm);
      forward_kinematics_B.alpha1 = forward_kinematics_B.xnorm;
    }
  }

  a[6] = 1.0;
  if (tau[1] != 0.0) {
    forward_kinematics_B.knt = 1;
    forward_kinematics_B.lastc = 7;
    while ((forward_kinematics_B.knt + 1 > 0) && (a[forward_kinematics_B.lastc] ==
            0.0)) {
      forward_kinematics_B.knt--;
      forward_kinematics_B.lastc--;
    }

    forward_kinematics_B.lastc = 4;
    exitg2 = false;
    while ((!exitg2) && (forward_kinematics_B.lastc > 0)) {
      forward_kinematics_B.rowleft = forward_kinematics_B.lastc + 8;
      forward_kinematics_B.jy = forward_kinematics_B.rowleft;
      do {
        exitg1 = 0;
        if (forward_kinematics_B.jy <= (forward_kinematics_B.knt << 2) +
            forward_kinematics_B.rowleft) {
          if (a[forward_kinematics_B.jy - 1] != 0.0) {
            exitg1 = 1;
          } else {
            forward_kinematics_B.jy += 4;
          }
        } else {
          forward_kinematics_B.lastc--;
          exitg1 = 2;
        }
      } while (exitg1 == 0);

      if (exitg1 == 1) {
        exitg2 = true;
      }
    }
  } else {
    forward_kinematics_B.knt = -1;
    forward_kinematics_B.lastc = 0;
  }

  if (forward_kinematics_B.knt + 1 > 0) {
    if (forward_kinematics_B.lastc != 0) {
      forward_kinematics_B.rowleft = 0;
      while (forward_kinematics_B.rowleft <= forward_kinematics_B.lastc - 1) {
        forward_kinematics_B.work_g[forward_kinematics_B.rowleft] = 0.0;
        forward_kinematics_B.rowleft++;
      }

      forward_kinematics_B.rowleft = 6;
      forward_kinematics_B.jy = (forward_kinematics_B.knt << 2) + 9;
      forward_kinematics_B.iac = 9;
      while (forward_kinematics_B.iac <= forward_kinematics_B.jy) {
        forward_kinematics_B.b_ix = 0;
        forward_kinematics_B.g = (forward_kinematics_B.iac +
          forward_kinematics_B.lastc) - 1;
        forward_kinematics_B.b_ia = forward_kinematics_B.iac;
        while (forward_kinematics_B.b_ia <= forward_kinematics_B.g) {
          forward_kinematics_B.work_g[forward_kinematics_B.b_ix] +=
            a[forward_kinematics_B.b_ia - 1] * a[forward_kinematics_B.rowleft];
          forward_kinematics_B.b_ix++;
          forward_kinematics_B.b_ia++;
        }

        forward_kinematics_B.rowleft++;
        forward_kinematics_B.iac += 4;
      }
    }

    if (!(-tau[1] == 0.0)) {
      forward_kinematics_B.rowleft = 8;
      forward_kinematics_B.jy = 6;
      forward_kinematics_B.iac = 0;
      while (forward_kinematics_B.iac <= forward_kinematics_B.knt) {
        if (a[forward_kinematics_B.jy] != 0.0) {
          forward_kinematics_B.xnorm = a[forward_kinematics_B.jy] * -tau[1];
          forward_kinematics_B.b_ix = 0;
          forward_kinematics_B.g = forward_kinematics_B.lastc +
            forward_kinematics_B.rowleft;
          forward_kinematics_B.b_ia = forward_kinematics_B.rowleft;
          while (forward_kinematics_B.b_ia + 1 <= forward_kinematics_B.g) {
            a[forward_kinematics_B.b_ia] +=
              forward_kinematics_B.work_g[forward_kinematics_B.b_ix] *
              forward_kinematics_B.xnorm;
            forward_kinematics_B.b_ix++;
            forward_kinematics_B.b_ia++;
          }
        }

        forward_kinematics_B.jy++;
        forward_kinematics_B.rowleft += 4;
        forward_kinematics_B.iac++;
      }
    }
  }

  forward_kinematics_xzlarf(2, 2, 7, tau[1], a, 11, forward_kinematics_B.work_g);
  a[6] = forward_kinematics_B.alpha1;
  forward_kinematics_B.alpha1 = a[11];
  tau[2] = 0.0;
  forward_kinematics_B.xnorm = forward_kinematics_xnrm2(0, a, 12);
  if (forward_kinematics_B.xnorm != 0.0) {
    forward_kinematics_B.xnorm = forward_kinematic_rt_hypotd_snf(a[11],
      forward_kinematics_B.xnorm);
    if (a[11] >= 0.0) {
      forward_kinematics_B.xnorm = -forward_kinematics_B.xnorm;
    }

    if (fabs(forward_kinematics_B.xnorm) < 1.0020841800044864E-292) {
      forward_kinematics_B.knt = -1;
      do {
        forward_kinematics_B.knt++;
        forward_kinematics_B.xnorm *= 9.9792015476736E+291;
        forward_kinematics_B.alpha1 *= 9.9792015476736E+291;
      } while (!(fabs(forward_kinematics_B.xnorm) >= 1.0020841800044864E-292));

      forward_kinematics_B.xnorm = forward_kinematic_rt_hypotd_snf
        (forward_kinematics_B.alpha1, forward_kinematics_xnrm2(0, a, 12));
      if (forward_kinematics_B.alpha1 >= 0.0) {
        forward_kinematics_B.xnorm = -forward_kinematics_B.xnorm;
      }

      tau[2] = (forward_kinematics_B.xnorm - forward_kinematics_B.alpha1) /
        forward_kinematics_B.xnorm;
      forward_kinematics_B.lastc = 0;
      while (forward_kinematics_B.lastc <= forward_kinematics_B.knt) {
        forward_kinematics_B.xnorm *= 1.0020841800044864E-292;
        forward_kinematics_B.lastc++;
      }

      forward_kinematics_B.alpha1 = forward_kinematics_B.xnorm;
    } else {
      tau[2] = (forward_kinematics_B.xnorm - a[11]) / forward_kinematics_B.xnorm;
      forward_kinematics_B.alpha1 = forward_kinematics_B.xnorm;
    }
  }

  a[11] = 1.0;
  if (tau[2] != 0.0) {
    forward_kinematics_B.knt = 0;
    forward_kinematics_B.lastc = 11;
    while ((forward_kinematics_B.knt + 1 > 0) && (a[forward_kinematics_B.lastc] ==
            0.0)) {
      forward_kinematics_B.knt--;
      forward_kinematics_B.lastc--;
    }

    forward_kinematics_B.lastc = 4;
    exitg2 = false;
    while ((!exitg2) && (forward_kinematics_B.lastc > 0)) {
      forward_kinematics_B.rowleft = forward_kinematics_B.lastc + 12;
      forward_kinematics_B.jy = forward_kinematics_B.rowleft;
      do {
        exitg1 = 0;
        if (forward_kinematics_B.jy <= (forward_kinematics_B.knt << 2) +
            forward_kinematics_B.rowleft) {
          if (a[forward_kinematics_B.jy - 1] != 0.0) {
            exitg1 = 1;
          } else {
            forward_kinematics_B.jy += 4;
          }
        } else {
          forward_kinematics_B.lastc--;
          exitg1 = 2;
        }
      } while (exitg1 == 0);

      if (exitg1 == 1) {
        exitg2 = true;
      }
    }
  } else {
    forward_kinematics_B.knt = -1;
    forward_kinematics_B.lastc = 0;
  }

  if (forward_kinematics_B.knt + 1 > 0) {
    if (forward_kinematics_B.lastc != 0) {
      forward_kinematics_B.rowleft = 0;
      while (forward_kinematics_B.rowleft <= forward_kinematics_B.lastc - 1) {
        forward_kinematics_B.work_g[forward_kinematics_B.rowleft] = 0.0;
        forward_kinematics_B.rowleft++;
      }

      forward_kinematics_B.rowleft = 11;
      forward_kinematics_B.jy = (forward_kinematics_B.knt << 2) + 13;
      forward_kinematics_B.iac = 13;
      while (forward_kinematics_B.iac <= forward_kinematics_B.jy) {
        forward_kinematics_B.b_ix = 0;
        forward_kinematics_B.g = (forward_kinematics_B.iac +
          forward_kinematics_B.lastc) - 1;
        forward_kinematics_B.b_ia = forward_kinematics_B.iac;
        while (forward_kinematics_B.b_ia <= forward_kinematics_B.g) {
          forward_kinematics_B.work_g[forward_kinematics_B.b_ix] +=
            a[forward_kinematics_B.b_ia - 1] * a[forward_kinematics_B.rowleft];
          forward_kinematics_B.b_ix++;
          forward_kinematics_B.b_ia++;
        }

        forward_kinematics_B.rowleft++;
        forward_kinematics_B.iac += 4;
      }
    }

    if (!(-tau[2] == 0.0)) {
      forward_kinematics_B.rowleft = 12;
      forward_kinematics_B.jy = 11;
      forward_kinematics_B.iac = 0;
      while (forward_kinematics_B.iac <= forward_kinematics_B.knt) {
        if (a[forward_kinematics_B.jy] != 0.0) {
          forward_kinematics_B.xnorm = a[forward_kinematics_B.jy] * -tau[2];
          forward_kinematics_B.b_ix = 0;
          forward_kinematics_B.g = forward_kinematics_B.lastc +
            forward_kinematics_B.rowleft;
          forward_kinematics_B.b_ia = forward_kinematics_B.rowleft;
          while (forward_kinematics_B.b_ia + 1 <= forward_kinematics_B.g) {
            a[forward_kinematics_B.b_ia] +=
              forward_kinematics_B.work_g[forward_kinematics_B.b_ix] *
              forward_kinematics_B.xnorm;
            forward_kinematics_B.b_ix++;
            forward_kinematics_B.b_ia++;
          }
        }

        forward_kinematics_B.jy++;
        forward_kinematics_B.rowleft += 4;
        forward_kinematics_B.iac++;
      }
    }
  }

  forward_kinematics_xzlarf(1, 1, 12, tau[2], a, 16, forward_kinematics_B.work_g);
  a[11] = forward_kinematics_B.alpha1;
}

// Function for MATLAB Function: '<S1>/MATLAB Function'
static real_T forward_kinematics_xnrm2_k(int32_T n, const real_T x[3])
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

// Function for MATLAB Function: '<S1>/MATLAB Function'
static real_T forward_kinematics_xzlarfg(int32_T n, real_T *alpha1, real_T x[3])
{
  real_T tau;
  real_T xnorm;
  int32_T knt;
  int32_T c_k;
  tau = 0.0;
  if (n > 0) {
    xnorm = forward_kinematics_xnrm2_k(n - 1, x);
    if (xnorm != 0.0) {
      xnorm = forward_kinematic_rt_hypotd_snf(*alpha1, xnorm);
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

        xnorm = forward_kinematic_rt_hypotd_snf(*alpha1,
          forward_kinematics_xnrm2_k(n - 1, x));
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

// Function for MATLAB Function: '<S1>/MATLAB Function'
static void forward_kinematics_xdlanv2(real_T *a, real_T *b, real_T *c, real_T
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
    forward_kinematics_B.bcmax = *d;
    *d = *a;
    *a = forward_kinematics_B.bcmax;
    *b = -*c;
    *c = 0.0;
  } else {
    forward_kinematics_B.tau_h = *a - *d;
    if ((forward_kinematics_B.tau_h == 0.0) && ((*b < 0.0) != (*c < 0.0))) {
      *cs = 1.0;
      *sn = 0.0;
    } else {
      forward_kinematics_B.p = 0.5 * forward_kinematics_B.tau_h;
      forward_kinematics_B.bcmis = fabs(*b);
      forward_kinematics_B.z = fabs(*c);
      tmp = rtIsNaN(forward_kinematics_B.z);
      if ((forward_kinematics_B.bcmis > forward_kinematics_B.z) || tmp) {
        forward_kinematics_B.bcmax = forward_kinematics_B.bcmis;
      } else {
        forward_kinematics_B.bcmax = forward_kinematics_B.z;
      }

      if ((forward_kinematics_B.bcmis < forward_kinematics_B.z) || tmp) {
        forward_kinematics_B.z = forward_kinematics_B.bcmis;
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

      forward_kinematics_B.bcmis = forward_kinematics_B.z * static_cast<real_T>
        (b_0) * static_cast<real_T>(c_0);
      forward_kinematics_B.scale_j = fabs(forward_kinematics_B.p);
      if ((!(forward_kinematics_B.scale_j > forward_kinematics_B.bcmax)) &&
          (!rtIsNaN(forward_kinematics_B.bcmax))) {
        forward_kinematics_B.scale_j = forward_kinematics_B.bcmax;
      }

      forward_kinematics_B.z = forward_kinematics_B.p /
        forward_kinematics_B.scale_j * forward_kinematics_B.p +
        forward_kinematics_B.bcmax / forward_kinematics_B.scale_j *
        forward_kinematics_B.bcmis;
      if (forward_kinematics_B.z >= 8.8817841970012523E-16) {
        if (!(forward_kinematics_B.p < 0.0)) {
          forward_kinematics_B.tau_h = sqrt(forward_kinematics_B.scale_j) * sqrt
            (forward_kinematics_B.z);
        } else {
          forward_kinematics_B.tau_h = -(sqrt(forward_kinematics_B.scale_j) *
            sqrt(forward_kinematics_B.z));
        }

        forward_kinematics_B.z = forward_kinematics_B.p +
          forward_kinematics_B.tau_h;
        *a = *d + forward_kinematics_B.z;
        *d -= forward_kinematics_B.bcmax / forward_kinematics_B.z *
          forward_kinematics_B.bcmis;
        forward_kinematics_B.tau_h = forward_kinematic_rt_hypotd_snf(*c,
          forward_kinematics_B.z);
        *cs = forward_kinematics_B.z / forward_kinematics_B.tau_h;
        *sn = *c / forward_kinematics_B.tau_h;
        *b -= *c;
        *c = 0.0;
      } else {
        forward_kinematics_B.bcmax = *b + *c;
        forward_kinematics_B.tau_h = forward_kinematic_rt_hypotd_snf
          (forward_kinematics_B.bcmax, forward_kinematics_B.tau_h);
        *cs = sqrt((fabs(forward_kinematics_B.bcmax) /
                    forward_kinematics_B.tau_h + 1.0) * 0.5);
        if (!(forward_kinematics_B.bcmax < 0.0)) {
          b_0 = 1;
        } else {
          b_0 = -1;
        }

        *sn = -(forward_kinematics_B.p / (forward_kinematics_B.tau_h * *cs)) *
          static_cast<real_T>(b_0);
        forward_kinematics_B.p = *a * *cs + *b * *sn;
        forward_kinematics_B.tau_h = -*a * *sn + *b * *cs;
        forward_kinematics_B.bcmax = *c * *cs + *d * *sn;
        forward_kinematics_B.bcmis = -*c * *sn + *d * *cs;
        *b = forward_kinematics_B.tau_h * *cs + forward_kinematics_B.bcmis * *sn;
        *c = -forward_kinematics_B.p * *sn + forward_kinematics_B.bcmax * *cs;
        forward_kinematics_B.bcmax = ((forward_kinematics_B.p * *cs +
          forward_kinematics_B.bcmax * *sn) + (-forward_kinematics_B.tau_h * *sn
          + forward_kinematics_B.bcmis * *cs)) * 0.5;
        *a = forward_kinematics_B.bcmax;
        *d = forward_kinematics_B.bcmax;
        if (*c != 0.0) {
          if (*b != 0.0) {
            if ((*b < 0.0) == (*c < 0.0)) {
              forward_kinematics_B.z = sqrt(fabs(*b));
              forward_kinematics_B.bcmis = sqrt(fabs(*c));
              if (!(*c < 0.0)) {
                forward_kinematics_B.p = forward_kinematics_B.z *
                  forward_kinematics_B.bcmis;
              } else {
                forward_kinematics_B.p = -(forward_kinematics_B.z *
                  forward_kinematics_B.bcmis);
              }

              forward_kinematics_B.tau_h = 1.0 / sqrt(fabs(*b + *c));
              *a = forward_kinematics_B.bcmax + forward_kinematics_B.p;
              *d = forward_kinematics_B.bcmax - forward_kinematics_B.p;
              *b -= *c;
              *c = 0.0;
              forward_kinematics_B.p = forward_kinematics_B.z *
                forward_kinematics_B.tau_h;
              forward_kinematics_B.tau_h *= forward_kinematics_B.bcmis;
              forward_kinematics_B.bcmax = *cs * forward_kinematics_B.p - *sn *
                forward_kinematics_B.tau_h;
              *sn = *cs * forward_kinematics_B.tau_h + *sn *
                forward_kinematics_B.p;
              *cs = forward_kinematics_B.bcmax;
            }
          } else {
            *b = -*c;
            *c = 0.0;
            forward_kinematics_B.bcmax = *cs;
            *cs = -*sn;
            *sn = forward_kinematics_B.bcmax;
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

// Function for MATLAB Function: '<S1>/MATLAB Function'
static void forward_kinematics_xrot(int32_T n, real_T x[16], int32_T ix0,
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

// Function for MATLAB Function: '<S1>/MATLAB Function'
static void forward_kinematics_xrot_m(int32_T n, real_T x[16], int32_T ix0,
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

// Function for MATLAB Function: '<S1>/MATLAB Function'
static int32_T forward_kinematics_eml_dlahqr(real_T h[16], real_T z[16])
{
  int32_T info;
  boolean_T goto150;
  int32_T s_tmp;
  boolean_T exitg1;
  boolean_T exitg2;
  boolean_T exitg3;
  info = 0;
  forward_kinematics_B.v_l[0] = 0.0;
  forward_kinematics_B.v_l[1] = 0.0;
  forward_kinematics_B.v_l[2] = 0.0;
  h[2] = 0.0;
  h[3] = 0.0;
  h[7] = 0.0;
  forward_kinematics_B.i_p = 3;
  exitg1 = false;
  while ((!exitg1) && (forward_kinematics_B.i_p + 1 >= 1)) {
    forward_kinematics_B.L = 1;
    goto150 = false;
    forward_kinematics_B.ix = 0;
    exitg2 = false;
    while ((!exitg2) && (forward_kinematics_B.ix < 301)) {
      forward_kinematics_B.k_a = forward_kinematics_B.i_p;
      exitg3 = false;
      while ((!exitg3) && (forward_kinematics_B.k_a + 1 > forward_kinematics_B.L))
      {
        s_tmp = ((forward_kinematics_B.k_a - 1) << 2) + forward_kinematics_B.k_a;
        forward_kinematics_B.htmp2 = fabs(h[s_tmp]);
        if (forward_kinematics_B.htmp2 <= 4.0083367200179456E-292) {
          exitg3 = true;
        } else {
          forward_kinematics_B.m = (forward_kinematics_B.k_a << 2) +
            forward_kinematics_B.k_a;
          forward_kinematics_B.nr = s_tmp - 1;
          forward_kinematics_B.tst = fabs(h[forward_kinematics_B.nr]) + fabs
            (h[forward_kinematics_B.m]);
          if (forward_kinematics_B.tst == 0.0) {
            if (forward_kinematics_B.k_a - 1 >= 1) {
              forward_kinematics_B.tst = fabs(h[(((forward_kinematics_B.k_a - 2)
                << 2) + forward_kinematics_B.k_a) - 1]);
            }

            if (forward_kinematics_B.k_a + 2 <= 4) {
              forward_kinematics_B.tst += fabs(h[((forward_kinematics_B.k_a << 2)
                + forward_kinematics_B.k_a) + 1]);
            }
          }

          if (forward_kinematics_B.htmp2 <= 2.2204460492503131E-16 *
              forward_kinematics_B.tst) {
            forward_kinematics_B.htmp1 = fabs(h[s_tmp]);
            forward_kinematics_B.htmp2 = fabs(h[forward_kinematics_B.m - 1]);
            if (forward_kinematics_B.htmp1 > forward_kinematics_B.htmp2) {
              forward_kinematics_B.tst = forward_kinematics_B.htmp1;
              forward_kinematics_B.ba = forward_kinematics_B.htmp2;
            } else {
              forward_kinematics_B.tst = forward_kinematics_B.htmp2;
              forward_kinematics_B.ba = forward_kinematics_B.htmp1;
            }

            forward_kinematics_B.htmp1 = fabs(h[forward_kinematics_B.m]);
            forward_kinematics_B.htmp2 = fabs(h[forward_kinematics_B.nr] -
              h[forward_kinematics_B.m]);
            if (forward_kinematics_B.htmp1 > forward_kinematics_B.htmp2) {
              forward_kinematics_B.aa = forward_kinematics_B.htmp1;
              forward_kinematics_B.htmp1 = forward_kinematics_B.htmp2;
            } else {
              forward_kinematics_B.aa = forward_kinematics_B.htmp2;
            }

            forward_kinematics_B.htmp2 = forward_kinematics_B.aa +
              forward_kinematics_B.tst;
            forward_kinematics_B.htmp1 = forward_kinematics_B.aa /
              forward_kinematics_B.htmp2 * forward_kinematics_B.htmp1 *
              2.2204460492503131E-16;
            if ((4.0083367200179456E-292 > forward_kinematics_B.htmp1) ||
                rtIsNaN(forward_kinematics_B.htmp1)) {
              forward_kinematics_B.htmp1 = 4.0083367200179456E-292;
            }

            if (forward_kinematics_B.tst / forward_kinematics_B.htmp2 *
                forward_kinematics_B.ba <= forward_kinematics_B.htmp1) {
              exitg3 = true;
            } else {
              forward_kinematics_B.k_a--;
            }
          } else {
            forward_kinematics_B.k_a--;
          }
        }
      }

      forward_kinematics_B.L = forward_kinematics_B.k_a + 1;
      if (forward_kinematics_B.k_a + 1 > 1) {
        h[forward_kinematics_B.k_a + ((forward_kinematics_B.k_a - 1) << 2)] =
          0.0;
      }

      if (forward_kinematics_B.k_a + 1 >= forward_kinematics_B.i_p) {
        goto150 = true;
        exitg2 = true;
      } else {
        switch (forward_kinematics_B.ix) {
         case 10:
          s_tmp = (forward_kinematics_B.k_a << 2) + forward_kinematics_B.k_a;
          forward_kinematics_B.htmp2 = fabs(h[(((forward_kinematics_B.k_a + 1) <<
            2) + forward_kinematics_B.k_a) + 2]) + fabs(h[s_tmp + 1]);
          forward_kinematics_B.ba = h[s_tmp] + 0.75 * forward_kinematics_B.htmp2;
          forward_kinematics_B.h12 = -0.4375 * forward_kinematics_B.htmp2;
          forward_kinematics_B.aa = forward_kinematics_B.htmp2;
          forward_kinematics_B.tst = forward_kinematics_B.ba;
          break;

         case 20:
          forward_kinematics_B.htmp2 = fabs(h[(((forward_kinematics_B.i_p - 2) <<
            2) + forward_kinematics_B.i_p) - 1]) + fabs(h
            [((forward_kinematics_B.i_p - 1) << 2) + forward_kinematics_B.i_p]);
          forward_kinematics_B.ba = h[(forward_kinematics_B.i_p << 2) +
            forward_kinematics_B.i_p] + 0.75 * forward_kinematics_B.htmp2;
          forward_kinematics_B.h12 = -0.4375 * forward_kinematics_B.htmp2;
          forward_kinematics_B.aa = forward_kinematics_B.htmp2;
          forward_kinematics_B.tst = forward_kinematics_B.ba;
          break;

         default:
          forward_kinematics_B.ba = h[(((forward_kinematics_B.i_p - 1) << 2) +
            forward_kinematics_B.i_p) - 1];
          forward_kinematics_B.aa = h[((forward_kinematics_B.i_p - 1) << 2) +
            forward_kinematics_B.i_p];
          forward_kinematics_B.h12 = h[((forward_kinematics_B.i_p << 2) +
            forward_kinematics_B.i_p) - 1];
          forward_kinematics_B.tst = h[(forward_kinematics_B.i_p << 2) +
            forward_kinematics_B.i_p];
          break;
        }

        forward_kinematics_B.htmp2 = ((fabs(forward_kinematics_B.ba) + fabs
          (forward_kinematics_B.h12)) + fabs(forward_kinematics_B.aa)) + fabs
          (forward_kinematics_B.tst);
        if (forward_kinematics_B.htmp2 == 0.0) {
          forward_kinematics_B.ba = 0.0;
          forward_kinematics_B.tst = 0.0;
          forward_kinematics_B.htmp1 = 0.0;
          forward_kinematics_B.aa = 0.0;
        } else {
          forward_kinematics_B.ba /= forward_kinematics_B.htmp2;
          forward_kinematics_B.aa /= forward_kinematics_B.htmp2;
          forward_kinematics_B.h12 /= forward_kinematics_B.htmp2;
          forward_kinematics_B.tst /= forward_kinematics_B.htmp2;
          forward_kinematics_B.htmp1 = (forward_kinematics_B.ba +
            forward_kinematics_B.tst) / 2.0;
          forward_kinematics_B.ba = (forward_kinematics_B.ba -
            forward_kinematics_B.htmp1) * (forward_kinematics_B.tst -
            forward_kinematics_B.htmp1) - forward_kinematics_B.h12 *
            forward_kinematics_B.aa;
          forward_kinematics_B.aa = sqrt(fabs(forward_kinematics_B.ba));
          if (forward_kinematics_B.ba >= 0.0) {
            forward_kinematics_B.ba = forward_kinematics_B.htmp1 *
              forward_kinematics_B.htmp2;
            forward_kinematics_B.htmp1 = forward_kinematics_B.ba;
            forward_kinematics_B.tst = forward_kinematics_B.aa *
              forward_kinematics_B.htmp2;
            forward_kinematics_B.aa = -forward_kinematics_B.tst;
          } else {
            forward_kinematics_B.ba = forward_kinematics_B.htmp1 +
              forward_kinematics_B.aa;
            forward_kinematics_B.htmp1 -= forward_kinematics_B.aa;
            if (fabs(forward_kinematics_B.ba - forward_kinematics_B.tst) <= fabs
                (forward_kinematics_B.htmp1 - forward_kinematics_B.tst)) {
              forward_kinematics_B.ba *= forward_kinematics_B.htmp2;
              forward_kinematics_B.htmp1 = forward_kinematics_B.ba;
            } else {
              forward_kinematics_B.htmp1 *= forward_kinematics_B.htmp2;
              forward_kinematics_B.ba = forward_kinematics_B.htmp1;
            }

            forward_kinematics_B.tst = 0.0;
            forward_kinematics_B.aa = 0.0;
          }
        }

        forward_kinematics_B.m = forward_kinematics_B.i_p - 1;
        exitg3 = false;
        while ((!exitg3) && (forward_kinematics_B.m >= forward_kinematics_B.k_a
                             + 1)) {
          s_tmp = ((forward_kinematics_B.m - 1) << 2) + forward_kinematics_B.m;
          forward_kinematics_B.nr = s_tmp - 1;
          forward_kinematics_B.h12 = h[forward_kinematics_B.nr] -
            forward_kinematics_B.htmp1;
          forward_kinematics_B.htmp2 = (fabs(forward_kinematics_B.h12) + fabs
            (forward_kinematics_B.aa)) + fabs(h[s_tmp]);
          forward_kinematics_B.h21s = h[s_tmp] / forward_kinematics_B.htmp2;
          forward_kinematics_B.hoffset = (forward_kinematics_B.m << 2) +
            forward_kinematics_B.m;
          forward_kinematics_B.v_l[0] = (forward_kinematics_B.h12 /
            forward_kinematics_B.htmp2 * (h[forward_kinematics_B.nr] -
            forward_kinematics_B.ba) + h[forward_kinematics_B.hoffset - 1] *
            forward_kinematics_B.h21s) - forward_kinematics_B.aa /
            forward_kinematics_B.htmp2 * forward_kinematics_B.tst;
          forward_kinematics_B.v_l[1] = (((h[forward_kinematics_B.nr] +
            h[forward_kinematics_B.hoffset]) - forward_kinematics_B.ba) -
            forward_kinematics_B.htmp1) * forward_kinematics_B.h21s;
          forward_kinematics_B.v_l[2] = h[forward_kinematics_B.hoffset + 1] *
            forward_kinematics_B.h21s;
          forward_kinematics_B.htmp2 = (fabs(forward_kinematics_B.v_l[0]) + fabs
            (forward_kinematics_B.v_l[1])) + fabs(forward_kinematics_B.v_l[2]);
          forward_kinematics_B.v_l[0] /= forward_kinematics_B.htmp2;
          forward_kinematics_B.v_l[1] /= forward_kinematics_B.htmp2;
          forward_kinematics_B.v_l[2] /= forward_kinematics_B.htmp2;
          if (forward_kinematics_B.k_a + 1 == forward_kinematics_B.m) {
            exitg3 = true;
          } else {
            s_tmp = ((forward_kinematics_B.m - 2) << 2) + forward_kinematics_B.m;
            if (fabs(h[s_tmp - 1]) * (fabs(forward_kinematics_B.v_l[1]) + fabs
                 (forward_kinematics_B.v_l[2])) <= ((fabs(h[s_tmp - 2]) + fabs
                  (h[forward_kinematics_B.nr])) + fabs
                 (h[forward_kinematics_B.hoffset])) * (2.2204460492503131E-16 *
                 fabs(forward_kinematics_B.v_l[0]))) {
              exitg3 = true;
            } else {
              forward_kinematics_B.m--;
            }
          }
        }

        for (s_tmp = forward_kinematics_B.m; s_tmp <= forward_kinematics_B.i_p;
             s_tmp++) {
          forward_kinematics_B.nr = (forward_kinematics_B.i_p - s_tmp) + 2;
          if (3 < forward_kinematics_B.nr) {
            forward_kinematics_B.nr = 3;
          }

          if (s_tmp > forward_kinematics_B.m) {
            forward_kinematics_B.hoffset = ((s_tmp - 2) << 2) + s_tmp;
            forward_kinematics_B.j_j = 0;
            while (forward_kinematics_B.j_j <= forward_kinematics_B.nr - 1) {
              forward_kinematics_B.v_l[forward_kinematics_B.j_j] = h
                [(forward_kinematics_B.j_j + forward_kinematics_B.hoffset) - 1];
              forward_kinematics_B.j_j++;
            }
          }

          forward_kinematics_B.tst = forward_kinematics_B.v_l[0];
          forward_kinematics_B.b_v[0] = forward_kinematics_B.v_l[0];
          forward_kinematics_B.b_v[1] = forward_kinematics_B.v_l[1];
          forward_kinematics_B.b_v[2] = forward_kinematics_B.v_l[2];
          forward_kinematics_B.htmp2 = forward_kinematics_xzlarfg
            (forward_kinematics_B.nr, &forward_kinematics_B.tst,
             forward_kinematics_B.b_v);
          forward_kinematics_B.v_l[1] = forward_kinematics_B.b_v[1];
          forward_kinematics_B.v_l[2] = forward_kinematics_B.b_v[2];
          forward_kinematics_B.v_l[0] = forward_kinematics_B.tst;
          if (s_tmp > forward_kinematics_B.m) {
            h[(s_tmp + ((s_tmp - 2) << 2)) - 1] = forward_kinematics_B.tst;
            h[s_tmp + ((s_tmp - 2) << 2)] = 0.0;
            if (s_tmp < forward_kinematics_B.i_p) {
              h[s_tmp + 1] = 0.0;
            }
          } else {
            if (forward_kinematics_B.m > forward_kinematics_B.k_a + 1) {
              h[s_tmp - 1] *= 1.0 - forward_kinematics_B.htmp2;
            }
          }

          forward_kinematics_B.tst = forward_kinematics_B.b_v[1];
          forward_kinematics_B.ba = forward_kinematics_B.htmp2 *
            forward_kinematics_B.b_v[1];
          switch (forward_kinematics_B.nr) {
           case 3:
            forward_kinematics_B.htmp1 = forward_kinematics_B.b_v[2];
            forward_kinematics_B.aa = forward_kinematics_B.htmp2 *
              forward_kinematics_B.b_v[2];
            forward_kinematics_B.b_j_e = s_tmp - 1;
            while (forward_kinematics_B.b_j_e + 1 < 5) {
              forward_kinematics_B.nr = (forward_kinematics_B.b_j_e << 2) +
                s_tmp;
              forward_kinematics_B.hoffset = forward_kinematics_B.nr - 1;
              forward_kinematics_B.j_j = forward_kinematics_B.nr + 1;
              forward_kinematics_B.h12 = (h[forward_kinematics_B.hoffset] +
                h[forward_kinematics_B.nr] * forward_kinematics_B.tst) +
                h[forward_kinematics_B.j_j] * forward_kinematics_B.htmp1;
              h[forward_kinematics_B.hoffset] -= forward_kinematics_B.h12 *
                forward_kinematics_B.htmp2;
              h[forward_kinematics_B.nr] -= forward_kinematics_B.h12 *
                forward_kinematics_B.ba;
              h[forward_kinematics_B.j_j] -= forward_kinematics_B.h12 *
                forward_kinematics_B.aa;
              forward_kinematics_B.b_j_e++;
            }

            forward_kinematics_B.nr = s_tmp + 3;
            forward_kinematics_B.b_j_e = forward_kinematics_B.i_p + 1;
            if (forward_kinematics_B.nr < forward_kinematics_B.b_j_e) {
              forward_kinematics_B.b_j_e = forward_kinematics_B.nr;
            }

            forward_kinematics_B.c_j = 0;
            while (forward_kinematics_B.c_j <= forward_kinematics_B.b_j_e - 1) {
              forward_kinematics_B.nr = ((s_tmp - 1) << 2) +
                forward_kinematics_B.c_j;
              forward_kinematics_B.hoffset = (s_tmp << 2) +
                forward_kinematics_B.c_j;
              forward_kinematics_B.j_j = ((s_tmp + 1) << 2) +
                forward_kinematics_B.c_j;
              forward_kinematics_B.h12 = (h[forward_kinematics_B.nr] +
                h[forward_kinematics_B.hoffset] * forward_kinematics_B.tst) +
                h[forward_kinematics_B.j_j] * forward_kinematics_B.htmp1;
              h[forward_kinematics_B.nr] -= forward_kinematics_B.h12 *
                forward_kinematics_B.htmp2;
              h[forward_kinematics_B.hoffset] -= forward_kinematics_B.h12 *
                forward_kinematics_B.ba;
              h[forward_kinematics_B.j_j] -= forward_kinematics_B.h12 *
                forward_kinematics_B.aa;
              forward_kinematics_B.c_j++;
            }

            for (forward_kinematics_B.b_j_e = 0; forward_kinematics_B.b_j_e < 4;
                 forward_kinematics_B.b_j_e++) {
              forward_kinematics_B.nr = ((s_tmp - 1) << 2) +
                forward_kinematics_B.b_j_e;
              forward_kinematics_B.hoffset = (s_tmp << 2) +
                forward_kinematics_B.b_j_e;
              forward_kinematics_B.j_j = ((s_tmp + 1) << 2) +
                forward_kinematics_B.b_j_e;
              forward_kinematics_B.h12 = (z[forward_kinematics_B.nr] +
                z[forward_kinematics_B.hoffset] * forward_kinematics_B.tst) +
                z[forward_kinematics_B.j_j] * forward_kinematics_B.htmp1;
              z[forward_kinematics_B.nr] -= forward_kinematics_B.h12 *
                forward_kinematics_B.htmp2;
              z[forward_kinematics_B.hoffset] -= forward_kinematics_B.h12 *
                forward_kinematics_B.ba;
              z[forward_kinematics_B.j_j] -= forward_kinematics_B.h12 *
                forward_kinematics_B.aa;
            }
            break;

           case 2:
            forward_kinematics_B.j_j = s_tmp - 1;
            while (forward_kinematics_B.j_j + 1 < 5) {
              forward_kinematics_B.nr = (forward_kinematics_B.j_j << 2) + s_tmp;
              forward_kinematics_B.hoffset = forward_kinematics_B.nr - 1;
              forward_kinematics_B.h12 = h[forward_kinematics_B.hoffset] +
                h[forward_kinematics_B.nr] * forward_kinematics_B.tst;
              h[forward_kinematics_B.hoffset] -= forward_kinematics_B.h12 *
                forward_kinematics_B.htmp2;
              h[forward_kinematics_B.nr] -= forward_kinematics_B.h12 *
                forward_kinematics_B.ba;
              forward_kinematics_B.j_j++;
            }

            forward_kinematics_B.j_j = 0;
            while (forward_kinematics_B.j_j <= forward_kinematics_B.i_p) {
              forward_kinematics_B.nr = ((s_tmp - 1) << 2) +
                forward_kinematics_B.j_j;
              forward_kinematics_B.hoffset = (s_tmp << 2) +
                forward_kinematics_B.j_j;
              forward_kinematics_B.h12 = h[forward_kinematics_B.nr] +
                h[forward_kinematics_B.hoffset] * forward_kinematics_B.tst;
              h[forward_kinematics_B.nr] -= forward_kinematics_B.h12 *
                forward_kinematics_B.htmp2;
              h[forward_kinematics_B.hoffset] -= forward_kinematics_B.h12 *
                forward_kinematics_B.ba;
              forward_kinematics_B.j_j++;
            }

            for (forward_kinematics_B.j_j = 0; forward_kinematics_B.j_j < 4;
                 forward_kinematics_B.j_j++) {
              forward_kinematics_B.nr = ((s_tmp - 1) << 2) +
                forward_kinematics_B.j_j;
              forward_kinematics_B.hoffset = (s_tmp << 2) +
                forward_kinematics_B.j_j;
              forward_kinematics_B.h12 = z[forward_kinematics_B.nr] +
                z[forward_kinematics_B.hoffset] * forward_kinematics_B.tst;
              z[forward_kinematics_B.nr] -= forward_kinematics_B.h12 *
                forward_kinematics_B.htmp2;
              z[forward_kinematics_B.hoffset] -= forward_kinematics_B.h12 *
                forward_kinematics_B.ba;
            }
            break;
          }
        }

        forward_kinematics_B.ix++;
      }
    }

    if (!goto150) {
      info = forward_kinematics_B.i_p + 1;
      exitg1 = true;
    } else {
      if ((forward_kinematics_B.i_p + 1 != forward_kinematics_B.L) &&
          (forward_kinematics_B.L == forward_kinematics_B.i_p)) {
        forward_kinematics_B.ix = (forward_kinematics_B.i_p - 1) << 2;
        forward_kinematics_B.k_a = forward_kinematics_B.ix +
          forward_kinematics_B.i_p;
        forward_kinematics_B.m = forward_kinematics_B.k_a - 1;
        forward_kinematics_B.ba = h[forward_kinematics_B.m];
        s_tmp = forward_kinematics_B.i_p << 2;
        forward_kinematics_B.nr = s_tmp + forward_kinematics_B.i_p;
        forward_kinematics_B.hoffset = forward_kinematics_B.nr - 1;
        forward_kinematics_B.htmp1 = h[forward_kinematics_B.hoffset];
        forward_kinematics_B.aa = h[forward_kinematics_B.k_a];
        forward_kinematics_B.h12 = h[forward_kinematics_B.nr];
        forward_kinematics_xdlanv2(&forward_kinematics_B.ba,
          &forward_kinematics_B.htmp1, &forward_kinematics_B.aa,
          &forward_kinematics_B.h12, &forward_kinematics_B.h21s,
          &forward_kinematics_B.unusedU1, &forward_kinematics_B.unusedU2,
          &forward_kinematics_B.unusedU3, &forward_kinematics_B.htmp2,
          &forward_kinematics_B.tst);
        h[forward_kinematics_B.m] = forward_kinematics_B.ba;
        h[forward_kinematics_B.hoffset] = forward_kinematics_B.htmp1;
        h[forward_kinematics_B.k_a] = forward_kinematics_B.aa;
        h[forward_kinematics_B.nr] = forward_kinematics_B.h12;
        if (4 > forward_kinematics_B.i_p + 1) {
          forward_kinematics_xrot(3 - forward_kinematics_B.i_p, h,
            forward_kinematics_B.i_p + ((forward_kinematics_B.i_p + 1) << 2),
            (forward_kinematics_B.i_p + ((forward_kinematics_B.i_p + 1) << 2)) +
            1, forward_kinematics_B.htmp2, forward_kinematics_B.tst);
        }

        forward_kinematics_xrot_m(forward_kinematics_B.i_p - 1, h,
          ((forward_kinematics_B.i_p - 1) << 2) + 1, (forward_kinematics_B.i_p <<
          2) + 1, forward_kinematics_B.htmp2, forward_kinematics_B.tst);
        forward_kinematics_B.ba = forward_kinematics_B.htmp2 *
          z[forward_kinematics_B.ix] + forward_kinematics_B.tst * z[s_tmp];
        z[s_tmp] = forward_kinematics_B.htmp2 * z[s_tmp] -
          forward_kinematics_B.tst * z[forward_kinematics_B.ix];
        z[forward_kinematics_B.ix] = forward_kinematics_B.ba;
        forward_kinematics_B.i_p = s_tmp + 1;
        forward_kinematics_B.ix++;
        forward_kinematics_B.ba = forward_kinematics_B.htmp2 *
          z[forward_kinematics_B.ix] + forward_kinematics_B.tst *
          z[forward_kinematics_B.i_p];
        z[forward_kinematics_B.i_p] = forward_kinematics_B.htmp2 *
          z[forward_kinematics_B.i_p] - forward_kinematics_B.tst *
          z[forward_kinematics_B.ix];
        z[forward_kinematics_B.ix] = forward_kinematics_B.ba;
        forward_kinematics_B.i_p++;
        forward_kinematics_B.ix++;
        forward_kinematics_B.ba = forward_kinematics_B.htmp2 *
          z[forward_kinematics_B.ix] + forward_kinematics_B.tst *
          z[forward_kinematics_B.i_p];
        z[forward_kinematics_B.i_p] = forward_kinematics_B.htmp2 *
          z[forward_kinematics_B.i_p] - forward_kinematics_B.tst *
          z[forward_kinematics_B.ix];
        z[forward_kinematics_B.ix] = forward_kinematics_B.ba;
        forward_kinematics_B.i_p++;
        forward_kinematics_B.ix++;
        forward_kinematics_B.ba = forward_kinematics_B.htmp2 *
          z[forward_kinematics_B.ix] + forward_kinematics_B.tst *
          z[forward_kinematics_B.i_p];
        z[forward_kinematics_B.i_p] = forward_kinematics_B.htmp2 *
          z[forward_kinematics_B.i_p] - forward_kinematics_B.tst *
          z[forward_kinematics_B.ix];
        z[forward_kinematics_B.ix] = forward_kinematics_B.ba;
      }

      forward_kinematics_B.i_p = forward_kinematics_B.L - 2;
    }
  }

  return info;
}

// Function for MATLAB Function: '<S1>/MATLAB Function'
static void forward_kinematics_eig(const real_T A[16], creal_T V[16], creal_T D
  [4])
{
  boolean_T p;
  int32_T exitg1;
  boolean_T exitg2;
  if (forward_kinematics_anyNonFinite(A)) {
    for (forward_kinematics_B.b_j = 0; forward_kinematics_B.b_j < 16;
         forward_kinematics_B.b_j++) {
      V[forward_kinematics_B.b_j].re = (rtNaN);
      V[forward_kinematics_B.b_j].im = 0.0;
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
    forward_kinematics_B.b_j = 0;
    exitg2 = false;
    while ((!exitg2) && (forward_kinematics_B.b_j < 4)) {
      forward_kinematics_B.i = 0;
      do {
        exitg1 = 0;
        if (forward_kinematics_B.i <= forward_kinematics_B.b_j) {
          if (!(A[(forward_kinematics_B.b_j << 2) + forward_kinematics_B.i] ==
                A[(forward_kinematics_B.i << 2) + forward_kinematics_B.b_j])) {
            p = false;
            exitg1 = 1;
          } else {
            forward_kinematics_B.i++;
          }
        } else {
          forward_kinematics_B.b_j++;
          exitg1 = 2;
        }
      } while (exitg1 == 0);

      if (exitg1 == 1) {
        exitg2 = true;
      }
    }

    if (p) {
      if (forward_kinematics_anyNonFinite(A)) {
        for (forward_kinematics_B.b_j = 0; forward_kinematics_B.b_j < 16;
             forward_kinematics_B.b_j++) {
          forward_kinematics_B.b_V[forward_kinematics_B.b_j] = (rtNaN);
        }

        forward_kinematics_B.b_j = 2;
        while (forward_kinematics_B.b_j < 5) {
          forward_kinematics_B.b_V[forward_kinematics_B.b_j - 1] = 0.0;
          forward_kinematics_B.b_j++;
        }

        forward_kinematics_B.b_j = 3;
        while (forward_kinematics_B.b_j < 5) {
          forward_kinematics_B.b_V[forward_kinematics_B.b_j + 3] = 0.0;
          forward_kinematics_B.b_j++;
        }

        forward_kinematics_B.b_V[11] = 0.0;
        for (forward_kinematics_B.b_j = 0; forward_kinematics_B.b_j < 16;
             forward_kinematics_B.b_j++) {
          forward_kinematics_B.b_A[forward_kinematics_B.b_j] = (rtNaN);
        }
      } else {
        memcpy(&forward_kinematics_B.b_A[0], &A[0], sizeof(real_T) << 4U);
        forward_kinematics_xgehrd(forward_kinematics_B.b_A,
          forward_kinematics_B.tau);
        memcpy(&forward_kinematics_B.b_V[0], &forward_kinematics_B.b_A[0],
               sizeof(real_T) << 4U);
        forward_kinematics_B.b_j = 0;
        while (forward_kinematics_B.b_j <= 2) {
          forward_kinematics_B.b_V[forward_kinematics_B.b_j + 12] = 0.0;
          forward_kinematics_B.b_j++;
        }

        forward_kinematics_B.b_j = 0;
        while (forward_kinematics_B.b_j <= 1) {
          forward_kinematics_B.b_V[forward_kinematics_B.b_j + 8] = 0.0;
          forward_kinematics_B.b_j++;
        }

        forward_kinematics_B.b_j = 1;
        while (forward_kinematics_B.b_j + 3 < 5) {
          forward_kinematics_B.b_V[forward_kinematics_B.b_j + 10] =
            forward_kinematics_B.b_V[forward_kinematics_B.b_j + 6];
          forward_kinematics_B.b_j++;
        }

        forward_kinematics_B.b_V[4] = 0.0;
        forward_kinematics_B.b_j = 0;
        while (forward_kinematics_B.b_j + 3 < 5) {
          forward_kinematics_B.b_V[forward_kinematics_B.b_j + 6] =
            forward_kinematics_B.b_V[forward_kinematics_B.b_j + 2];
          forward_kinematics_B.b_j++;
        }

        forward_kinematics_B.work[0] = 0.0;
        forward_kinematics_B.b_V[1] = 0.0;
        forward_kinematics_B.work[1] = 0.0;
        forward_kinematics_B.b_V[2] = 0.0;
        forward_kinematics_B.work[2] = 0.0;
        forward_kinematics_B.b_V[3] = 0.0;
        forward_kinematics_B.work[3] = 0.0;
        forward_kinematics_B.b_V[0] = 1.0;
        forward_kinematics_B.b_V[15] = 1.0 - forward_kinematics_B.tau[2];
        forward_kinematics_B.b_j = 0;
        while (forward_kinematics_B.b_j <= 1) {
          forward_kinematics_B.b_V[14 - forward_kinematics_B.b_j] = 0.0;
          forward_kinematics_B.b_j++;
        }

        forward_kinematics_B.b_V[10] = 1.0;
        forward_kinematics_xzlarf(2, 1, 11, forward_kinematics_B.tau[1],
          forward_kinematics_B.b_V, 15, forward_kinematics_B.work);
        forward_kinematics_B.b_j = 11;
        while (forward_kinematics_B.b_j + 1 <= 12) {
          forward_kinematics_B.b_V[forward_kinematics_B.b_j] *=
            -forward_kinematics_B.tau[1];
          forward_kinematics_B.b_j++;
        }

        forward_kinematics_B.b_V[10] = 1.0 - forward_kinematics_B.tau[1];
        forward_kinematics_B.b_V[9] = 0.0;
        forward_kinematics_B.b_V[5] = 1.0;
        forward_kinematics_xzlarf(3, 2, 6, forward_kinematics_B.tau[0],
          forward_kinematics_B.b_V, 10, forward_kinematics_B.work);
        forward_kinematics_B.b_j = 6;
        while (forward_kinematics_B.b_j + 1 <= 8) {
          forward_kinematics_B.b_V[forward_kinematics_B.b_j] *=
            -forward_kinematics_B.tau[0];
          forward_kinematics_B.b_j++;
        }

        forward_kinematics_B.b_V[5] = 1.0 - forward_kinematics_B.tau[0];
        forward_kinematics_eml_dlahqr(forward_kinematics_B.b_A,
          forward_kinematics_B.b_V);
      }

      for (forward_kinematics_B.b_j = 0; forward_kinematics_B.b_j < 16;
           forward_kinematics_B.b_j++) {
        V[forward_kinematics_B.b_j].re =
          forward_kinematics_B.b_V[forward_kinematics_B.b_j];
        V[forward_kinematics_B.b_j].im = 0.0;
      }

      D[0].re = forward_kinematics_B.b_A[0];
      D[0].im = 0.0;
      D[1].re = forward_kinematics_B.b_A[5];
      D[1].im = 0.0;
      D[2].re = forward_kinematics_B.b_A[10];
      D[2].im = 0.0;
      D[3].re = forward_kinematics_B.b_A[15];
      D[3].im = 0.0;
    } else {
      for (forward_kinematics_B.b_j = 0; forward_kinematics_B.b_j < 16;
           forward_kinematics_B.b_j++) {
        forward_kinematics_B.At[forward_kinematics_B.b_j].re =
          A[forward_kinematics_B.b_j];
        forward_kinematics_B.At[forward_kinematics_B.b_j].im = 0.0;
      }

      forward_kinematics_xzggev(forward_kinematics_B.At,
        &forward_kinematics_B.b_j, D, forward_kinematics_B.beta1, V);
      forward_kinematics_B.colnorm = 0.0;
      forward_kinematics_B.scale = 3.3121686421112381E-170;
      forward_kinematics_B.b_j = 0;
      while (forward_kinematics_B.b_j + 1 <= 4) {
        forward_kinematics_B.absxk = fabs(V[forward_kinematics_B.b_j].re);
        if (forward_kinematics_B.absxk > forward_kinematics_B.scale) {
          forward_kinematics_B.t = forward_kinematics_B.scale /
            forward_kinematics_B.absxk;
          forward_kinematics_B.colnorm = forward_kinematics_B.colnorm *
            forward_kinematics_B.t * forward_kinematics_B.t + 1.0;
          forward_kinematics_B.scale = forward_kinematics_B.absxk;
        } else {
          forward_kinematics_B.t = forward_kinematics_B.absxk /
            forward_kinematics_B.scale;
          forward_kinematics_B.colnorm += forward_kinematics_B.t *
            forward_kinematics_B.t;
        }

        forward_kinematics_B.absxk = fabs(V[forward_kinematics_B.b_j].im);
        if (forward_kinematics_B.absxk > forward_kinematics_B.scale) {
          forward_kinematics_B.t = forward_kinematics_B.scale /
            forward_kinematics_B.absxk;
          forward_kinematics_B.colnorm = forward_kinematics_B.colnorm *
            forward_kinematics_B.t * forward_kinematics_B.t + 1.0;
          forward_kinematics_B.scale = forward_kinematics_B.absxk;
        } else {
          forward_kinematics_B.t = forward_kinematics_B.absxk /
            forward_kinematics_B.scale;
          forward_kinematics_B.colnorm += forward_kinematics_B.t *
            forward_kinematics_B.t;
        }

        forward_kinematics_B.b_j++;
      }

      forward_kinematics_B.colnorm = forward_kinematics_B.scale * sqrt
        (forward_kinematics_B.colnorm);
      forward_kinematics_B.b_j = 0;
      while (forward_kinematics_B.b_j + 1 <= 4) {
        if (V[forward_kinematics_B.b_j].im == 0.0) {
          forward_kinematics_B.scale = V[forward_kinematics_B.b_j].re /
            forward_kinematics_B.colnorm;
          forward_kinematics_B.absxk = 0.0;
        } else if (V[forward_kinematics_B.b_j].re == 0.0) {
          forward_kinematics_B.scale = 0.0;
          forward_kinematics_B.absxk = V[forward_kinematics_B.b_j].im /
            forward_kinematics_B.colnorm;
        } else {
          forward_kinematics_B.scale = V[forward_kinematics_B.b_j].re /
            forward_kinematics_B.colnorm;
          forward_kinematics_B.absxk = V[forward_kinematics_B.b_j].im /
            forward_kinematics_B.colnorm;
        }

        V[forward_kinematics_B.b_j].re = forward_kinematics_B.scale;
        V[forward_kinematics_B.b_j].im = forward_kinematics_B.absxk;
        forward_kinematics_B.b_j++;
      }

      forward_kinematics_B.colnorm = 0.0;
      forward_kinematics_B.scale = 3.3121686421112381E-170;
      forward_kinematics_B.b_j = 4;
      while (forward_kinematics_B.b_j + 1 <= 8) {
        forward_kinematics_B.absxk = fabs(V[forward_kinematics_B.b_j].re);
        if (forward_kinematics_B.absxk > forward_kinematics_B.scale) {
          forward_kinematics_B.t = forward_kinematics_B.scale /
            forward_kinematics_B.absxk;
          forward_kinematics_B.colnorm = forward_kinematics_B.colnorm *
            forward_kinematics_B.t * forward_kinematics_B.t + 1.0;
          forward_kinematics_B.scale = forward_kinematics_B.absxk;
        } else {
          forward_kinematics_B.t = forward_kinematics_B.absxk /
            forward_kinematics_B.scale;
          forward_kinematics_B.colnorm += forward_kinematics_B.t *
            forward_kinematics_B.t;
        }

        forward_kinematics_B.absxk = fabs(V[forward_kinematics_B.b_j].im);
        if (forward_kinematics_B.absxk > forward_kinematics_B.scale) {
          forward_kinematics_B.t = forward_kinematics_B.scale /
            forward_kinematics_B.absxk;
          forward_kinematics_B.colnorm = forward_kinematics_B.colnorm *
            forward_kinematics_B.t * forward_kinematics_B.t + 1.0;
          forward_kinematics_B.scale = forward_kinematics_B.absxk;
        } else {
          forward_kinematics_B.t = forward_kinematics_B.absxk /
            forward_kinematics_B.scale;
          forward_kinematics_B.colnorm += forward_kinematics_B.t *
            forward_kinematics_B.t;
        }

        forward_kinematics_B.b_j++;
      }

      forward_kinematics_B.colnorm = forward_kinematics_B.scale * sqrt
        (forward_kinematics_B.colnorm);
      forward_kinematics_B.b_j = 4;
      while (forward_kinematics_B.b_j + 1 <= 8) {
        if (V[forward_kinematics_B.b_j].im == 0.0) {
          forward_kinematics_B.scale = V[forward_kinematics_B.b_j].re /
            forward_kinematics_B.colnorm;
          forward_kinematics_B.absxk = 0.0;
        } else if (V[forward_kinematics_B.b_j].re == 0.0) {
          forward_kinematics_B.scale = 0.0;
          forward_kinematics_B.absxk = V[forward_kinematics_B.b_j].im /
            forward_kinematics_B.colnorm;
        } else {
          forward_kinematics_B.scale = V[forward_kinematics_B.b_j].re /
            forward_kinematics_B.colnorm;
          forward_kinematics_B.absxk = V[forward_kinematics_B.b_j].im /
            forward_kinematics_B.colnorm;
        }

        V[forward_kinematics_B.b_j].re = forward_kinematics_B.scale;
        V[forward_kinematics_B.b_j].im = forward_kinematics_B.absxk;
        forward_kinematics_B.b_j++;
      }

      forward_kinematics_B.colnorm = 0.0;
      forward_kinematics_B.scale = 3.3121686421112381E-170;
      forward_kinematics_B.b_j = 8;
      while (forward_kinematics_B.b_j + 1 <= 12) {
        forward_kinematics_B.absxk = fabs(V[forward_kinematics_B.b_j].re);
        if (forward_kinematics_B.absxk > forward_kinematics_B.scale) {
          forward_kinematics_B.t = forward_kinematics_B.scale /
            forward_kinematics_B.absxk;
          forward_kinematics_B.colnorm = forward_kinematics_B.colnorm *
            forward_kinematics_B.t * forward_kinematics_B.t + 1.0;
          forward_kinematics_B.scale = forward_kinematics_B.absxk;
        } else {
          forward_kinematics_B.t = forward_kinematics_B.absxk /
            forward_kinematics_B.scale;
          forward_kinematics_B.colnorm += forward_kinematics_B.t *
            forward_kinematics_B.t;
        }

        forward_kinematics_B.absxk = fabs(V[forward_kinematics_B.b_j].im);
        if (forward_kinematics_B.absxk > forward_kinematics_B.scale) {
          forward_kinematics_B.t = forward_kinematics_B.scale /
            forward_kinematics_B.absxk;
          forward_kinematics_B.colnorm = forward_kinematics_B.colnorm *
            forward_kinematics_B.t * forward_kinematics_B.t + 1.0;
          forward_kinematics_B.scale = forward_kinematics_B.absxk;
        } else {
          forward_kinematics_B.t = forward_kinematics_B.absxk /
            forward_kinematics_B.scale;
          forward_kinematics_B.colnorm += forward_kinematics_B.t *
            forward_kinematics_B.t;
        }

        forward_kinematics_B.b_j++;
      }

      forward_kinematics_B.colnorm = forward_kinematics_B.scale * sqrt
        (forward_kinematics_B.colnorm);
      forward_kinematics_B.b_j = 8;
      while (forward_kinematics_B.b_j + 1 <= 12) {
        if (V[forward_kinematics_B.b_j].im == 0.0) {
          forward_kinematics_B.scale = V[forward_kinematics_B.b_j].re /
            forward_kinematics_B.colnorm;
          forward_kinematics_B.absxk = 0.0;
        } else if (V[forward_kinematics_B.b_j].re == 0.0) {
          forward_kinematics_B.scale = 0.0;
          forward_kinematics_B.absxk = V[forward_kinematics_B.b_j].im /
            forward_kinematics_B.colnorm;
        } else {
          forward_kinematics_B.scale = V[forward_kinematics_B.b_j].re /
            forward_kinematics_B.colnorm;
          forward_kinematics_B.absxk = V[forward_kinematics_B.b_j].im /
            forward_kinematics_B.colnorm;
        }

        V[forward_kinematics_B.b_j].re = forward_kinematics_B.scale;
        V[forward_kinematics_B.b_j].im = forward_kinematics_B.absxk;
        forward_kinematics_B.b_j++;
      }

      forward_kinematics_B.colnorm = 0.0;
      forward_kinematics_B.scale = 3.3121686421112381E-170;
      forward_kinematics_B.b_j = 12;
      while (forward_kinematics_B.b_j + 1 <= 16) {
        forward_kinematics_B.absxk = fabs(V[forward_kinematics_B.b_j].re);
        if (forward_kinematics_B.absxk > forward_kinematics_B.scale) {
          forward_kinematics_B.t = forward_kinematics_B.scale /
            forward_kinematics_B.absxk;
          forward_kinematics_B.colnorm = forward_kinematics_B.colnorm *
            forward_kinematics_B.t * forward_kinematics_B.t + 1.0;
          forward_kinematics_B.scale = forward_kinematics_B.absxk;
        } else {
          forward_kinematics_B.t = forward_kinematics_B.absxk /
            forward_kinematics_B.scale;
          forward_kinematics_B.colnorm += forward_kinematics_B.t *
            forward_kinematics_B.t;
        }

        forward_kinematics_B.absxk = fabs(V[forward_kinematics_B.b_j].im);
        if (forward_kinematics_B.absxk > forward_kinematics_B.scale) {
          forward_kinematics_B.t = forward_kinematics_B.scale /
            forward_kinematics_B.absxk;
          forward_kinematics_B.colnorm = forward_kinematics_B.colnorm *
            forward_kinematics_B.t * forward_kinematics_B.t + 1.0;
          forward_kinematics_B.scale = forward_kinematics_B.absxk;
        } else {
          forward_kinematics_B.t = forward_kinematics_B.absxk /
            forward_kinematics_B.scale;
          forward_kinematics_B.colnorm += forward_kinematics_B.t *
            forward_kinematics_B.t;
        }

        forward_kinematics_B.b_j++;
      }

      forward_kinematics_B.colnorm = forward_kinematics_B.scale * sqrt
        (forward_kinematics_B.colnorm);
      forward_kinematics_B.b_j = 12;
      while (forward_kinematics_B.b_j + 1 <= 16) {
        if (V[forward_kinematics_B.b_j].im == 0.0) {
          forward_kinematics_B.scale = V[forward_kinematics_B.b_j].re /
            forward_kinematics_B.colnorm;
          forward_kinematics_B.absxk = 0.0;
        } else if (V[forward_kinematics_B.b_j].re == 0.0) {
          forward_kinematics_B.scale = 0.0;
          forward_kinematics_B.absxk = V[forward_kinematics_B.b_j].im /
            forward_kinematics_B.colnorm;
        } else {
          forward_kinematics_B.scale = V[forward_kinematics_B.b_j].re /
            forward_kinematics_B.colnorm;
          forward_kinematics_B.absxk = V[forward_kinematics_B.b_j].im /
            forward_kinematics_B.colnorm;
        }

        V[forward_kinematics_B.b_j].re = forward_kinematics_B.scale;
        V[forward_kinematics_B.b_j].im = forward_kinematics_B.absxk;
        forward_kinematics_B.b_j++;
      }

      if (forward_kinematics_B.beta1[0].im == 0.0) {
        if (D[0].im == 0.0) {
          forward_kinematics_B.scale = D[0].re / forward_kinematics_B.beta1[0].
            re;
          forward_kinematics_B.absxk = 0.0;
        } else if (D[0].re == 0.0) {
          forward_kinematics_B.scale = 0.0;
          forward_kinematics_B.absxk = D[0].im / forward_kinematics_B.beta1[0].
            re;
        } else {
          forward_kinematics_B.scale = D[0].re / forward_kinematics_B.beta1[0].
            re;
          forward_kinematics_B.absxk = D[0].im / forward_kinematics_B.beta1[0].
            re;
        }
      } else if (forward_kinematics_B.beta1[0].re == 0.0) {
        if (D[0].re == 0.0) {
          forward_kinematics_B.scale = D[0].im / forward_kinematics_B.beta1[0].
            im;
          forward_kinematics_B.absxk = 0.0;
        } else if (D[0].im == 0.0) {
          forward_kinematics_B.scale = 0.0;
          forward_kinematics_B.absxk = -(D[0].re / forward_kinematics_B.beta1[0]
            .im);
        } else {
          forward_kinematics_B.scale = D[0].im / forward_kinematics_B.beta1[0].
            im;
          forward_kinematics_B.absxk = -(D[0].re / forward_kinematics_B.beta1[0]
            .im);
        }
      } else {
        forward_kinematics_B.colnorm = fabs(forward_kinematics_B.beta1[0].re);
        forward_kinematics_B.scale = fabs(forward_kinematics_B.beta1[0].im);
        if (forward_kinematics_B.colnorm > forward_kinematics_B.scale) {
          forward_kinematics_B.colnorm = forward_kinematics_B.beta1[0].im /
            forward_kinematics_B.beta1[0].re;
          forward_kinematics_B.absxk = forward_kinematics_B.colnorm *
            forward_kinematics_B.beta1[0].im + forward_kinematics_B.beta1[0].re;
          forward_kinematics_B.scale = (forward_kinematics_B.colnorm * D[0].im +
            D[0].re) / forward_kinematics_B.absxk;
          forward_kinematics_B.absxk = (D[0].im - forward_kinematics_B.colnorm *
            D[0].re) / forward_kinematics_B.absxk;
        } else if (forward_kinematics_B.scale == forward_kinematics_B.colnorm) {
          forward_kinematics_B.absxk = forward_kinematics_B.beta1[0].re > 0.0 ?
            0.5 : -0.5;
          forward_kinematics_B.t = forward_kinematics_B.beta1[0].im > 0.0 ? 0.5 :
            -0.5;
          forward_kinematics_B.scale = (D[0].re * forward_kinematics_B.absxk +
            D[0].im * forward_kinematics_B.t) / forward_kinematics_B.colnorm;
          forward_kinematics_B.absxk = (D[0].im * forward_kinematics_B.absxk -
            D[0].re * forward_kinematics_B.t) / forward_kinematics_B.colnorm;
        } else {
          forward_kinematics_B.colnorm = forward_kinematics_B.beta1[0].re /
            forward_kinematics_B.beta1[0].im;
          forward_kinematics_B.absxk = forward_kinematics_B.colnorm *
            forward_kinematics_B.beta1[0].re + forward_kinematics_B.beta1[0].im;
          forward_kinematics_B.scale = (forward_kinematics_B.colnorm * D[0].re +
            D[0].im) / forward_kinematics_B.absxk;
          forward_kinematics_B.absxk = (forward_kinematics_B.colnorm * D[0].im -
            D[0].re) / forward_kinematics_B.absxk;
        }
      }

      D[0].re = forward_kinematics_B.scale;
      D[0].im = forward_kinematics_B.absxk;
      if (forward_kinematics_B.beta1[1].im == 0.0) {
        if (D[1].im == 0.0) {
          forward_kinematics_B.scale = D[1].re / forward_kinematics_B.beta1[1].
            re;
          forward_kinematics_B.absxk = 0.0;
        } else if (D[1].re == 0.0) {
          forward_kinematics_B.scale = 0.0;
          forward_kinematics_B.absxk = D[1].im / forward_kinematics_B.beta1[1].
            re;
        } else {
          forward_kinematics_B.scale = D[1].re / forward_kinematics_B.beta1[1].
            re;
          forward_kinematics_B.absxk = D[1].im / forward_kinematics_B.beta1[1].
            re;
        }
      } else if (forward_kinematics_B.beta1[1].re == 0.0) {
        if (D[1].re == 0.0) {
          forward_kinematics_B.scale = D[1].im / forward_kinematics_B.beta1[1].
            im;
          forward_kinematics_B.absxk = 0.0;
        } else if (D[1].im == 0.0) {
          forward_kinematics_B.scale = 0.0;
          forward_kinematics_B.absxk = -(D[1].re / forward_kinematics_B.beta1[1]
            .im);
        } else {
          forward_kinematics_B.scale = D[1].im / forward_kinematics_B.beta1[1].
            im;
          forward_kinematics_B.absxk = -(D[1].re / forward_kinematics_B.beta1[1]
            .im);
        }
      } else {
        forward_kinematics_B.colnorm = fabs(forward_kinematics_B.beta1[1].re);
        forward_kinematics_B.scale = fabs(forward_kinematics_B.beta1[1].im);
        if (forward_kinematics_B.colnorm > forward_kinematics_B.scale) {
          forward_kinematics_B.colnorm = forward_kinematics_B.beta1[1].im /
            forward_kinematics_B.beta1[1].re;
          forward_kinematics_B.absxk = forward_kinematics_B.colnorm *
            forward_kinematics_B.beta1[1].im + forward_kinematics_B.beta1[1].re;
          forward_kinematics_B.scale = (forward_kinematics_B.colnorm * D[1].im +
            D[1].re) / forward_kinematics_B.absxk;
          forward_kinematics_B.absxk = (D[1].im - forward_kinematics_B.colnorm *
            D[1].re) / forward_kinematics_B.absxk;
        } else if (forward_kinematics_B.scale == forward_kinematics_B.colnorm) {
          forward_kinematics_B.absxk = forward_kinematics_B.beta1[1].re > 0.0 ?
            0.5 : -0.5;
          forward_kinematics_B.t = forward_kinematics_B.beta1[1].im > 0.0 ? 0.5 :
            -0.5;
          forward_kinematics_B.scale = (D[1].re * forward_kinematics_B.absxk +
            D[1].im * forward_kinematics_B.t) / forward_kinematics_B.colnorm;
          forward_kinematics_B.absxk = (D[1].im * forward_kinematics_B.absxk -
            D[1].re * forward_kinematics_B.t) / forward_kinematics_B.colnorm;
        } else {
          forward_kinematics_B.colnorm = forward_kinematics_B.beta1[1].re /
            forward_kinematics_B.beta1[1].im;
          forward_kinematics_B.absxk = forward_kinematics_B.colnorm *
            forward_kinematics_B.beta1[1].re + forward_kinematics_B.beta1[1].im;
          forward_kinematics_B.scale = (forward_kinematics_B.colnorm * D[1].re +
            D[1].im) / forward_kinematics_B.absxk;
          forward_kinematics_B.absxk = (forward_kinematics_B.colnorm * D[1].im -
            D[1].re) / forward_kinematics_B.absxk;
        }
      }

      D[1].re = forward_kinematics_B.scale;
      D[1].im = forward_kinematics_B.absxk;
      if (forward_kinematics_B.beta1[2].im == 0.0) {
        if (D[2].im == 0.0) {
          forward_kinematics_B.scale = D[2].re / forward_kinematics_B.beta1[2].
            re;
          forward_kinematics_B.absxk = 0.0;
        } else if (D[2].re == 0.0) {
          forward_kinematics_B.scale = 0.0;
          forward_kinematics_B.absxk = D[2].im / forward_kinematics_B.beta1[2].
            re;
        } else {
          forward_kinematics_B.scale = D[2].re / forward_kinematics_B.beta1[2].
            re;
          forward_kinematics_B.absxk = D[2].im / forward_kinematics_B.beta1[2].
            re;
        }
      } else if (forward_kinematics_B.beta1[2].re == 0.0) {
        if (D[2].re == 0.0) {
          forward_kinematics_B.scale = D[2].im / forward_kinematics_B.beta1[2].
            im;
          forward_kinematics_B.absxk = 0.0;
        } else if (D[2].im == 0.0) {
          forward_kinematics_B.scale = 0.0;
          forward_kinematics_B.absxk = -(D[2].re / forward_kinematics_B.beta1[2]
            .im);
        } else {
          forward_kinematics_B.scale = D[2].im / forward_kinematics_B.beta1[2].
            im;
          forward_kinematics_B.absxk = -(D[2].re / forward_kinematics_B.beta1[2]
            .im);
        }
      } else {
        forward_kinematics_B.colnorm = fabs(forward_kinematics_B.beta1[2].re);
        forward_kinematics_B.scale = fabs(forward_kinematics_B.beta1[2].im);
        if (forward_kinematics_B.colnorm > forward_kinematics_B.scale) {
          forward_kinematics_B.colnorm = forward_kinematics_B.beta1[2].im /
            forward_kinematics_B.beta1[2].re;
          forward_kinematics_B.absxk = forward_kinematics_B.colnorm *
            forward_kinematics_B.beta1[2].im + forward_kinematics_B.beta1[2].re;
          forward_kinematics_B.scale = (forward_kinematics_B.colnorm * D[2].im +
            D[2].re) / forward_kinematics_B.absxk;
          forward_kinematics_B.absxk = (D[2].im - forward_kinematics_B.colnorm *
            D[2].re) / forward_kinematics_B.absxk;
        } else if (forward_kinematics_B.scale == forward_kinematics_B.colnorm) {
          forward_kinematics_B.absxk = forward_kinematics_B.beta1[2].re > 0.0 ?
            0.5 : -0.5;
          forward_kinematics_B.t = forward_kinematics_B.beta1[2].im > 0.0 ? 0.5 :
            -0.5;
          forward_kinematics_B.scale = (D[2].re * forward_kinematics_B.absxk +
            D[2].im * forward_kinematics_B.t) / forward_kinematics_B.colnorm;
          forward_kinematics_B.absxk = (D[2].im * forward_kinematics_B.absxk -
            D[2].re * forward_kinematics_B.t) / forward_kinematics_B.colnorm;
        } else {
          forward_kinematics_B.colnorm = forward_kinematics_B.beta1[2].re /
            forward_kinematics_B.beta1[2].im;
          forward_kinematics_B.absxk = forward_kinematics_B.colnorm *
            forward_kinematics_B.beta1[2].re + forward_kinematics_B.beta1[2].im;
          forward_kinematics_B.scale = (forward_kinematics_B.colnorm * D[2].re +
            D[2].im) / forward_kinematics_B.absxk;
          forward_kinematics_B.absxk = (forward_kinematics_B.colnorm * D[2].im -
            D[2].re) / forward_kinematics_B.absxk;
        }
      }

      D[2].re = forward_kinematics_B.scale;
      D[2].im = forward_kinematics_B.absxk;
      if (forward_kinematics_B.beta1[3].im == 0.0) {
        if (D[3].im == 0.0) {
          forward_kinematics_B.scale = D[3].re / forward_kinematics_B.beta1[3].
            re;
          forward_kinematics_B.absxk = 0.0;
        } else if (D[3].re == 0.0) {
          forward_kinematics_B.scale = 0.0;
          forward_kinematics_B.absxk = D[3].im / forward_kinematics_B.beta1[3].
            re;
        } else {
          forward_kinematics_B.scale = D[3].re / forward_kinematics_B.beta1[3].
            re;
          forward_kinematics_B.absxk = D[3].im / forward_kinematics_B.beta1[3].
            re;
        }
      } else if (forward_kinematics_B.beta1[3].re == 0.0) {
        if (D[3].re == 0.0) {
          forward_kinematics_B.scale = D[3].im / forward_kinematics_B.beta1[3].
            im;
          forward_kinematics_B.absxk = 0.0;
        } else if (D[3].im == 0.0) {
          forward_kinematics_B.scale = 0.0;
          forward_kinematics_B.absxk = -(D[3].re / forward_kinematics_B.beta1[3]
            .im);
        } else {
          forward_kinematics_B.scale = D[3].im / forward_kinematics_B.beta1[3].
            im;
          forward_kinematics_B.absxk = -(D[3].re / forward_kinematics_B.beta1[3]
            .im);
        }
      } else {
        forward_kinematics_B.colnorm = fabs(forward_kinematics_B.beta1[3].re);
        forward_kinematics_B.scale = fabs(forward_kinematics_B.beta1[3].im);
        if (forward_kinematics_B.colnorm > forward_kinematics_B.scale) {
          forward_kinematics_B.colnorm = forward_kinematics_B.beta1[3].im /
            forward_kinematics_B.beta1[3].re;
          forward_kinematics_B.absxk = forward_kinematics_B.colnorm *
            forward_kinematics_B.beta1[3].im + forward_kinematics_B.beta1[3].re;
          forward_kinematics_B.scale = (forward_kinematics_B.colnorm * D[3].im +
            D[3].re) / forward_kinematics_B.absxk;
          forward_kinematics_B.absxk = (D[3].im - forward_kinematics_B.colnorm *
            D[3].re) / forward_kinematics_B.absxk;
        } else if (forward_kinematics_B.scale == forward_kinematics_B.colnorm) {
          forward_kinematics_B.absxk = forward_kinematics_B.beta1[3].re > 0.0 ?
            0.5 : -0.5;
          forward_kinematics_B.t = forward_kinematics_B.beta1[3].im > 0.0 ? 0.5 :
            -0.5;
          forward_kinematics_B.scale = (D[3].re * forward_kinematics_B.absxk +
            D[3].im * forward_kinematics_B.t) / forward_kinematics_B.colnorm;
          forward_kinematics_B.absxk = (D[3].im * forward_kinematics_B.absxk -
            D[3].re * forward_kinematics_B.t) / forward_kinematics_B.colnorm;
        } else {
          forward_kinematics_B.colnorm = forward_kinematics_B.beta1[3].re /
            forward_kinematics_B.beta1[3].im;
          forward_kinematics_B.absxk = forward_kinematics_B.colnorm *
            forward_kinematics_B.beta1[3].re + forward_kinematics_B.beta1[3].im;
          forward_kinematics_B.scale = (forward_kinematics_B.colnorm * D[3].re +
            D[3].im) / forward_kinematics_B.absxk;
          forward_kinematics_B.absxk = (forward_kinematics_B.colnorm * D[3].im -
            D[3].re) / forward_kinematics_B.absxk;
        }
      }

      D[3].re = forward_kinematics_B.scale;
      D[3].im = forward_kinematics_B.absxk;
    }
  }
}

static void matlabCodegenHandle_matlabCo_ef(ros_slros_internal_block_Subs_T *obj)
{
  if (!obj->matlabCodegenIsDeleted) {
    obj->matlabCodegenIsDeleted = true;
  }
}

static void emxFreeStruct_c_rigidBodyJoint(c_rigidBodyJoint_forward_kine_T
  *pStruct)
{
  forward_kinemati_emxFree_char_T(&pStruct->Type);
  forward_kinemati_emxFree_real_T(&pStruct->MotionSubspace);
}

static void emxFreeStruct_o_robotics_manip_(o_robotics_manip_internal_Rig_T
  *pStruct)
{
  forward_kinemati_emxFree_char_T(&pStruct->NameInternal);
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
  forward_kinemati_emxFree_char_T(&pStruct->NameInternal);
  emxFreeStruct_c_rigidBodyJoint(&pStruct->JointInternal);
}

static void emxFreeStruct_c_rigidBodyJoint1(c_rigidBodyJoint_forward_ki_e_T
  *pStruct)
{
  forward_kinemati_emxFree_char_T(&pStruct->Type);
}

static void emxFreeStruct_o_robotics_mani_e(o_robotics_manip_internal_R_e_T
  *pStruct)
{
  forward_kinemati_emxFree_char_T(&pStruct->NameInternal);
  emxFreeStruct_c_rigidBodyJoint1(&pStruct->JointInternal);
}

static void emxFreeStruct_p_robotics_mani_e(p_robotics_manip_internal_R_e_T
  *pStruct)
{
  emxFreeStruct_o_robotics_mani_e(&pStruct->Base);
}

static void emxFreeStruct_robotics_slmani_e(robotics_slmanip_internal_b_e_T
  *pStruct)
{
  emxFreeStruct_p_robotics_mani_e(&pStruct->TreeInternal);
}

static void emxFreeStruct_n_robotics_mani_e(n_robotics_manip_internal_R_e_T
  *pStruct)
{
  forward_kinemati_emxFree_char_T(&pStruct->NameInternal);
  emxFreeStruct_c_rigidBodyJoint1(&pStruct->JointInternal);
}

static void matlabCodegenHandle_matlabCodeg(ros_slros_internal_block_Publ_T *obj)
{
  if (!obj->matlabCodegenIsDeleted) {
    obj->matlabCodegenIsDeleted = true;
  }
}

static void emxInitStruct_c_rigidBodyJoint(c_rigidBodyJoint_forward_kine_T
  *pStruct)
{
  forward_kinemati_emxInit_char_T(&pStruct->Type, 2);
  forward_kinemati_emxInit_real_T(&pStruct->MotionSubspace, 2);
}

static void emxInitStruct_o_robotics_manip_(o_robotics_manip_internal_Rig_T
  *pStruct)
{
  forward_kinemati_emxInit_char_T(&pStruct->NameInternal, 2);
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
  forward_kinemati_emxInit_char_T(&pStruct->NameInternal, 2);
  emxInitStruct_c_rigidBodyJoint(&pStruct->JointInternal);
}

static n_robotics_manip_internal_Rig_T *forward_kin_RigidBody_RigidBody
  (n_robotics_manip_internal_Rig_T *obj)
{
  n_robotics_manip_internal_Rig_T *b_obj;
  int8_T msubspace_data[36];
  emxArray_char_T_forward_kinem_T *switch_expression;
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
  forwar_emxEnsureCapacity_char_T(obj->NameInternal, b_kstr);
  for (b_kstr = 0; b_kstr < 13; b_kstr++) {
    obj->NameInternal->data[b_kstr] = tmp_0[b_kstr];
  }

  obj->ParentIndex = 0.0;
  b_kstr = obj->JointInternal.Type->size[0] * obj->JointInternal.Type->size[1];
  obj->JointInternal.Type->size[0] = 1;
  obj->JointInternal.Type->size[1] = 5;
  forwar_emxEnsureCapacity_char_T(obj->JointInternal.Type, b_kstr);
  for (b_kstr = 0; b_kstr < 5; b_kstr++) {
    obj->JointInternal.Type->data[b_kstr] = tmp_1[b_kstr];
  }

  forward_kinemati_emxInit_char_T(&switch_expression, 2);
  b_kstr = switch_expression->size[0] * switch_expression->size[1];
  switch_expression->size[0] = 1;
  switch_expression->size[1] = obj->JointInternal.Type->size[1];
  forwar_emxEnsureCapacity_char_T(switch_expression, b_kstr);
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

  forward_kinemati_emxFree_char_T(&switch_expression);
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
  forwar_emxEnsureCapacity_real_T(obj->JointInternal.MotionSubspace, b_kstr);
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
  forwar_emxEnsureCapacity_real_T(obj->JointInternal.MotionSubspace, b_kstr);
  for (b_kstr = 0; b_kstr < 6; b_kstr++) {
    obj->JointInternal.MotionSubspace->data[b_kstr] = 0.0;
  }

  obj->JointInternal.JointAxisInternal[0] = 0.0;
  obj->JointInternal.JointAxisInternal[1] = 0.0;
  obj->JointInternal.JointAxisInternal[2] = 0.0;
  return b_obj;
}

static n_robotics_manip_internal_Rig_T *forward_k_RigidBody_RigidBody_e
  (n_robotics_manip_internal_Rig_T *obj)
{
  n_robotics_manip_internal_Rig_T *b_obj;
  int8_T msubspace_data[36];
  emxArray_char_T_forward_kinem_T *switch_expression;
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
  forwar_emxEnsureCapacity_char_T(obj->NameInternal, b_kstr);
  for (b_kstr = 0; b_kstr < 10; b_kstr++) {
    obj->NameInternal->data[b_kstr] = tmp_0[b_kstr];
  }

  obj->ParentIndex = 1.0;
  b_kstr = obj->JointInternal.Type->size[0] * obj->JointInternal.Type->size[1];
  obj->JointInternal.Type->size[0] = 1;
  obj->JointInternal.Type->size[1] = 8;
  forwar_emxEnsureCapacity_char_T(obj->JointInternal.Type, b_kstr);
  for (b_kstr = 0; b_kstr < 8; b_kstr++) {
    obj->JointInternal.Type->data[b_kstr] = tmp_1[b_kstr];
  }

  forward_kinemati_emxInit_char_T(&switch_expression, 2);
  b_kstr = switch_expression->size[0] * switch_expression->size[1];
  switch_expression->size[0] = 1;
  switch_expression->size[1] = obj->JointInternal.Type->size[1];
  forwar_emxEnsureCapacity_char_T(switch_expression, b_kstr);
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

  forward_kinemati_emxFree_char_T(&switch_expression);
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
  forwar_emxEnsureCapacity_real_T(obj->JointInternal.MotionSubspace, b_kstr);
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
  forwar_emxEnsureCapacity_real_T(obj->JointInternal.MotionSubspace, b_kstr);
  for (b_kstr = 0; b_kstr < 6; b_kstr++) {
    obj->JointInternal.MotionSubspace->data[b_kstr] = tmp_5[b_kstr];
  }

  obj->JointInternal.JointAxisInternal[0] = 0.0;
  obj->JointInternal.JointAxisInternal[1] = 0.0;
  obj->JointInternal.JointAxisInternal[2] = 1.0;
  return b_obj;
}

static n_robotics_manip_internal_Rig_T *forward__RigidBody_RigidBody_ef
  (n_robotics_manip_internal_Rig_T *obj)
{
  n_robotics_manip_internal_Rig_T *b_obj;
  int8_T msubspace_data[36];
  emxArray_char_T_forward_kinem_T *switch_expression;
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
  forwar_emxEnsureCapacity_char_T(obj->NameInternal, b_kstr);
  for (b_kstr = 0; b_kstr < 10; b_kstr++) {
    obj->NameInternal->data[b_kstr] = tmp_0[b_kstr];
  }

  obj->ParentIndex = 2.0;
  b_kstr = obj->JointInternal.Type->size[0] * obj->JointInternal.Type->size[1];
  obj->JointInternal.Type->size[0] = 1;
  obj->JointInternal.Type->size[1] = 8;
  forwar_emxEnsureCapacity_char_T(obj->JointInternal.Type, b_kstr);
  for (b_kstr = 0; b_kstr < 8; b_kstr++) {
    obj->JointInternal.Type->data[b_kstr] = tmp_1[b_kstr];
  }

  forward_kinemati_emxInit_char_T(&switch_expression, 2);
  b_kstr = switch_expression->size[0] * switch_expression->size[1];
  switch_expression->size[0] = 1;
  switch_expression->size[1] = obj->JointInternal.Type->size[1];
  forwar_emxEnsureCapacity_char_T(switch_expression, b_kstr);
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

  forward_kinemati_emxFree_char_T(&switch_expression);
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
  forwar_emxEnsureCapacity_real_T(obj->JointInternal.MotionSubspace, b_kstr);
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
  forwar_emxEnsureCapacity_real_T(obj->JointInternal.MotionSubspace, b_kstr);
  for (b_kstr = 0; b_kstr < 6; b_kstr++) {
    obj->JointInternal.MotionSubspace->data[b_kstr] = tmp_5[b_kstr];
  }

  obj->JointInternal.JointAxisInternal[0] = 0.0;
  obj->JointInternal.JointAxisInternal[1] = 0.0;
  obj->JointInternal.JointAxisInternal[2] = -1.0;
  return b_obj;
}

static n_robotics_manip_internal_Rig_T *forward_RigidBody_RigidBody_efg
  (n_robotics_manip_internal_Rig_T *obj)
{
  n_robotics_manip_internal_Rig_T *b_obj;
  int8_T msubspace_data[36];
  emxArray_char_T_forward_kinem_T *switch_expression;
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
  forwar_emxEnsureCapacity_char_T(obj->NameInternal, b_kstr);
  for (b_kstr = 0; b_kstr < 10; b_kstr++) {
    obj->NameInternal->data[b_kstr] = tmp_0[b_kstr];
  }

  obj->ParentIndex = 3.0;
  b_kstr = obj->JointInternal.Type->size[0] * obj->JointInternal.Type->size[1];
  obj->JointInternal.Type->size[0] = 1;
  obj->JointInternal.Type->size[1] = 8;
  forwar_emxEnsureCapacity_char_T(obj->JointInternal.Type, b_kstr);
  for (b_kstr = 0; b_kstr < 8; b_kstr++) {
    obj->JointInternal.Type->data[b_kstr] = tmp_1[b_kstr];
  }

  forward_kinemati_emxInit_char_T(&switch_expression, 2);
  b_kstr = switch_expression->size[0] * switch_expression->size[1];
  switch_expression->size[0] = 1;
  switch_expression->size[1] = obj->JointInternal.Type->size[1];
  forwar_emxEnsureCapacity_char_T(switch_expression, b_kstr);
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

  forward_kinemati_emxFree_char_T(&switch_expression);
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
  forwar_emxEnsureCapacity_real_T(obj->JointInternal.MotionSubspace, b_kstr);
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
  forwar_emxEnsureCapacity_real_T(obj->JointInternal.MotionSubspace, b_kstr);
  for (b_kstr = 0; b_kstr < 6; b_kstr++) {
    obj->JointInternal.MotionSubspace->data[b_kstr] = tmp_5[b_kstr];
  }

  obj->JointInternal.JointAxisInternal[0] = 0.0;
  obj->JointInternal.JointAxisInternal[1] = 0.0;
  obj->JointInternal.JointAxisInternal[2] = 1.0;
  return b_obj;
}

static n_robotics_manip_internal_Rig_T *forwar_RigidBody_RigidBody_efgd
  (n_robotics_manip_internal_Rig_T *obj)
{
  n_robotics_manip_internal_Rig_T *b_obj;
  int8_T msubspace_data[36];
  emxArray_char_T_forward_kinem_T *switch_expression;
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
  forwar_emxEnsureCapacity_char_T(obj->NameInternal, b_kstr);
  for (b_kstr = 0; b_kstr < 10; b_kstr++) {
    obj->NameInternal->data[b_kstr] = tmp_0[b_kstr];
  }

  obj->ParentIndex = 4.0;
  b_kstr = obj->JointInternal.Type->size[0] * obj->JointInternal.Type->size[1];
  obj->JointInternal.Type->size[0] = 1;
  obj->JointInternal.Type->size[1] = 8;
  forwar_emxEnsureCapacity_char_T(obj->JointInternal.Type, b_kstr);
  for (b_kstr = 0; b_kstr < 8; b_kstr++) {
    obj->JointInternal.Type->data[b_kstr] = tmp_1[b_kstr];
  }

  forward_kinemati_emxInit_char_T(&switch_expression, 2);
  b_kstr = switch_expression->size[0] * switch_expression->size[1];
  switch_expression->size[0] = 1;
  switch_expression->size[1] = obj->JointInternal.Type->size[1];
  forwar_emxEnsureCapacity_char_T(switch_expression, b_kstr);
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

  forward_kinemati_emxFree_char_T(&switch_expression);
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
  forwar_emxEnsureCapacity_real_T(obj->JointInternal.MotionSubspace, b_kstr);
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
  forwar_emxEnsureCapacity_real_T(obj->JointInternal.MotionSubspace, b_kstr);
  for (b_kstr = 0; b_kstr < 6; b_kstr++) {
    obj->JointInternal.MotionSubspace->data[b_kstr] = tmp_5[b_kstr];
  }

  obj->JointInternal.JointAxisInternal[0] = 0.0;
  obj->JointInternal.JointAxisInternal[1] = 0.0;
  obj->JointInternal.JointAxisInternal[2] = 1.0;
  return b_obj;
}

static n_robotics_manip_internal_Rig_T *forwa_RigidBody_RigidBody_efgdb
  (n_robotics_manip_internal_Rig_T *obj)
{
  n_robotics_manip_internal_Rig_T *b_obj;
  int8_T msubspace_data[36];
  emxArray_char_T_forward_kinem_T *switch_expression;
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
  forwar_emxEnsureCapacity_char_T(obj->NameInternal, b_kstr);
  for (b_kstr = 0; b_kstr < 10; b_kstr++) {
    obj->NameInternal->data[b_kstr] = tmp_0[b_kstr];
  }

  obj->ParentIndex = 5.0;
  b_kstr = obj->JointInternal.Type->size[0] * obj->JointInternal.Type->size[1];
  obj->JointInternal.Type->size[0] = 1;
  obj->JointInternal.Type->size[1] = 8;
  forwar_emxEnsureCapacity_char_T(obj->JointInternal.Type, b_kstr);
  for (b_kstr = 0; b_kstr < 8; b_kstr++) {
    obj->JointInternal.Type->data[b_kstr] = tmp_1[b_kstr];
  }

  forward_kinemati_emxInit_char_T(&switch_expression, 2);
  b_kstr = switch_expression->size[0] * switch_expression->size[1];
  switch_expression->size[0] = 1;
  switch_expression->size[1] = obj->JointInternal.Type->size[1];
  forwar_emxEnsureCapacity_char_T(switch_expression, b_kstr);
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

  forward_kinemati_emxFree_char_T(&switch_expression);
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
  forwar_emxEnsureCapacity_real_T(obj->JointInternal.MotionSubspace, b_kstr);
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
  forwar_emxEnsureCapacity_real_T(obj->JointInternal.MotionSubspace, b_kstr);
  for (b_kstr = 0; b_kstr < 6; b_kstr++) {
    obj->JointInternal.MotionSubspace->data[b_kstr] = tmp_5[b_kstr];
  }

  obj->JointInternal.JointAxisInternal[0] = 0.0;
  obj->JointInternal.JointAxisInternal[1] = 1.0;
  obj->JointInternal.JointAxisInternal[2] = 0.0;
  return b_obj;
}

static n_robotics_manip_internal_Rig_T *forw_RigidBody_RigidBody_efgdbo
  (n_robotics_manip_internal_Rig_T *obj)
{
  n_robotics_manip_internal_Rig_T *b_obj;
  int8_T msubspace_data[36];
  emxArray_char_T_forward_kinem_T *switch_expression;
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
  forwar_emxEnsureCapacity_char_T(obj->NameInternal, b_kstr);
  for (b_kstr = 0; b_kstr < 10; b_kstr++) {
    obj->NameInternal->data[b_kstr] = tmp_0[b_kstr];
  }

  obj->ParentIndex = 6.0;
  b_kstr = obj->JointInternal.Type->size[0] * obj->JointInternal.Type->size[1];
  obj->JointInternal.Type->size[0] = 1;
  obj->JointInternal.Type->size[1] = 8;
  forwar_emxEnsureCapacity_char_T(obj->JointInternal.Type, b_kstr);
  for (b_kstr = 0; b_kstr < 8; b_kstr++) {
    obj->JointInternal.Type->data[b_kstr] = tmp_1[b_kstr];
  }

  forward_kinemati_emxInit_char_T(&switch_expression, 2);
  b_kstr = switch_expression->size[0] * switch_expression->size[1];
  switch_expression->size[0] = 1;
  switch_expression->size[1] = obj->JointInternal.Type->size[1];
  forwar_emxEnsureCapacity_char_T(switch_expression, b_kstr);
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

  forward_kinemati_emxFree_char_T(&switch_expression);
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
  forwar_emxEnsureCapacity_real_T(obj->JointInternal.MotionSubspace, b_kstr);
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
  forwar_emxEnsureCapacity_real_T(obj->JointInternal.MotionSubspace, b_kstr);
  for (b_kstr = 0; b_kstr < 6; b_kstr++) {
    obj->JointInternal.MotionSubspace->data[b_kstr] = tmp_5[b_kstr];
  }

  obj->JointInternal.JointAxisInternal[0] = 0.0;
  obj->JointInternal.JointAxisInternal[1] = 0.0;
  obj->JointInternal.JointAxisInternal[2] = 1.0;
  return b_obj;
}

static n_robotics_manip_internal_Rig_T *for_RigidBody_RigidBody_efgdboc
  (n_robotics_manip_internal_Rig_T *obj)
{
  n_robotics_manip_internal_Rig_T *b_obj;
  int8_T msubspace_data[36];
  emxArray_char_T_forward_kinem_T *switch_expression;
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
  forwar_emxEnsureCapacity_char_T(obj->NameInternal, b_kstr);
  for (b_kstr = 0; b_kstr < 11; b_kstr++) {
    obj->NameInternal->data[b_kstr] = tmp_0[b_kstr];
  }

  obj->ParentIndex = 7.0;
  b_kstr = obj->JointInternal.Type->size[0] * obj->JointInternal.Type->size[1];
  obj->JointInternal.Type->size[0] = 1;
  obj->JointInternal.Type->size[1] = 5;
  forwar_emxEnsureCapacity_char_T(obj->JointInternal.Type, b_kstr);
  for (b_kstr = 0; b_kstr < 5; b_kstr++) {
    obj->JointInternal.Type->data[b_kstr] = tmp_1[b_kstr];
  }

  forward_kinemati_emxInit_char_T(&switch_expression, 2);
  b_kstr = switch_expression->size[0] * switch_expression->size[1];
  switch_expression->size[0] = 1;
  switch_expression->size[1] = obj->JointInternal.Type->size[1];
  forwar_emxEnsureCapacity_char_T(switch_expression, b_kstr);
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

  forward_kinemati_emxFree_char_T(&switch_expression);
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
  forwar_emxEnsureCapacity_real_T(obj->JointInternal.MotionSubspace, b_kstr);
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
  forwar_emxEnsureCapacity_real_T(obj->JointInternal.MotionSubspace, b_kstr);
  for (b_kstr = 0; b_kstr < 6; b_kstr++) {
    obj->JointInternal.MotionSubspace->data[b_kstr] = 0.0;
  }

  obj->JointInternal.JointAxisInternal[0] = 0.0;
  obj->JointInternal.JointAxisInternal[1] = 0.0;
  obj->JointInternal.JointAxisInternal[2] = 0.0;
  return b_obj;
}

static o_robotics_manip_internal_Rig_T *fo_RigidBody_RigidBody_efgdboc5
  (o_robotics_manip_internal_Rig_T *obj)
{
  o_robotics_manip_internal_Rig_T *b_obj;
  emxArray_char_T_forward_kinem_T *switch_expression;
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
  forwar_emxEnsureCapacity_char_T(obj->NameInternal, b_kstr);
  for (b_kstr = 0; b_kstr < 5; b_kstr++) {
    obj->NameInternal->data[b_kstr] = tmp_0[b_kstr];
  }

  b_kstr = obj->JointInternal.Type->size[0] * obj->JointInternal.Type->size[1];
  obj->JointInternal.Type->size[0] = 1;
  obj->JointInternal.Type->size[1] = 5;
  forwar_emxEnsureCapacity_char_T(obj->JointInternal.Type, b_kstr);
  for (b_kstr = 0; b_kstr < 5; b_kstr++) {
    obj->JointInternal.Type->data[b_kstr] = tmp_1[b_kstr];
  }

  forward_kinemati_emxInit_char_T(&switch_expression, 2);
  b_kstr = switch_expression->size[0] * switch_expression->size[1];
  switch_expression->size[0] = 1;
  switch_expression->size[1] = obj->JointInternal.Type->size[1];
  forwar_emxEnsureCapacity_char_T(switch_expression, b_kstr);
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
      forward_kinematics_B.b_h[b_kstr] = tmp_3[b_kstr];
    }

    b_bool = false;
    if (switch_expression->size[1] == 9) {
      b_kstr = 1;
      do {
        exitg1 = 0;
        if (b_kstr - 1 < 9) {
          loop_ub = b_kstr - 1;
          if (switch_expression->data[loop_ub] !=
              forward_kinematics_B.b_h[loop_ub]) {
            exitg1 = 1;
          } else {
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

  forward_kinemati_emxFree_char_T(&switch_expression);
  switch (b_kstr) {
   case 0:
    tmp[0] = 0;
    tmp[1] = 0;
    tmp[2] = 1;
    tmp[3] = 0;
    tmp[4] = 0;
    tmp[5] = 0;
    for (b_kstr = 0; b_kstr < 6; b_kstr++) {
      forward_kinematics_B.msubspace_data[b_kstr] = tmp[b_kstr];
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
      forward_kinematics_B.msubspace_data[b_kstr] = tmp[b_kstr];
    }

    obj->JointInternal.PositionNumber = 1.0;
    obj->JointInternal.JointAxisInternal[0] = 0.0;
    obj->JointInternal.JointAxisInternal[1] = 0.0;
    obj->JointInternal.JointAxisInternal[2] = 1.0;
    break;

   default:
    for (b_kstr = 0; b_kstr < 6; b_kstr++) {
      forward_kinematics_B.msubspace_data[b_kstr] = 0;
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
  forwar_emxEnsureCapacity_real_T(obj->JointInternal.MotionSubspace, b_kstr);
  for (b_kstr = 0; b_kstr < 6; b_kstr++) {
    obj->JointInternal.MotionSubspace->data[b_kstr] =
      forward_kinematics_B.msubspace_data[b_kstr];
  }

  return b_obj;
}

static p_robotics_manip_internal_Rig_T *for_RigidBodyTree_RigidBodyTree
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
  obj->Bodies[0] = forward_kin_RigidBody_RigidBody(iobj_0);
  obj->Bodies[0]->Index = 1.0;
  obj->Bodies[1] = forward_k_RigidBody_RigidBody_e(iobj_7);
  obj->Bodies[1]->Index = 2.0;
  obj->Bodies[2] = forward__RigidBody_RigidBody_ef(iobj_1);
  obj->Bodies[2]->Index = 3.0;
  obj->Bodies[3] = forward_RigidBody_RigidBody_efg(iobj_2);
  obj->Bodies[3]->Index = 4.0;
  obj->Bodies[4] = forwar_RigidBody_RigidBody_efgd(iobj_3);
  obj->Bodies[4]->Index = 5.0;
  obj->Bodies[5] = forwa_RigidBody_RigidBody_efgdb(iobj_4);
  obj->Bodies[5]->Index = 6.0;
  obj->Bodies[6] = forw_RigidBody_RigidBody_efgdbo(iobj_5);
  obj->Bodies[6]->Index = 7.0;
  obj->Bodies[7] = for_RigidBody_RigidBody_efgdboc(iobj_6);
  obj->Bodies[7]->Index = 8.0;
  obj->NumBodies = 8.0;
  obj->PositionNumber = 6.0;
  obj->VelocityNumber = 6.0;
  for (i = 0; i < 16; i++) {
    obj->PositionDoFMap[i] = tmp[i];
  }

  fo_RigidBody_RigidBody_efgdboc5(&obj->Base);
  return b_obj;
}

static void emxInitStruct_c_rigidBodyJoint1(c_rigidBodyJoint_forward_ki_e_T
  *pStruct)
{
  forward_kinemati_emxInit_char_T(&pStruct->Type, 2);
}

static void emxInitStruct_o_robotics_mani_e(o_robotics_manip_internal_R_e_T
  *pStruct)
{
  forward_kinemati_emxInit_char_T(&pStruct->NameInternal, 2);
  emxInitStruct_c_rigidBodyJoint1(&pStruct->JointInternal);
}

static void emxInitStruct_p_robotics_mani_e(p_robotics_manip_internal_R_e_T
  *pStruct)
{
  emxInitStruct_o_robotics_mani_e(&pStruct->Base);
}

static void emxInitStruct_robotics_slmani_e(robotics_slmanip_internal_b_e_T
  *pStruct)
{
  emxInitStruct_p_robotics_mani_e(&pStruct->TreeInternal);
}

static void emxInitStruct_n_robotics_mani_e(n_robotics_manip_internal_R_e_T
  *pStruct)
{
  forward_kinemati_emxInit_char_T(&pStruct->NameInternal, 2);
  emxInitStruct_c_rigidBodyJoint1(&pStruct->JointInternal);
}

static n_robotics_manip_internal_R_e_T *f_RigidBody_RigidBody_efgdboc5e
  (n_robotics_manip_internal_R_e_T *obj)
{
  n_robotics_manip_internal_R_e_T *b_obj;
  emxArray_char_T_forward_kinem_T *switch_expression;
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
  forwar_emxEnsureCapacity_char_T(obj->NameInternal, b_kstr);
  for (b_kstr = 0; b_kstr < 13; b_kstr++) {
    obj->NameInternal->data[b_kstr] = tmp[b_kstr];
  }

  obj->ParentIndex = 0.0;
  b_kstr = obj->JointInternal.Type->size[0] * obj->JointInternal.Type->size[1];
  obj->JointInternal.Type->size[0] = 1;
  obj->JointInternal.Type->size[1] = 5;
  forwar_emxEnsureCapacity_char_T(obj->JointInternal.Type, b_kstr);
  for (b_kstr = 0; b_kstr < 5; b_kstr++) {
    obj->JointInternal.Type->data[b_kstr] = tmp_0[b_kstr];
  }

  forward_kinemati_emxInit_char_T(&switch_expression, 2);
  b_kstr = switch_expression->size[0] * switch_expression->size[1];
  switch_expression->size[0] = 1;
  switch_expression->size[1] = obj->JointInternal.Type->size[1];
  forwar_emxEnsureCapacity_char_T(switch_expression, b_kstr);
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

  forward_kinemati_emxFree_char_T(&switch_expression);
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

static n_robotics_manip_internal_R_e_T *RigidBody_RigidBody_efgdboc5ed
  (n_robotics_manip_internal_R_e_T *obj)
{
  n_robotics_manip_internal_R_e_T *b_obj;
  emxArray_char_T_forward_kinem_T *switch_expression;
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
  forwar_emxEnsureCapacity_char_T(obj->NameInternal, b_kstr);
  for (b_kstr = 0; b_kstr < 10; b_kstr++) {
    obj->NameInternal->data[b_kstr] = tmp[b_kstr];
  }

  obj->ParentIndex = 1.0;
  b_kstr = obj->JointInternal.Type->size[0] * obj->JointInternal.Type->size[1];
  obj->JointInternal.Type->size[0] = 1;
  obj->JointInternal.Type->size[1] = 8;
  forwar_emxEnsureCapacity_char_T(obj->JointInternal.Type, b_kstr);
  for (b_kstr = 0; b_kstr < 8; b_kstr++) {
    obj->JointInternal.Type->data[b_kstr] = tmp_0[b_kstr];
  }

  forward_kinemati_emxInit_char_T(&switch_expression, 2);
  b_kstr = switch_expression->size[0] * switch_expression->size[1];
  switch_expression->size[0] = 1;
  switch_expression->size[1] = obj->JointInternal.Type->size[1];
  forwar_emxEnsureCapacity_char_T(switch_expression, b_kstr);
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

  forward_kinemati_emxFree_char_T(&switch_expression);
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

static n_robotics_manip_internal_R_e_T *RigidBody_RigidBody_efgdboc5edi
  (n_robotics_manip_internal_R_e_T *obj)
{
  n_robotics_manip_internal_R_e_T *b_obj;
  emxArray_char_T_forward_kinem_T *switch_expression;
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
  forwar_emxEnsureCapacity_char_T(obj->NameInternal, b_kstr);
  for (b_kstr = 0; b_kstr < 10; b_kstr++) {
    obj->NameInternal->data[b_kstr] = tmp[b_kstr];
  }

  obj->ParentIndex = 2.0;
  b_kstr = obj->JointInternal.Type->size[0] * obj->JointInternal.Type->size[1];
  obj->JointInternal.Type->size[0] = 1;
  obj->JointInternal.Type->size[1] = 8;
  forwar_emxEnsureCapacity_char_T(obj->JointInternal.Type, b_kstr);
  for (b_kstr = 0; b_kstr < 8; b_kstr++) {
    obj->JointInternal.Type->data[b_kstr] = tmp_0[b_kstr];
  }

  forward_kinemati_emxInit_char_T(&switch_expression, 2);
  b_kstr = switch_expression->size[0] * switch_expression->size[1];
  switch_expression->size[0] = 1;
  switch_expression->size[1] = obj->JointInternal.Type->size[1];
  forwar_emxEnsureCapacity_char_T(switch_expression, b_kstr);
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

  forward_kinemati_emxFree_char_T(&switch_expression);
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

static n_robotics_manip_internal_R_e_T *RigidBody_RigidBod_efgdboc5edi3
  (n_robotics_manip_internal_R_e_T *obj)
{
  n_robotics_manip_internal_R_e_T *b_obj;
  emxArray_char_T_forward_kinem_T *switch_expression;
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
  forwar_emxEnsureCapacity_char_T(obj->NameInternal, b_kstr);
  for (b_kstr = 0; b_kstr < 10; b_kstr++) {
    obj->NameInternal->data[b_kstr] = tmp[b_kstr];
  }

  obj->ParentIndex = 3.0;
  b_kstr = obj->JointInternal.Type->size[0] * obj->JointInternal.Type->size[1];
  obj->JointInternal.Type->size[0] = 1;
  obj->JointInternal.Type->size[1] = 8;
  forwar_emxEnsureCapacity_char_T(obj->JointInternal.Type, b_kstr);
  for (b_kstr = 0; b_kstr < 8; b_kstr++) {
    obj->JointInternal.Type->data[b_kstr] = tmp_0[b_kstr];
  }

  forward_kinemati_emxInit_char_T(&switch_expression, 2);
  b_kstr = switch_expression->size[0] * switch_expression->size[1];
  switch_expression->size[0] = 1;
  switch_expression->size[1] = obj->JointInternal.Type->size[1];
  forwar_emxEnsureCapacity_char_T(switch_expression, b_kstr);
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

  forward_kinemati_emxFree_char_T(&switch_expression);
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

static n_robotics_manip_internal_R_e_T *RigidBody_RigidBo_efgdboc5edi3c
  (n_robotics_manip_internal_R_e_T *obj)
{
  n_robotics_manip_internal_R_e_T *b_obj;
  emxArray_char_T_forward_kinem_T *switch_expression;
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
  forwar_emxEnsureCapacity_char_T(obj->NameInternal, b_kstr);
  for (b_kstr = 0; b_kstr < 10; b_kstr++) {
    obj->NameInternal->data[b_kstr] = tmp[b_kstr];
  }

  obj->ParentIndex = 4.0;
  b_kstr = obj->JointInternal.Type->size[0] * obj->JointInternal.Type->size[1];
  obj->JointInternal.Type->size[0] = 1;
  obj->JointInternal.Type->size[1] = 8;
  forwar_emxEnsureCapacity_char_T(obj->JointInternal.Type, b_kstr);
  for (b_kstr = 0; b_kstr < 8; b_kstr++) {
    obj->JointInternal.Type->data[b_kstr] = tmp_0[b_kstr];
  }

  forward_kinemati_emxInit_char_T(&switch_expression, 2);
  b_kstr = switch_expression->size[0] * switch_expression->size[1];
  switch_expression->size[0] = 1;
  switch_expression->size[1] = obj->JointInternal.Type->size[1];
  forwar_emxEnsureCapacity_char_T(switch_expression, b_kstr);
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

  forward_kinemati_emxFree_char_T(&switch_expression);
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

static p_robotics_manip_internal_R_e_T *f_RigidBodyTree_RigidBodyTree_e
  (p_robotics_manip_internal_R_e_T *obj, n_robotics_manip_internal_R_e_T *iobj_0,
   n_robotics_manip_internal_R_e_T *iobj_1, n_robotics_manip_internal_R_e_T
   *iobj_2, n_robotics_manip_internal_R_e_T *iobj_3,
   n_robotics_manip_internal_R_e_T *iobj_4, n_robotics_manip_internal_R_e_T
   *iobj_5, n_robotics_manip_internal_R_e_T *iobj_6,
   n_robotics_manip_internal_R_e_T *iobj_7)
{
  p_robotics_manip_internal_R_e_T *b_obj;
  o_robotics_manip_internal_R_e_T *obj_0;
  emxArray_char_T_forward_kinem_T *switch_expression;
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
  obj->Bodies[0] = f_RigidBody_RigidBody_efgdboc5e(iobj_0);
  obj->Bodies[1] = RigidBody_RigidBody_efgdboc5ed(iobj_7);
  obj->Bodies[2] = RigidBody_RigidBody_efgdboc5edi(iobj_1);
  obj->Bodies[3] = RigidBody_RigidBod_efgdboc5edi3(iobj_2);
  obj->Bodies[4] = RigidBody_RigidBo_efgdboc5edi3c(iobj_3);
  b_kstr = iobj_4->NameInternal->size[0] * iobj_4->NameInternal->size[1];
  iobj_4->NameInternal->size[0] = 1;
  iobj_4->NameInternal->size[1] = 10;
  forwar_emxEnsureCapacity_char_T(iobj_4->NameInternal, b_kstr);
  for (b_kstr = 0; b_kstr < 10; b_kstr++) {
    iobj_4->NameInternal->data[b_kstr] = tmp[b_kstr];
  }

  iobj_4->ParentIndex = 5.0;
  b_kstr = iobj_4->JointInternal.Type->size[0] * iobj_4->
    JointInternal.Type->size[1];
  iobj_4->JointInternal.Type->size[0] = 1;
  iobj_4->JointInternal.Type->size[1] = 8;
  forwar_emxEnsureCapacity_char_T(iobj_4->JointInternal.Type, b_kstr);
  for (b_kstr = 0; b_kstr < 8; b_kstr++) {
    iobj_4->JointInternal.Type->data[b_kstr] = tmp_0[b_kstr];
  }

  forward_kinemati_emxInit_char_T(&switch_expression, 2);
  b_kstr = switch_expression->size[0] * switch_expression->size[1];
  switch_expression->size[0] = 1;
  switch_expression->size[1] = iobj_4->JointInternal.Type->size[1];
  forwar_emxEnsureCapacity_char_T(switch_expression, b_kstr);
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
  forwar_emxEnsureCapacity_char_T(iobj_5->NameInternal, b_kstr);
  for (b_kstr = 0; b_kstr < 10; b_kstr++) {
    iobj_5->NameInternal->data[b_kstr] = tmp_4[b_kstr];
  }

  iobj_5->ParentIndex = 6.0;
  b_kstr = iobj_5->JointInternal.Type->size[0] * iobj_5->
    JointInternal.Type->size[1];
  iobj_5->JointInternal.Type->size[0] = 1;
  iobj_5->JointInternal.Type->size[1] = 8;
  forwar_emxEnsureCapacity_char_T(iobj_5->JointInternal.Type, b_kstr);
  for (b_kstr = 0; b_kstr < 8; b_kstr++) {
    iobj_5->JointInternal.Type->data[b_kstr] = tmp_0[b_kstr];
  }

  b_kstr = switch_expression->size[0] * switch_expression->size[1];
  switch_expression->size[0] = 1;
  switch_expression->size[1] = iobj_5->JointInternal.Type->size[1];
  forwar_emxEnsureCapacity_char_T(switch_expression, b_kstr);
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
  forwar_emxEnsureCapacity_char_T(iobj_6->NameInternal, b_kstr);
  for (b_kstr = 0; b_kstr < 11; b_kstr++) {
    iobj_6->NameInternal->data[b_kstr] = tmp_6[b_kstr];
  }

  iobj_6->ParentIndex = 7.0;
  b_kstr = iobj_6->JointInternal.Type->size[0] * iobj_6->
    JointInternal.Type->size[1];
  iobj_6->JointInternal.Type->size[0] = 1;
  iobj_6->JointInternal.Type->size[1] = 5;
  forwar_emxEnsureCapacity_char_T(iobj_6->JointInternal.Type, b_kstr);
  for (b_kstr = 0; b_kstr < 5; b_kstr++) {
    iobj_6->JointInternal.Type->data[b_kstr] = tmp_7[b_kstr];
  }

  b_kstr = switch_expression->size[0] * switch_expression->size[1];
  switch_expression->size[0] = 1;
  switch_expression->size[1] = iobj_6->JointInternal.Type->size[1];
  forwar_emxEnsureCapacity_char_T(switch_expression, b_kstr);
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
  forwar_emxEnsureCapacity_char_T(obj->Base.NameInternal, b_kstr);
  for (b_kstr = 0; b_kstr < 5; b_kstr++) {
    obj->Base.NameInternal->data[b_kstr] = tmp_8[b_kstr];
  }

  b_kstr = obj->Base.JointInternal.Type->size[0] * obj->
    Base.JointInternal.Type->size[1];
  obj->Base.JointInternal.Type->size[0] = 1;
  obj->Base.JointInternal.Type->size[1] = 5;
  forwar_emxEnsureCapacity_char_T(obj->Base.JointInternal.Type, b_kstr);
  for (b_kstr = 0; b_kstr < 5; b_kstr++) {
    obj_0->JointInternal.Type->data[b_kstr] = tmp_7[b_kstr];
  }

  b_kstr = switch_expression->size[0] * switch_expression->size[1];
  switch_expression->size[0] = 1;
  switch_expression->size[1] = obj->Base.JointInternal.Type->size[1];
  forwar_emxEnsureCapacity_char_T(switch_expression, b_kstr);
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

  forward_kinemati_emxFree_char_T(&switch_expression);
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
void forward_kinematics_step(void)
{
  emxArray_real_T_forward_kinem_T *b;
  robotics_slmanip_internal_b_e_T *obj;
  p_robotics_manip_internal_R_e_T *obj_0;
  emxArray_f_cell_wrap_forward__T *Ttree;
  emxArray_char_T_forward_kinem_T *bname;
  n_robotics_manip_internal_R_e_T *obj_1;
  static const char_T tmp[11] = { 'e', 'd', 'o', '_', 'l', 'i', 'n', 'k', '_',
    'e', 'e' };

  static const char_T tmp_0[5] = { 'w', 'o', 'r', 'l', 'd' };

  int32_T exitg1;
  boolean_T exitg2;

  // Outputs for Atomic SubSystem: '<Root>/Subscribe'
  // MATLABSystem: '<S5>/SourceBlock' incorporates:
  //   Inport: '<S12>/In1'

  forward_kinemat_SystemCore_step(&forward_kinematics_B.b_varargout_1,
    forward_kinematics_B.b_varargout_2_Name,
    &forward_kinematics_B.b_varargout_2_Name_SL_Info_Curr,
    &forward_kinematics_B.b_varargout_2_Name_SL_Info_Rece,
    forward_kinematics_B.b_varargout_2_Position,
    &forward_kinematics_B.b_varargout_2_Position_SL_Info_,
    &forward_kinematics_B.b_varargout_2_Position_SL_Inf_p,
    forward_kinematics_B.b_varargout_2_Velocity,
    &forward_kinematics_B.b_varargout_2_Velocity_SL_Info_,
    &forward_kinematics_B.b_varargout_2_Velocity_SL_Inf_o,
    forward_kinematics_B.b_varargout_2_Effort,
    &forward_kinematics_B.b_varargout_2_Effort_SL_Info_Cu,
    &forward_kinematics_B.b_varargout_2_Effort_SL_Info_Re,
    &forward_kinematics_B.b_varargout_2_Header);

  // Outputs for Enabled SubSystem: '<S5>/Enabled Subsystem' incorporates:
  //   EnablePort: '<S12>/Enable'

  if (forward_kinematics_B.b_varargout_1) {
    memcpy(&forward_kinematics_B.In1.Name[0],
           &forward_kinematics_B.b_varargout_2_Name[0], sizeof
           (SL_Bus_forward_kinematics_std_msgs_String) << 4U);
    forward_kinematics_B.In1.Name_SL_Info.CurrentLength =
      forward_kinematics_B.b_varargout_2_Name_SL_Info_Curr;
    forward_kinematics_B.In1.Name_SL_Info.ReceivedLength =
      forward_kinematics_B.b_varargout_2_Name_SL_Info_Rece;
    forward_kinematics_B.In1.Position_SL_Info.CurrentLength =
      forward_kinematics_B.b_varargout_2_Position_SL_Info_;
    forward_kinematics_B.In1.Position_SL_Info.ReceivedLength =
      forward_kinematics_B.b_varargout_2_Position_SL_Inf_p;
    forward_kinematics_B.In1.Velocity_SL_Info.CurrentLength =
      forward_kinematics_B.b_varargout_2_Velocity_SL_Info_;
    forward_kinematics_B.In1.Velocity_SL_Info.ReceivedLength =
      forward_kinematics_B.b_varargout_2_Velocity_SL_Inf_o;
    memcpy(&forward_kinematics_B.In1.Position[0],
           &forward_kinematics_B.b_varargout_2_Position[0], sizeof(real_T) << 7U);
    memcpy(&forward_kinematics_B.In1.Velocity[0],
           &forward_kinematics_B.b_varargout_2_Velocity[0], sizeof(real_T) << 7U);
    memcpy(&forward_kinematics_B.In1.Effort[0],
           &forward_kinematics_B.b_varargout_2_Effort[0], sizeof(real_T) << 7U);
    forward_kinematics_B.In1.Effort_SL_Info.CurrentLength =
      forward_kinematics_B.b_varargout_2_Effort_SL_Info_Cu;
    forward_kinematics_B.In1.Effort_SL_Info.ReceivedLength =
      forward_kinematics_B.b_varargout_2_Effort_SL_Info_Re;
    forward_kinematics_B.In1.Header = forward_kinematics_B.b_varargout_2_Header;
  }

  // End of MATLABSystem: '<S5>/SourceBlock'
  // End of Outputs for SubSystem: '<S5>/Enabled Subsystem'
  // End of Outputs for SubSystem: '<Root>/Subscribe'
  forward_kinemati_emxInit_real_T(&b, 2);
  forward_kin_emxInit_f_cell_wrap(&Ttree, 2);
  forward_kinemati_emxInit_char_T(&bname, 2);

  // MATLABSystem: '<S7>/MATLAB System'
  RigidBodyTree_geometricJacobian(&forward_kinematics_DW.obj.TreeInternal,
    &forward_kinematics_B.In1.Position[0], b);

  // MATLABSystem: '<S8>/MATLAB System'
  obj = &forward_kinematics_DW.obj_o;
  obj_0 = &forward_kinematics_DW.obj_o.TreeInternal;
  RigidBodyTree_forwardKinemati_e(&obj->TreeInternal,
    &forward_kinematics_B.In1.Position[0], Ttree);
  forward_kinematics_B.bid1 = -1.0;
  forward_kinematics_B.b_kstr = bname->size[0] * bname->size[1];
  bname->size[0] = 1;
  bname->size[1] = obj_0->Base.NameInternal->size[1];
  forwar_emxEnsureCapacity_char_T(bname, forward_kinematics_B.b_kstr);
  forward_kinematics_B.loop_ub = obj_0->Base.NameInternal->size[0] *
    obj_0->Base.NameInternal->size[1] - 1;
  for (forward_kinematics_B.b_kstr = 0; forward_kinematics_B.b_kstr <=
       forward_kinematics_B.loop_ub; forward_kinematics_B.b_kstr++) {
    bname->data[forward_kinematics_B.b_kstr] = obj_0->Base.NameInternal->
      data[forward_kinematics_B.b_kstr];
  }

  for (forward_kinematics_B.b_kstr = 0; forward_kinematics_B.b_kstr < 11;
       forward_kinematics_B.b_kstr++) {
    forward_kinematics_B.b_b[forward_kinematics_B.b_kstr] =
      tmp[forward_kinematics_B.b_kstr];
  }

  forward_kinematics_B.b_varargout_1 = false;
  if (bname->size[1] == 11) {
    forward_kinematics_B.b_kstr = 1;
    do {
      exitg1 = 0;
      if (forward_kinematics_B.b_kstr - 1 < 11) {
        forward_kinematics_B.loop_ub = forward_kinematics_B.b_kstr - 1;
        if (bname->data[forward_kinematics_B.loop_ub] !=
            forward_kinematics_B.b_b[forward_kinematics_B.loop_ub]) {
          exitg1 = 1;
        } else {
          forward_kinematics_B.b_kstr++;
        }
      } else {
        forward_kinematics_B.b_varargout_1 = true;
        exitg1 = 1;
      }
    } while (exitg1 == 0);
  }

  if (forward_kinematics_B.b_varargout_1) {
    forward_kinematics_B.bid1 = 0.0;
  } else {
    forward_kinematics_B.K12 = obj->TreeInternal.NumBodies;
    forward_kinematics_B.b_i_f = 0;
    exitg2 = false;
    while ((!exitg2) && (forward_kinematics_B.b_i_f <= static_cast<int32_T>
                         (forward_kinematics_B.K12) - 1)) {
      obj_1 = obj_0->Bodies[forward_kinematics_B.b_i_f];
      forward_kinematics_B.b_kstr = bname->size[0] * bname->size[1];
      bname->size[0] = 1;
      bname->size[1] = obj_1->NameInternal->size[1];
      forwar_emxEnsureCapacity_char_T(bname, forward_kinematics_B.b_kstr);
      forward_kinematics_B.loop_ub = obj_1->NameInternal->size[0] *
        obj_1->NameInternal->size[1] - 1;
      for (forward_kinematics_B.b_kstr = 0; forward_kinematics_B.b_kstr <=
           forward_kinematics_B.loop_ub; forward_kinematics_B.b_kstr++) {
        bname->data[forward_kinematics_B.b_kstr] = obj_1->NameInternal->
          data[forward_kinematics_B.b_kstr];
      }

      for (forward_kinematics_B.b_kstr = 0; forward_kinematics_B.b_kstr < 11;
           forward_kinematics_B.b_kstr++) {
        forward_kinematics_B.b_b[forward_kinematics_B.b_kstr] =
          tmp[forward_kinematics_B.b_kstr];
      }

      forward_kinematics_B.b_varargout_1 = false;
      if (bname->size[1] == 11) {
        forward_kinematics_B.b_kstr = 1;
        do {
          exitg1 = 0;
          if (forward_kinematics_B.b_kstr - 1 < 11) {
            forward_kinematics_B.loop_ub = forward_kinematics_B.b_kstr - 1;
            if (bname->data[forward_kinematics_B.loop_ub] !=
                forward_kinematics_B.b_b[forward_kinematics_B.loop_ub]) {
              exitg1 = 1;
            } else {
              forward_kinematics_B.b_kstr++;
            }
          } else {
            forward_kinematics_B.b_varargout_1 = true;
            exitg1 = 1;
          }
        } while (exitg1 == 0);
      }

      if (forward_kinematics_B.b_varargout_1) {
        forward_kinematics_B.bid1 = static_cast<real_T>
          (forward_kinematics_B.b_i_f) + 1.0;
        exitg2 = true;
      } else {
        forward_kinematics_B.b_i_f++;
      }
    }
  }

  if (forward_kinematics_B.bid1 == 0.0) {
    memset(&forward_kinematics_B.T1[0], 0, sizeof(real_T) << 4U);
    forward_kinematics_B.T1[0] = 1.0;
    forward_kinematics_B.T1[5] = 1.0;
    forward_kinematics_B.T1[10] = 1.0;
    forward_kinematics_B.T1[15] = 1.0;
  } else {
    for (forward_kinematics_B.b_kstr = 0; forward_kinematics_B.b_kstr < 16;
         forward_kinematics_B.b_kstr++) {
      forward_kinematics_B.T1[forward_kinematics_B.b_kstr] = Ttree->data[
        static_cast<int32_T>(forward_kinematics_B.bid1) - 1]
        .f1[forward_kinematics_B.b_kstr];
    }
  }

  forward_kinematics_B.bid1 = -1.0;
  forward_kinematics_B.b_kstr = bname->size[0] * bname->size[1];
  bname->size[0] = 1;
  bname->size[1] = obj_0->Base.NameInternal->size[1];
  forwar_emxEnsureCapacity_char_T(bname, forward_kinematics_B.b_kstr);
  forward_kinematics_B.loop_ub = obj_0->Base.NameInternal->size[0] *
    obj_0->Base.NameInternal->size[1] - 1;
  for (forward_kinematics_B.b_kstr = 0; forward_kinematics_B.b_kstr <=
       forward_kinematics_B.loop_ub; forward_kinematics_B.b_kstr++) {
    bname->data[forward_kinematics_B.b_kstr] = obj_0->Base.NameInternal->
      data[forward_kinematics_B.b_kstr];
  }

  for (forward_kinematics_B.b_kstr = 0; forward_kinematics_B.b_kstr < 5;
       forward_kinematics_B.b_kstr++) {
    forward_kinematics_B.b_o[forward_kinematics_B.b_kstr] =
      tmp_0[forward_kinematics_B.b_kstr];
  }

  forward_kinematics_B.b_varargout_1 = false;
  if (bname->size[1] == 5) {
    forward_kinematics_B.b_kstr = 1;
    do {
      exitg1 = 0;
      if (forward_kinematics_B.b_kstr - 1 < 5) {
        forward_kinematics_B.loop_ub = forward_kinematics_B.b_kstr - 1;
        if (bname->data[forward_kinematics_B.loop_ub] !=
            forward_kinematics_B.b_o[forward_kinematics_B.loop_ub]) {
          exitg1 = 1;
        } else {
          forward_kinematics_B.b_kstr++;
        }
      } else {
        forward_kinematics_B.b_varargout_1 = true;
        exitg1 = 1;
      }
    } while (exitg1 == 0);
  }

  if (forward_kinematics_B.b_varargout_1) {
    forward_kinematics_B.bid1 = 0.0;
  } else {
    forward_kinematics_B.K12 = obj->TreeInternal.NumBodies;
    forward_kinematics_B.b_i_f = 0;
    exitg2 = false;
    while ((!exitg2) && (forward_kinematics_B.b_i_f <= static_cast<int32_T>
                         (forward_kinematics_B.K12) - 1)) {
      obj_1 = obj_0->Bodies[forward_kinematics_B.b_i_f];
      forward_kinematics_B.b_kstr = bname->size[0] * bname->size[1];
      bname->size[0] = 1;
      bname->size[1] = obj_1->NameInternal->size[1];
      forwar_emxEnsureCapacity_char_T(bname, forward_kinematics_B.b_kstr);
      forward_kinematics_B.loop_ub = obj_1->NameInternal->size[0] *
        obj_1->NameInternal->size[1] - 1;
      for (forward_kinematics_B.b_kstr = 0; forward_kinematics_B.b_kstr <=
           forward_kinematics_B.loop_ub; forward_kinematics_B.b_kstr++) {
        bname->data[forward_kinematics_B.b_kstr] = obj_1->NameInternal->
          data[forward_kinematics_B.b_kstr];
      }

      for (forward_kinematics_B.b_kstr = 0; forward_kinematics_B.b_kstr < 5;
           forward_kinematics_B.b_kstr++) {
        forward_kinematics_B.b_o[forward_kinematics_B.b_kstr] =
          tmp_0[forward_kinematics_B.b_kstr];
      }

      forward_kinematics_B.b_varargout_1 = false;
      if (bname->size[1] == 5) {
        forward_kinematics_B.b_kstr = 1;
        do {
          exitg1 = 0;
          if (forward_kinematics_B.b_kstr - 1 < 5) {
            forward_kinematics_B.loop_ub = forward_kinematics_B.b_kstr - 1;
            if (bname->data[forward_kinematics_B.loop_ub] !=
                forward_kinematics_B.b_o[forward_kinematics_B.loop_ub]) {
              exitg1 = 1;
            } else {
              forward_kinematics_B.b_kstr++;
            }
          } else {
            forward_kinematics_B.b_varargout_1 = true;
            exitg1 = 1;
          }
        } while (exitg1 == 0);
      }

      if (forward_kinematics_B.b_varargout_1) {
        forward_kinematics_B.bid1 = static_cast<real_T>
          (forward_kinematics_B.b_i_f) + 1.0;
        exitg2 = true;
      } else {
        forward_kinematics_B.b_i_f++;
      }
    }
  }

  forward_kinemati_emxFree_char_T(&bname);

  // MATLABSystem: '<S8>/MATLAB System'
  if (forward_kinematics_B.bid1 == 0.0) {
    memset(&forward_kinematics_B.T2[0], 0, sizeof(real_T) << 4U);
    forward_kinematics_B.T2[0] = 1.0;
    forward_kinematics_B.T2[5] = 1.0;
    forward_kinematics_B.T2[10] = 1.0;
    forward_kinematics_B.T2[15] = 1.0;
  } else {
    for (forward_kinematics_B.b_kstr = 0; forward_kinematics_B.b_kstr < 16;
         forward_kinematics_B.b_kstr++) {
      forward_kinematics_B.T2[forward_kinematics_B.b_kstr] = Ttree->data[
        static_cast<int32_T>(forward_kinematics_B.bid1) - 1]
        .f1[forward_kinematics_B.b_kstr];
    }
  }

  forward_kin_emxFree_f_cell_wrap(&Ttree);

  // MATLABSystem: '<S8>/MATLAB System'
  for (forward_kinematics_B.b_kstr = 0; forward_kinematics_B.b_kstr < 3;
       forward_kinematics_B.b_kstr++) {
    forward_kinematics_B.R_g[3 * forward_kinematics_B.b_kstr] =
      forward_kinematics_B.T2[forward_kinematics_B.b_kstr];
    forward_kinematics_B.R_g[3 * forward_kinematics_B.b_kstr + 1] =
      forward_kinematics_B.T2[forward_kinematics_B.b_kstr + 4];
    forward_kinematics_B.R_g[3 * forward_kinematics_B.b_kstr + 2] =
      forward_kinematics_B.T2[forward_kinematics_B.b_kstr + 8];
  }

  for (forward_kinematics_B.b_kstr = 0; forward_kinematics_B.b_kstr < 9;
       forward_kinematics_B.b_kstr++) {
    forward_kinematics_B.R_g1[forward_kinematics_B.b_kstr] =
      -forward_kinematics_B.R_g[forward_kinematics_B.b_kstr];
  }

  for (forward_kinematics_B.b_kstr = 0; forward_kinematics_B.b_kstr < 3;
       forward_kinematics_B.b_kstr++) {
    forward_kinematics_B.b_i_f = forward_kinematics_B.b_kstr << 2;
    forward_kinematics_B.R[forward_kinematics_B.b_i_f] =
      forward_kinematics_B.R_g[3 * forward_kinematics_B.b_kstr];
    forward_kinematics_B.R[forward_kinematics_B.b_i_f + 1] =
      forward_kinematics_B.R_g[3 * forward_kinematics_B.b_kstr + 1];
    forward_kinematics_B.R[forward_kinematics_B.b_i_f + 2] =
      forward_kinematics_B.R_g[3 * forward_kinematics_B.b_kstr + 2];
    forward_kinematics_B.R[forward_kinematics_B.b_kstr + 12] =
      forward_kinematics_B.R_g1[forward_kinematics_B.b_kstr + 6] *
      forward_kinematics_B.T2[14] +
      (forward_kinematics_B.R_g1[forward_kinematics_B.b_kstr + 3] *
       forward_kinematics_B.T2[13] +
       forward_kinematics_B.R_g1[forward_kinematics_B.b_kstr] *
       forward_kinematics_B.T2[12]);
  }

  forward_kinematics_B.R[3] = 0.0;
  forward_kinematics_B.R[7] = 0.0;
  forward_kinematics_B.R[11] = 0.0;
  forward_kinematics_B.R[15] = 1.0;
  for (forward_kinematics_B.b_kstr = 0; forward_kinematics_B.b_kstr < 4;
       forward_kinematics_B.b_kstr++) {
    for (forward_kinematics_B.b_i_f = 0; forward_kinematics_B.b_i_f < 4;
         forward_kinematics_B.b_i_f++) {
      forward_kinematics_B.loop_ub = forward_kinematics_B.b_kstr << 2;
      forward_kinematics_B.rtb_MATLABSystem_tmp = forward_kinematics_B.b_i_f +
        forward_kinematics_B.loop_ub;
      forward_kinematics_B.T2[forward_kinematics_B.rtb_MATLABSystem_tmp] = 0.0;
      forward_kinematics_B.T2[forward_kinematics_B.rtb_MATLABSystem_tmp] +=
        forward_kinematics_B.T1[forward_kinematics_B.loop_ub] *
        forward_kinematics_B.R[forward_kinematics_B.b_i_f];
      forward_kinematics_B.T2[forward_kinematics_B.rtb_MATLABSystem_tmp] +=
        forward_kinematics_B.T1[forward_kinematics_B.loop_ub + 1] *
        forward_kinematics_B.R[forward_kinematics_B.b_i_f + 4];
      forward_kinematics_B.T2[forward_kinematics_B.rtb_MATLABSystem_tmp] +=
        forward_kinematics_B.T1[forward_kinematics_B.loop_ub + 2] *
        forward_kinematics_B.R[forward_kinematics_B.b_i_f + 8];
      forward_kinematics_B.T2[forward_kinematics_B.rtb_MATLABSystem_tmp] +=
        forward_kinematics_B.T1[forward_kinematics_B.loop_ub + 3] *
        forward_kinematics_B.R[forward_kinematics_B.b_i_f + 12];
    }
  }

  // Product: '<S1>/MatrixMultiply' incorporates:
  //   MATLABSystem: '<S7>/MATLAB System'

  for (forward_kinematics_B.b_kstr = 0; forward_kinematics_B.b_kstr < 6;
       forward_kinematics_B.b_kstr++) {
    forward_kinematics_B.MatrixMultiply[forward_kinematics_B.b_kstr] = 0.0;
    for (forward_kinematics_B.b_i_f = 0; forward_kinematics_B.b_i_f < 6;
         forward_kinematics_B.b_i_f++) {
      forward_kinematics_B.K12 = b->data[6 * forward_kinematics_B.b_i_f +
        forward_kinematics_B.b_kstr] *
        forward_kinematics_B.In1.Velocity[forward_kinematics_B.b_i_f] +
        forward_kinematics_B.MatrixMultiply[forward_kinematics_B.b_kstr];
      forward_kinematics_B.MatrixMultiply[forward_kinematics_B.b_kstr] =
        forward_kinematics_B.K12;
    }
  }

  // End of Product: '<S1>/MatrixMultiply'
  forward_kinemati_emxFree_real_T(&b);

  // MATLAB Function: '<S1>/MATLAB Function'
  forward_kinematics_B.K12 = forward_kinematics_B.T2[4] +
    forward_kinematics_B.T2[1];
  forward_kinematics_B.bid1 = forward_kinematics_B.T2[8] +
    forward_kinematics_B.T2[2];
  forward_kinematics_B.K14 = forward_kinematics_B.T2[6] -
    forward_kinematics_B.T2[9];
  forward_kinematics_B.K23 = forward_kinematics_B.T2[9] +
    forward_kinematics_B.T2[6];
  forward_kinematics_B.K24 = forward_kinematics_B.T2[8] -
    forward_kinematics_B.T2[2];
  forward_kinematics_B.K34 = forward_kinematics_B.T2[1] -
    forward_kinematics_B.T2[4];
  forward_kinematics_B.T1[0] = ((forward_kinematics_B.T2[0] -
    forward_kinematics_B.T2[5]) - forward_kinematics_B.T2[10]) / 3.0;
  forward_kinematics_B.T1[4] = forward_kinematics_B.K12 / 3.0;
  forward_kinematics_B.T1[8] = forward_kinematics_B.bid1 / 3.0;
  forward_kinematics_B.T1[12] = forward_kinematics_B.K14 / 3.0;
  forward_kinematics_B.T1[1] = forward_kinematics_B.K12 / 3.0;
  forward_kinematics_B.T1[5] = ((forward_kinematics_B.T2[5] -
    forward_kinematics_B.T2[0]) - forward_kinematics_B.T2[10]) / 3.0;
  forward_kinematics_B.T1[9] = forward_kinematics_B.K23 / 3.0;
  forward_kinematics_B.T1[13] = forward_kinematics_B.K24 / 3.0;
  forward_kinematics_B.T1[2] = forward_kinematics_B.bid1 / 3.0;
  forward_kinematics_B.T1[6] = forward_kinematics_B.K23 / 3.0;
  forward_kinematics_B.T1[10] = ((forward_kinematics_B.T2[10] -
    forward_kinematics_B.T2[0]) - forward_kinematics_B.T2[5]) / 3.0;
  forward_kinematics_B.T1[14] = forward_kinematics_B.K34 / 3.0;
  forward_kinematics_B.T1[3] = forward_kinematics_B.K14 / 3.0;
  forward_kinematics_B.T1[7] = forward_kinematics_B.K24 / 3.0;
  forward_kinematics_B.T1[11] = forward_kinematics_B.K34 / 3.0;
  forward_kinematics_B.T1[15] = ((forward_kinematics_B.T2[0] +
    forward_kinematics_B.T2[5]) + forward_kinematics_B.T2[10]) / 3.0;
  forward_kinematics_eig(forward_kinematics_B.T1, forward_kinematics_B.eigVec,
    forward_kinematics_B.eigVal);
  forward_kinematics_B.cartOrn[0] = forward_kinematics_B.eigVal[0].re;
  forward_kinematics_B.cartOrn[1] = forward_kinematics_B.eigVal[1].re;
  forward_kinematics_B.cartOrn[2] = forward_kinematics_B.eigVal[2].re;
  forward_kinematics_B.cartOrn[3] = forward_kinematics_B.eigVal[3].re;
  if (!rtIsNaN(forward_kinematics_B.eigVal[0].re)) {
    forward_kinematics_B.b_kstr = 1;
  } else {
    forward_kinematics_B.b_kstr = 0;
    forward_kinematics_B.b_i_f = 2;
    exitg2 = false;
    while ((!exitg2) && (forward_kinematics_B.b_i_f < 5)) {
      if (!rtIsNaN(forward_kinematics_B.cartOrn[forward_kinematics_B.b_i_f - 1]))
      {
        forward_kinematics_B.b_kstr = forward_kinematics_B.b_i_f;
        exitg2 = true;
      } else {
        forward_kinematics_B.b_i_f++;
      }
    }
  }

  if (forward_kinematics_B.b_kstr != 0) {
    forward_kinematics_B.K12 =
      forward_kinematics_B.cartOrn[forward_kinematics_B.b_kstr - 1];
    forward_kinematics_B.b_i_f = forward_kinematics_B.b_kstr - 1;
    while (forward_kinematics_B.b_kstr + 1 < 5) {
      if (forward_kinematics_B.K12 <
          forward_kinematics_B.cartOrn[forward_kinematics_B.b_kstr]) {
        forward_kinematics_B.K12 =
          forward_kinematics_B.cartOrn[forward_kinematics_B.b_kstr];
        forward_kinematics_B.b_i_f = forward_kinematics_B.b_kstr;
      }

      forward_kinematics_B.b_kstr++;
    }

    forward_kinematics_B.b_kstr = forward_kinematics_B.b_i_f;
  }

  forward_kinematics_B.b_kstr <<= 2;
  forward_kinematics_B.b_i_f = forward_kinematics_B.b_kstr + 3;
  forward_kinematics_B.cartOrn[0] =
    forward_kinematics_B.eigVec[forward_kinematics_B.b_i_f].re;
  forward_kinematics_B.cartOrn[1] =
    forward_kinematics_B.eigVec[forward_kinematics_B.b_kstr].re;
  forward_kinematics_B.loop_ub = forward_kinematics_B.b_kstr + 1;
  forward_kinematics_B.cartOrn[2] =
    forward_kinematics_B.eigVec[forward_kinematics_B.loop_ub].re;
  forward_kinematics_B.rtb_MATLABSystem_tmp = forward_kinematics_B.b_kstr + 2;
  forward_kinematics_B.cartOrn[3] =
    forward_kinematics_B.eigVec[forward_kinematics_B.rtb_MATLABSystem_tmp].re;
  if (forward_kinematics_B.eigVec[forward_kinematics_B.b_i_f].re < 0.0) {
    forward_kinematics_B.cartOrn[0] =
      -forward_kinematics_B.eigVec[forward_kinematics_B.b_i_f].re;
    forward_kinematics_B.cartOrn[1] =
      -forward_kinematics_B.eigVec[forward_kinematics_B.b_kstr].re;
    forward_kinematics_B.cartOrn[2] =
      -forward_kinematics_B.eigVec[forward_kinematics_B.loop_ub].re;
    forward_kinematics_B.cartOrn[3] =
      -forward_kinematics_B.eigVec[forward_kinematics_B.rtb_MATLABSystem_tmp].re;
  }

  // Outputs for Atomic SubSystem: '<Root>/Subscribe1'
  // MATLABSystem: '<S6>/SourceBlock' incorporates:
  //   Inport: '<S13>/In1'

  forward_kinematics_B.b_varargout_1 =
    Sub_forward_kinematics_287.getLatestMessage
    (&forward_kinematics_B.b_varargout_2_o);

  // Outputs for Enabled SubSystem: '<S6>/Enabled Subsystem' incorporates:
  //   EnablePort: '<S13>/Enable'

  if (forward_kinematics_B.b_varargout_1) {
    forward_kinematics_B.In1_b = forward_kinematics_B.b_varargout_2_o;
  }

  // End of MATLABSystem: '<S6>/SourceBlock'
  // End of Outputs for SubSystem: '<S6>/Enabled Subsystem'
  // End of Outputs for SubSystem: '<Root>/Subscribe1'

  // BusAssignment: '<S2>/Bus Assignment' incorporates:
  //   Constant: '<S10>/Constant'
  //   MATLAB Function: '<S1>/MATLAB Function'

  forward_kinematics_B.BusAssignment = forward_kinematics_P.Constant_Value_h;
  forward_kinematics_B.BusAssignment.Header.Stamp =
    forward_kinematics_B.In1_b.Clock_;
  forward_kinematics_B.BusAssignment.Pose.Position.X = forward_kinematics_B.T2
    [12];
  forward_kinematics_B.BusAssignment.Pose.Position.Y = forward_kinematics_B.T2
    [13];
  forward_kinematics_B.BusAssignment.Pose.Position.Z = forward_kinematics_B.T2
    [14];
  forward_kinematics_B.BusAssignment.Pose.Orientation.X =
    forward_kinematics_B.cartOrn[0];
  forward_kinematics_B.BusAssignment.Pose.Orientation.Y =
    forward_kinematics_B.cartOrn[1];
  forward_kinematics_B.BusAssignment.Pose.Orientation.Z =
    forward_kinematics_B.cartOrn[2];
  forward_kinematics_B.BusAssignment.Pose.Orientation.W =
    forward_kinematics_B.cartOrn[3];

  // Outputs for Atomic SubSystem: '<Root>/Publish'
  // MATLABSystem: '<S3>/SinkBlock'
  Pub_forward_kinematics_282.publish(&forward_kinematics_B.BusAssignment);

  // End of Outputs for SubSystem: '<Root>/Publish'

  // BusAssignment: '<S2>/Bus Assignment1' incorporates:
  //   Constant: '<S11>/Constant'
  //   MATLAB Function: '<S1>/MATLAB Function'

  forward_kinematics_B.BusAssignment1 = forward_kinematics_P.Constant_Value_hs;
  forward_kinematics_B.BusAssignment1.Header.Stamp =
    forward_kinematics_B.In1_b.Clock_;
  forward_kinematics_B.BusAssignment1.Twist.Linear.X =
    forward_kinematics_B.MatrixMultiply[3];
  forward_kinematics_B.BusAssignment1.Twist.Linear.Y =
    forward_kinematics_B.MatrixMultiply[4];
  forward_kinematics_B.BusAssignment1.Twist.Linear.Z =
    forward_kinematics_B.MatrixMultiply[5];
  forward_kinematics_B.BusAssignment1.Twist.Angular.X =
    forward_kinematics_B.MatrixMultiply[0];
  forward_kinematics_B.BusAssignment1.Twist.Angular.Y =
    forward_kinematics_B.MatrixMultiply[1];
  forward_kinematics_B.BusAssignment1.Twist.Angular.Z =
    forward_kinematics_B.MatrixMultiply[2];

  // Outputs for Atomic SubSystem: '<Root>/Publish1'
  // MATLABSystem: '<S4>/SinkBlock'
  Pub_forward_kinematics_283.publish(&forward_kinematics_B.BusAssignment1);

  // End of Outputs for SubSystem: '<Root>/Publish1'
}

// Model initialize function
void forward_kinematics_initialize(void)
{
  // Registration code

  // initialize non-finites
  rt_InitInfAndNaN(sizeof(real_T));

  {
    char_T tmp[7];
    int32_T i;
    static const char_T tmp_0[13] = { '/', 'j', 'o', 'i', 'n', 't', '_', 's',
      't', 'a', 't', 'e', 's' };

    static const char_T tmp_1[6] = { '/', 'c', 'l', 'o', 'c', 'k' };

    static const char_T tmp_2[14] = { '/', 'c', 'a', 'r', 't', 'e', 's', 'i',
      'a', 'n', '_', 'p', 'o', 's' };

    static const char_T tmp_3[14] = { '/', 'c', 'a', 'r', 't', 'e', 's', 'i',
      'a', 'n', '_', 'v', 'e', 'l' };

    // SystemInitialize for Atomic SubSystem: '<Root>/Subscribe'
    // SystemInitialize for Enabled SubSystem: '<S5>/Enabled Subsystem'
    // SystemInitialize for Outport: '<S12>/Out1'
    forward_kinematics_B.In1 = forward_kinematics_P.Out1_Y0;

    // End of SystemInitialize for SubSystem: '<S5>/Enabled Subsystem'

    // Start for MATLABSystem: '<S5>/SourceBlock'
    forward_kinematics_DW.obj_b.matlabCodegenIsDeleted = false;
    forward_kinematics_DW.obj_b.isInitialized = 1;
    for (i = 0; i < 13; i++) {
      forward_kinematics_B.cv1[i] = tmp_0[i];
    }

    forward_kinematics_B.cv1[13] = '\x00';
    Sub_forward_kinematics_286.createSubscriber(forward_kinematics_B.cv1, 1);
    forward_kinematics_DW.obj_b.isSetupComplete = true;

    // End of Start for MATLABSystem: '<S5>/SourceBlock'
    // End of SystemInitialize for SubSystem: '<Root>/Subscribe'

    // SystemInitialize for Atomic SubSystem: '<Root>/Subscribe1'
    // SystemInitialize for Enabled SubSystem: '<S6>/Enabled Subsystem'
    // SystemInitialize for Outport: '<S13>/Out1'
    forward_kinematics_B.In1_b = forward_kinematics_P.Out1_Y0_e;

    // End of SystemInitialize for SubSystem: '<S6>/Enabled Subsystem'

    // Start for MATLABSystem: '<S6>/SourceBlock'
    forward_kinematics_DW.obj_l5.matlabCodegenIsDeleted = false;
    forward_kinematics_DW.obj_l5.isInitialized = 1;
    for (i = 0; i < 6; i++) {
      tmp[i] = tmp_1[i];
    }

    tmp[6] = '\x00';
    Sub_forward_kinematics_287.createSubscriber(tmp, 1);
    forward_kinematics_DW.obj_l5.isSetupComplete = true;

    // End of Start for MATLABSystem: '<S6>/SourceBlock'
    // End of SystemInitialize for SubSystem: '<Root>/Subscribe1'

    // SystemInitialize for Atomic SubSystem: '<Root>/Publish'
    // Start for MATLABSystem: '<S3>/SinkBlock'
    forward_kinematics_DW.obj_l.matlabCodegenIsDeleted = false;
    forward_kinematics_DW.obj_l.isInitialized = 1;
    for (i = 0; i < 14; i++) {
      forward_kinematics_B.cv[i] = tmp_2[i];
    }

    forward_kinematics_B.cv[14] = '\x00';
    Pub_forward_kinematics_282.createPublisher(forward_kinematics_B.cv, 1);
    forward_kinematics_DW.obj_l.isSetupComplete = true;

    // End of Start for MATLABSystem: '<S3>/SinkBlock'
    // End of SystemInitialize for SubSystem: '<Root>/Publish'

    // SystemInitialize for Atomic SubSystem: '<Root>/Publish1'
    // Start for MATLABSystem: '<S4>/SinkBlock'
    forward_kinematics_DW.obj_i.matlabCodegenIsDeleted = false;
    forward_kinematics_DW.obj_i.isInitialized = 1;
    for (i = 0; i < 14; i++) {
      forward_kinematics_B.cv[i] = tmp_3[i];
    }

    forward_kinematics_B.cv[14] = '\x00';
    Pub_forward_kinematics_283.createPublisher(forward_kinematics_B.cv, 1);
    forward_kinematics_DW.obj_i.isSetupComplete = true;

    // End of Start for MATLABSystem: '<S4>/SinkBlock'
    // End of SystemInitialize for SubSystem: '<Root>/Publish1'
    emxInitStruct_robotics_slmanip_(&forward_kinematics_DW.obj);
    emxInitStruct_n_robotics_manip_(&forward_kinematics_DW.gobj_1);
    emxInitStruct_n_robotics_manip_(&forward_kinematics_DW.gobj_16);
    emxInitStruct_n_robotics_manip_(&forward_kinematics_DW.gobj_15);
    emxInitStruct_n_robotics_manip_(&forward_kinematics_DW.gobj_14);
    emxInitStruct_n_robotics_manip_(&forward_kinematics_DW.gobj_13);
    emxInitStruct_n_robotics_manip_(&forward_kinematics_DW.gobj_12);
    emxInitStruct_n_robotics_manip_(&forward_kinematics_DW.gobj_11);
    emxInitStruct_n_robotics_manip_(&forward_kinematics_DW.gobj_10);
    emxInitStruct_n_robotics_manip_(&forward_kinematics_DW.gobj_9);
    emxInitStruct_n_robotics_manip_(&forward_kinematics_DW.gobj_8);
    emxInitStruct_n_robotics_manip_(&forward_kinematics_DW.gobj_7);
    emxInitStruct_n_robotics_manip_(&forward_kinematics_DW.gobj_6);
    emxInitStruct_n_robotics_manip_(&forward_kinematics_DW.gobj_5);
    emxInitStruct_n_robotics_manip_(&forward_kinematics_DW.gobj_4);
    emxInitStruct_n_robotics_manip_(&forward_kinematics_DW.gobj_3);
    emxInitStruct_n_robotics_manip_(&forward_kinematics_DW.gobj_2);

    // Start for MATLABSystem: '<S7>/MATLAB System'
    forward_kinematics_DW.obj.isInitialized = 0;
    forward_kinematics_DW.obj.isInitialized = 1;
    for_RigidBodyTree_RigidBodyTree(&forward_kinematics_DW.obj.TreeInternal,
      &forward_kinematics_DW.gobj_2, &forward_kinematics_DW.gobj_4,
      &forward_kinematics_DW.gobj_5, &forward_kinematics_DW.gobj_6,
      &forward_kinematics_DW.gobj_7, &forward_kinematics_DW.gobj_8,
      &forward_kinematics_DW.gobj_9, &forward_kinematics_DW.gobj_3);
    emxInitStruct_robotics_slmani_e(&forward_kinematics_DW.obj_o);
    emxInitStruct_n_robotics_mani_e(&forward_kinematics_DW.gobj_1_h);
    emxInitStruct_n_robotics_mani_e(&forward_kinematics_DW.gobj_16_n);
    emxInitStruct_n_robotics_mani_e(&forward_kinematics_DW.gobj_15_k);
    emxInitStruct_n_robotics_mani_e(&forward_kinematics_DW.gobj_14_p);
    emxInitStruct_n_robotics_mani_e(&forward_kinematics_DW.gobj_13_j);
    emxInitStruct_n_robotics_mani_e(&forward_kinematics_DW.gobj_12_j);
    emxInitStruct_n_robotics_mani_e(&forward_kinematics_DW.gobj_11_m);
    emxInitStruct_n_robotics_mani_e(&forward_kinematics_DW.gobj_10_l);
    emxInitStruct_n_robotics_mani_e(&forward_kinematics_DW.gobj_9_f);
    emxInitStruct_n_robotics_mani_e(&forward_kinematics_DW.gobj_8_b);
    emxInitStruct_n_robotics_mani_e(&forward_kinematics_DW.gobj_7_k);
    emxInitStruct_n_robotics_mani_e(&forward_kinematics_DW.gobj_6_m);
    emxInitStruct_n_robotics_mani_e(&forward_kinematics_DW.gobj_5_o);
    emxInitStruct_n_robotics_mani_e(&forward_kinematics_DW.gobj_4_p);
    emxInitStruct_n_robotics_mani_e(&forward_kinematics_DW.gobj_3_d);
    emxInitStruct_n_robotics_mani_e(&forward_kinematics_DW.gobj_2_g);

    // Start for MATLABSystem: '<S8>/MATLAB System'
    forward_kinematics_DW.obj_o.isInitialized = 0;
    forward_kinematics_DW.obj_o.isInitialized = 1;
    f_RigidBodyTree_RigidBodyTree_e(&forward_kinematics_DW.obj_o.TreeInternal,
      &forward_kinematics_DW.gobj_2_g, &forward_kinematics_DW.gobj_4_p,
      &forward_kinematics_DW.gobj_5_o, &forward_kinematics_DW.gobj_6_m,
      &forward_kinematics_DW.gobj_7_k, &forward_kinematics_DW.gobj_8_b,
      &forward_kinematics_DW.gobj_9_f, &forward_kinematics_DW.gobj_3_d);
  }
}

// Model terminate function
void forward_kinematics_terminate(void)
{
  // Terminate for Atomic SubSystem: '<Root>/Subscribe'
  // Terminate for MATLABSystem: '<S5>/SourceBlock'
  matlabCodegenHandle_matlabCo_ef(&forward_kinematics_DW.obj_b);

  // End of Terminate for SubSystem: '<Root>/Subscribe'
  emxFreeStruct_robotics_slmanip_(&forward_kinematics_DW.obj);
  emxFreeStruct_n_robotics_manip_(&forward_kinematics_DW.gobj_1);
  emxFreeStruct_n_robotics_manip_(&forward_kinematics_DW.gobj_16);
  emxFreeStruct_n_robotics_manip_(&forward_kinematics_DW.gobj_15);
  emxFreeStruct_n_robotics_manip_(&forward_kinematics_DW.gobj_14);
  emxFreeStruct_n_robotics_manip_(&forward_kinematics_DW.gobj_13);
  emxFreeStruct_n_robotics_manip_(&forward_kinematics_DW.gobj_12);
  emxFreeStruct_n_robotics_manip_(&forward_kinematics_DW.gobj_11);
  emxFreeStruct_n_robotics_manip_(&forward_kinematics_DW.gobj_10);
  emxFreeStruct_n_robotics_manip_(&forward_kinematics_DW.gobj_9);
  emxFreeStruct_n_robotics_manip_(&forward_kinematics_DW.gobj_8);
  emxFreeStruct_n_robotics_manip_(&forward_kinematics_DW.gobj_7);
  emxFreeStruct_n_robotics_manip_(&forward_kinematics_DW.gobj_6);
  emxFreeStruct_n_robotics_manip_(&forward_kinematics_DW.gobj_5);
  emxFreeStruct_n_robotics_manip_(&forward_kinematics_DW.gobj_4);
  emxFreeStruct_n_robotics_manip_(&forward_kinematics_DW.gobj_3);
  emxFreeStruct_n_robotics_manip_(&forward_kinematics_DW.gobj_2);
  emxFreeStruct_robotics_slmani_e(&forward_kinematics_DW.obj_o);
  emxFreeStruct_n_robotics_mani_e(&forward_kinematics_DW.gobj_1_h);
  emxFreeStruct_n_robotics_mani_e(&forward_kinematics_DW.gobj_16_n);
  emxFreeStruct_n_robotics_mani_e(&forward_kinematics_DW.gobj_15_k);
  emxFreeStruct_n_robotics_mani_e(&forward_kinematics_DW.gobj_14_p);
  emxFreeStruct_n_robotics_mani_e(&forward_kinematics_DW.gobj_13_j);
  emxFreeStruct_n_robotics_mani_e(&forward_kinematics_DW.gobj_12_j);
  emxFreeStruct_n_robotics_mani_e(&forward_kinematics_DW.gobj_11_m);
  emxFreeStruct_n_robotics_mani_e(&forward_kinematics_DW.gobj_10_l);
  emxFreeStruct_n_robotics_mani_e(&forward_kinematics_DW.gobj_9_f);
  emxFreeStruct_n_robotics_mani_e(&forward_kinematics_DW.gobj_8_b);
  emxFreeStruct_n_robotics_mani_e(&forward_kinematics_DW.gobj_7_k);
  emxFreeStruct_n_robotics_mani_e(&forward_kinematics_DW.gobj_6_m);
  emxFreeStruct_n_robotics_mani_e(&forward_kinematics_DW.gobj_5_o);
  emxFreeStruct_n_robotics_mani_e(&forward_kinematics_DW.gobj_4_p);
  emxFreeStruct_n_robotics_mani_e(&forward_kinematics_DW.gobj_3_d);
  emxFreeStruct_n_robotics_mani_e(&forward_kinematics_DW.gobj_2_g);

  // Terminate for Atomic SubSystem: '<Root>/Subscribe1'
  // Terminate for MATLABSystem: '<S6>/SourceBlock'
  matlabCodegenHandle_matlabCo_ef(&forward_kinematics_DW.obj_l5);

  // End of Terminate for SubSystem: '<Root>/Subscribe1'

  // Terminate for Atomic SubSystem: '<Root>/Publish'
  // Terminate for MATLABSystem: '<S3>/SinkBlock'
  matlabCodegenHandle_matlabCodeg(&forward_kinematics_DW.obj_l);

  // End of Terminate for SubSystem: '<Root>/Publish'

  // Terminate for Atomic SubSystem: '<Root>/Publish1'
  // Terminate for MATLABSystem: '<S4>/SinkBlock'
  matlabCodegenHandle_matlabCodeg(&forward_kinematics_DW.obj_i);

  // End of Terminate for SubSystem: '<Root>/Publish1'
}

//
// File trailer for generated code.
//
// [EOF]
//
