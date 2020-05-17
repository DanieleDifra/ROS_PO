//
// Academic License - for use in teaching, academic research, and meeting
// course requirements at degree granting institutions only.  Not for
// government, commercial, or other organizational use.
//
// File: dynamic_simulation.cpp
//
// Code generated for Simulink model 'dynamic_simulation'.
//
// Model version                  : 1.129
// Simulink Coder version         : 9.3 (R2020a) 18-Nov-2019
// C/C++ source code generated on : Sat May 16 00:53:14 2020
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
static void dynamic_simulat_SystemCore_step(boolean_T *varargout_1, real_T
  varargout_2_Data[128], uint32_T *varargout_2_Data_SL_Info_Curren, uint32_T
  *varargout_2_Data_SL_Info_Receiv, uint32_T *varargout_2_Layout_DataOffset,
  SL_Bus_dynamic_simulation_std_msgs_MultiArrayDimension varargout_2_Layout_Dim
  [16], uint32_T *varargout_2_Layout_Dim_SL_Info_, uint32_T
  *varargout_2_Layout_Dim_SL_Inf_0);
static void dynamic_simulati_emxInit_real_T(emxArray_real_T_dynamic_simul_T
  **pEmxArray, int32_T numDimensions);
static void dynami_emxEnsureCapacity_real_T(emxArray_real_T_dynamic_simul_T
  *emxArray, int32_T oldNumel);
static void dynamic_sim_emxInit_f_cell_wrap(emxArray_f_cell_wrap_dynamic__T
  **pEmxArray, int32_T numDimensions);
static void d_emxEnsureCapacity_f_cell_wrap(emxArray_f_cell_wrap_dynamic__T
  *emxArray, int32_T oldNumel);
static void dy_rigidBodyJoint_get_JointAxis(const
  c_rigidBodyJoint_dynamic_simu_T *obj, real_T ax[3]);
static void dynamic_simulation_cat(real_T varargin_1, real_T varargin_2, real_T
  varargin_3, real_T varargin_4, real_T varargin_5, real_T varargin_6, real_T
  varargin_7, real_T varargin_8, real_T varargin_9, real_T y[9]);
static void rigidBodyJoint_transformBodyT_k(const
  c_rigidBodyJoint_dynamic_simu_T *obj, const real_T q_data[], const int32_T
  *q_size, real_T T[16]);
static void rigidBodyJoint_transformBodyToP(const
  c_rigidBodyJoint_dynamic_simu_T *obj, real_T T[16]);
static void dynamic_simulation_tforminv(const real_T T[16], real_T Tinv[16]);
static void dynamic_sim_tformToSpatialXform(const real_T T[16], real_T X[36]);
static void dynamic_simulati_emxInit_char_T(emxArray_char_T_dynamic_simul_T
  **pEmxArray, int32_T numDimensions);
static void dynami_emxEnsureCapacity_char_T(emxArray_char_T_dynamic_simul_T
  *emxArray, int32_T oldNumel);
static void dynamic_simulati_emxFree_char_T(emxArray_char_T_dynamic_simul_T
  **pEmxArray);
static void dynamic_simulati_emxFree_real_T(emxArray_real_T_dynamic_simul_T
  **pEmxArray);
static void dynamic_sim_emxFree_f_cell_wrap(emxArray_f_cell_wrap_dynamic__T
  **pEmxArray);
static void RigidBodyTreeDynamics_massMatri(p_robotics_manip_internal_Rig_T
  *robot, const real_T q[6], emxArray_real_T_dynamic_simul_T *H,
  emxArray_real_T_dynamic_simul_T *lambda);
static void RigidBodyTreeDynamics_inverseDy(p_robotics_manip_internal_Rig_T
  *robot, const real_T q[6], const real_T qdot[6], const real_T fext[48], real_T
  tau[6]);
static void matlabCodegenHandle_matlabCodeg(ros_slros_internal_block_GetP_T *obj);
static void dynamic_simul_matlabCodegenHa_i(ros_slros_internal_block_Subs_T *obj);
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
  for (dynamic_simulation_B.i_ip = 0; dynamic_simulation_B.i_ip < numDimensions;
       dynamic_simulation_B.i_ip++) {
    emxArray->size[dynamic_simulation_B.i_ip] = 0;
  }
}

static void dynami_emxEnsureCapacity_real_T(emxArray_real_T_dynamic_simul_T
  *emxArray, int32_T oldNumel)
{
  void *newData;
  if (oldNumel < 0) {
    oldNumel = 0;
  }

  dynamic_simulation_B.newNumel = 1;
  for (dynamic_simulation_B.i_a = 0; dynamic_simulation_B.i_a <
       emxArray->numDimensions; dynamic_simulation_B.i_a++) {
    dynamic_simulation_B.newNumel *= emxArray->size[dynamic_simulation_B.i_a];
  }

  if (dynamic_simulation_B.newNumel > emxArray->allocatedSize) {
    dynamic_simulation_B.i_a = emxArray->allocatedSize;
    if (dynamic_simulation_B.i_a < 16) {
      dynamic_simulation_B.i_a = 16;
    }

    while (dynamic_simulation_B.i_a < dynamic_simulation_B.newNumel) {
      if (dynamic_simulation_B.i_a > 1073741823) {
        dynamic_simulation_B.i_a = MAX_int32_T;
      } else {
        dynamic_simulation_B.i_a <<= 1;
      }
    }

    newData = calloc(static_cast<uint32_T>(dynamic_simulation_B.i_a), sizeof
                     (real_T));
    if (emxArray->data != NULL) {
      memcpy(newData, emxArray->data, sizeof(real_T) * oldNumel);
      if (emxArray->canFreeData) {
        free(emxArray->data);
      }
    }

    emxArray->data = (real_T *)newData;
    emxArray->allocatedSize = dynamic_simulation_B.i_a;
    emxArray->canFreeData = true;
  }
}

static void dynamic_sim_emxInit_f_cell_wrap(emxArray_f_cell_wrap_dynamic__T
  **pEmxArray, int32_T numDimensions)
{
  emxArray_f_cell_wrap_dynamic__T *emxArray;
  *pEmxArray = (emxArray_f_cell_wrap_dynamic__T *)malloc(sizeof
    (emxArray_f_cell_wrap_dynamic__T));
  emxArray = *pEmxArray;
  emxArray->data = (f_cell_wrap_dynamic_simulatio_T *)NULL;
  emxArray->numDimensions = numDimensions;
  emxArray->size = (int32_T *)malloc(sizeof(int32_T) * numDimensions);
  emxArray->allocatedSize = 0;
  emxArray->canFreeData = true;
  for (dynamic_simulation_B.i_f = 0; dynamic_simulation_B.i_f < numDimensions;
       dynamic_simulation_B.i_f++) {
    emxArray->size[dynamic_simulation_B.i_f] = 0;
  }
}

static void d_emxEnsureCapacity_f_cell_wrap(emxArray_f_cell_wrap_dynamic__T
  *emxArray, int32_T oldNumel)
{
  void *newData;
  if (oldNumel < 0) {
    oldNumel = 0;
  }

  dynamic_simulation_B.newNumel_l = 1;
  for (dynamic_simulation_B.i_oj = 0; dynamic_simulation_B.i_oj <
       emxArray->numDimensions; dynamic_simulation_B.i_oj++) {
    dynamic_simulation_B.newNumel_l *= emxArray->size[dynamic_simulation_B.i_oj];
  }

  if (dynamic_simulation_B.newNumel_l > emxArray->allocatedSize) {
    dynamic_simulation_B.i_oj = emxArray->allocatedSize;
    if (dynamic_simulation_B.i_oj < 16) {
      dynamic_simulation_B.i_oj = 16;
    }

    while (dynamic_simulation_B.i_oj < dynamic_simulation_B.newNumel_l) {
      if (dynamic_simulation_B.i_oj > 1073741823) {
        dynamic_simulation_B.i_oj = MAX_int32_T;
      } else {
        dynamic_simulation_B.i_oj <<= 1;
      }
    }

    newData = calloc(static_cast<uint32_T>(dynamic_simulation_B.i_oj), sizeof
                     (f_cell_wrap_dynamic_simulatio_T));
    if (emxArray->data != NULL) {
      memcpy(newData, emxArray->data, sizeof(f_cell_wrap_dynamic_simulatio_T)
             * oldNumel);
      if (emxArray->canFreeData) {
        free(emxArray->data);
      }
    }

    emxArray->data = (f_cell_wrap_dynamic_simulatio_T *)newData;
    emxArray->allocatedSize = dynamic_simulation_B.i_oj;
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
  for (dynamic_simulation_B.b_kstr_c = 0; dynamic_simulation_B.b_kstr_c < 8;
       dynamic_simulation_B.b_kstr_c++) {
    dynamic_simulation_B.b_o[dynamic_simulation_B.b_kstr_c] =
      tmp[dynamic_simulation_B.b_kstr_c];
  }

  dynamic_simulation_B.b_bool_l = false;
  if (obj->Type->size[1] == 8) {
    dynamic_simulation_B.b_kstr_c = 1;
    do {
      exitg1 = 0;
      if (dynamic_simulation_B.b_kstr_c - 1 < 8) {
        dynamic_simulation_B.kstr_m = dynamic_simulation_B.b_kstr_c - 1;
        if (obj->Type->data[dynamic_simulation_B.kstr_m] !=
            dynamic_simulation_B.b_o[dynamic_simulation_B.kstr_m]) {
          exitg1 = 1;
        } else {
          dynamic_simulation_B.b_kstr_c++;
        }
      } else {
        dynamic_simulation_B.b_bool_l = true;
        exitg1 = 1;
      }
    } while (exitg1 == 0);
  }

  guard1 = false;
  if (dynamic_simulation_B.b_bool_l) {
    guard1 = true;
  } else {
    for (dynamic_simulation_B.b_kstr_c = 0; dynamic_simulation_B.b_kstr_c < 9;
         dynamic_simulation_B.b_kstr_c++) {
      dynamic_simulation_B.b[dynamic_simulation_B.b_kstr_c] =
        tmp_0[dynamic_simulation_B.b_kstr_c];
    }

    dynamic_simulation_B.b_bool_l = false;
    if (obj->Type->size[1] == 9) {
      dynamic_simulation_B.b_kstr_c = 1;
      do {
        exitg1 = 0;
        if (dynamic_simulation_B.b_kstr_c - 1 < 9) {
          dynamic_simulation_B.kstr_m = dynamic_simulation_B.b_kstr_c - 1;
          if (obj->Type->data[dynamic_simulation_B.kstr_m] !=
              dynamic_simulation_B.b[dynamic_simulation_B.kstr_m]) {
            exitg1 = 1;
          } else {
            dynamic_simulation_B.b_kstr_c++;
          }
        } else {
          dynamic_simulation_B.b_bool_l = true;
          exitg1 = 1;
        }
      } while (exitg1 == 0);
    }

    if (dynamic_simulation_B.b_bool_l) {
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
  c_rigidBodyJoint_dynamic_simu_T *obj, const real_T q_data[], const int32_T
  *q_size, real_T T[16])
{
  static const char_T tmp[5] = { 'f', 'i', 'x', 'e', 'd' };

  static const char_T tmp_0[8] = { 'r', 'e', 'v', 'o', 'l', 'u', 't', 'e' };

  int32_T exitg1;
  for (dynamic_simulation_B.b_kstr = 0; dynamic_simulation_B.b_kstr < 5;
       dynamic_simulation_B.b_kstr++) {
    dynamic_simulation_B.b_jz[dynamic_simulation_B.b_kstr] =
      tmp[dynamic_simulation_B.b_kstr];
  }

  dynamic_simulation_B.b_bool_o = false;
  if (obj->Type->size[1] == 5) {
    dynamic_simulation_B.b_kstr = 1;
    do {
      exitg1 = 0;
      if (dynamic_simulation_B.b_kstr - 1 < 5) {
        dynamic_simulation_B.kstr = dynamic_simulation_B.b_kstr - 1;
        if (obj->Type->data[dynamic_simulation_B.kstr] !=
            dynamic_simulation_B.b_jz[dynamic_simulation_B.kstr]) {
          exitg1 = 1;
        } else {
          dynamic_simulation_B.b_kstr++;
        }
      } else {
        dynamic_simulation_B.b_bool_o = true;
        exitg1 = 1;
      }
    } while (exitg1 == 0);
  }

  if (dynamic_simulation_B.b_bool_o) {
    dynamic_simulation_B.b_kstr = 0;
  } else {
    for (dynamic_simulation_B.b_kstr = 0; dynamic_simulation_B.b_kstr < 8;
         dynamic_simulation_B.b_kstr++) {
      dynamic_simulation_B.b_l[dynamic_simulation_B.b_kstr] =
        tmp_0[dynamic_simulation_B.b_kstr];
    }

    dynamic_simulation_B.b_bool_o = false;
    if (obj->Type->size[1] == 8) {
      dynamic_simulation_B.b_kstr = 1;
      do {
        exitg1 = 0;
        if (dynamic_simulation_B.b_kstr - 1 < 8) {
          dynamic_simulation_B.kstr = dynamic_simulation_B.b_kstr - 1;
          if (obj->Type->data[dynamic_simulation_B.kstr] !=
              dynamic_simulation_B.b_l[dynamic_simulation_B.kstr]) {
            exitg1 = 1;
          } else {
            dynamic_simulation_B.b_kstr++;
          }
        } else {
          dynamic_simulation_B.b_bool_o = true;
          exitg1 = 1;
        }
      } while (exitg1 == 0);
    }

    if (dynamic_simulation_B.b_bool_o) {
      dynamic_simulation_B.b_kstr = 1;
    } else {
      dynamic_simulation_B.b_kstr = -1;
    }
  }

  switch (dynamic_simulation_B.b_kstr) {
   case 0:
    memset(&dynamic_simulation_B.TJ[0], 0, sizeof(real_T) << 4U);
    dynamic_simulation_B.TJ[0] = 1.0;
    dynamic_simulation_B.TJ[5] = 1.0;
    dynamic_simulation_B.TJ[10] = 1.0;
    dynamic_simulation_B.TJ[15] = 1.0;
    break;

   case 1:
    dy_rigidBodyJoint_get_JointAxis(obj, dynamic_simulation_B.v);
    dynamic_simulation_B.result_data[0] = dynamic_simulation_B.v[0];
    dynamic_simulation_B.result_data[1] = dynamic_simulation_B.v[1];
    dynamic_simulation_B.result_data[2] = dynamic_simulation_B.v[2];
    if (0 <= (*q_size != 0) - 1) {
      dynamic_simulation_B.result_data[3] = q_data[0];
    }

    dynamic_simulation_B.cth = 1.0 / sqrt((dynamic_simulation_B.result_data[0] *
      dynamic_simulation_B.result_data[0] + dynamic_simulation_B.result_data[1] *
      dynamic_simulation_B.result_data[1]) + dynamic_simulation_B.result_data[2]
      * dynamic_simulation_B.result_data[2]);
    dynamic_simulation_B.v[0] = dynamic_simulation_B.result_data[0] *
      dynamic_simulation_B.cth;
    dynamic_simulation_B.v[1] = dynamic_simulation_B.result_data[1] *
      dynamic_simulation_B.cth;
    dynamic_simulation_B.v[2] = dynamic_simulation_B.result_data[2] *
      dynamic_simulation_B.cth;
    dynamic_simulation_B.cth = cos(dynamic_simulation_B.result_data[3]);
    dynamic_simulation_B.sth = sin(dynamic_simulation_B.result_data[3]);
    dynamic_simulation_B.tempR_tmp = dynamic_simulation_B.v[1] *
      dynamic_simulation_B.v[0] * (1.0 - dynamic_simulation_B.cth);
    dynamic_simulation_B.tempR_tmp_l = dynamic_simulation_B.v[2] *
      dynamic_simulation_B.sth;
    dynamic_simulation_B.tempR_tmp_h = dynamic_simulation_B.v[2] *
      dynamic_simulation_B.v[0] * (1.0 - dynamic_simulation_B.cth);
    dynamic_simulation_B.tempR_tmp_b = dynamic_simulation_B.v[1] *
      dynamic_simulation_B.sth;
    dynamic_simulation_B.tempR_tmp_d = dynamic_simulation_B.v[2] *
      dynamic_simulation_B.v[1] * (1.0 - dynamic_simulation_B.cth);
    dynamic_simulation_B.sth *= dynamic_simulation_B.v[0];
    dynamic_simulation_cat(dynamic_simulation_B.v[0] * dynamic_simulation_B.v[0]
      * (1.0 - dynamic_simulation_B.cth) + dynamic_simulation_B.cth,
      dynamic_simulation_B.tempR_tmp - dynamic_simulation_B.tempR_tmp_l,
      dynamic_simulation_B.tempR_tmp_h + dynamic_simulation_B.tempR_tmp_b,
      dynamic_simulation_B.tempR_tmp + dynamic_simulation_B.tempR_tmp_l,
      dynamic_simulation_B.v[1] * dynamic_simulation_B.v[1] * (1.0 -
      dynamic_simulation_B.cth) + dynamic_simulation_B.cth,
      dynamic_simulation_B.tempR_tmp_d - dynamic_simulation_B.sth,
      dynamic_simulation_B.tempR_tmp_h - dynamic_simulation_B.tempR_tmp_b,
      dynamic_simulation_B.tempR_tmp_d + dynamic_simulation_B.sth,
      dynamic_simulation_B.v[2] * dynamic_simulation_B.v[2] * (1.0 -
      dynamic_simulation_B.cth) + dynamic_simulation_B.cth,
      dynamic_simulation_B.tempR);
    for (dynamic_simulation_B.b_kstr = 0; dynamic_simulation_B.b_kstr < 3;
         dynamic_simulation_B.b_kstr++) {
      dynamic_simulation_B.kstr = dynamic_simulation_B.b_kstr + 1;
      dynamic_simulation_B.R_p[dynamic_simulation_B.kstr - 1] =
        dynamic_simulation_B.tempR[(dynamic_simulation_B.kstr - 1) * 3];
      dynamic_simulation_B.kstr = dynamic_simulation_B.b_kstr + 1;
      dynamic_simulation_B.R_p[dynamic_simulation_B.kstr + 2] =
        dynamic_simulation_B.tempR[(dynamic_simulation_B.kstr - 1) * 3 + 1];
      dynamic_simulation_B.kstr = dynamic_simulation_B.b_kstr + 1;
      dynamic_simulation_B.R_p[dynamic_simulation_B.kstr + 5] =
        dynamic_simulation_B.tempR[(dynamic_simulation_B.kstr - 1) * 3 + 2];
    }

    memset(&dynamic_simulation_B.TJ[0], 0, sizeof(real_T) << 4U);
    for (dynamic_simulation_B.b_kstr = 0; dynamic_simulation_B.b_kstr < 3;
         dynamic_simulation_B.b_kstr++) {
      dynamic_simulation_B.kstr = dynamic_simulation_B.b_kstr << 2;
      dynamic_simulation_B.TJ[dynamic_simulation_B.kstr] =
        dynamic_simulation_B.R_p[3 * dynamic_simulation_B.b_kstr];
      dynamic_simulation_B.TJ[dynamic_simulation_B.kstr + 1] =
        dynamic_simulation_B.R_p[3 * dynamic_simulation_B.b_kstr + 1];
      dynamic_simulation_B.TJ[dynamic_simulation_B.kstr + 2] =
        dynamic_simulation_B.R_p[3 * dynamic_simulation_B.b_kstr + 2];
    }

    dynamic_simulation_B.TJ[15] = 1.0;
    break;

   default:
    dy_rigidBodyJoint_get_JointAxis(obj, dynamic_simulation_B.v);
    memset(&dynamic_simulation_B.tempR[0], 0, 9U * sizeof(real_T));
    dynamic_simulation_B.tempR[0] = 1.0;
    dynamic_simulation_B.tempR[4] = 1.0;
    dynamic_simulation_B.tempR[8] = 1.0;
    for (dynamic_simulation_B.b_kstr = 0; dynamic_simulation_B.b_kstr < 3;
         dynamic_simulation_B.b_kstr++) {
      dynamic_simulation_B.kstr = dynamic_simulation_B.b_kstr << 2;
      dynamic_simulation_B.TJ[dynamic_simulation_B.kstr] =
        dynamic_simulation_B.tempR[3 * dynamic_simulation_B.b_kstr];
      dynamic_simulation_B.TJ[dynamic_simulation_B.kstr + 1] =
        dynamic_simulation_B.tempR[3 * dynamic_simulation_B.b_kstr + 1];
      dynamic_simulation_B.TJ[dynamic_simulation_B.kstr + 2] =
        dynamic_simulation_B.tempR[3 * dynamic_simulation_B.b_kstr + 2];
      dynamic_simulation_B.TJ[dynamic_simulation_B.b_kstr + 12] =
        dynamic_simulation_B.v[dynamic_simulation_B.b_kstr] * q_data[0];
    }

    dynamic_simulation_B.TJ[3] = 0.0;
    dynamic_simulation_B.TJ[7] = 0.0;
    dynamic_simulation_B.TJ[11] = 0.0;
    dynamic_simulation_B.TJ[15] = 1.0;
    break;
  }

  for (dynamic_simulation_B.b_kstr = 0; dynamic_simulation_B.b_kstr < 4;
       dynamic_simulation_B.b_kstr++) {
    for (dynamic_simulation_B.kstr = 0; dynamic_simulation_B.kstr < 4;
         dynamic_simulation_B.kstr++) {
      dynamic_simulation_B.obj_tmp_tmp = dynamic_simulation_B.kstr << 2;
      dynamic_simulation_B.obj_tmp = dynamic_simulation_B.b_kstr +
        dynamic_simulation_B.obj_tmp_tmp;
      dynamic_simulation_B.obj[dynamic_simulation_B.obj_tmp] = 0.0;
      dynamic_simulation_B.obj[dynamic_simulation_B.obj_tmp] +=
        dynamic_simulation_B.TJ[dynamic_simulation_B.obj_tmp_tmp] *
        obj->JointToParentTransform[dynamic_simulation_B.b_kstr];
      dynamic_simulation_B.obj[dynamic_simulation_B.obj_tmp] +=
        dynamic_simulation_B.TJ[dynamic_simulation_B.obj_tmp_tmp + 1] *
        obj->JointToParentTransform[dynamic_simulation_B.b_kstr + 4];
      dynamic_simulation_B.obj[dynamic_simulation_B.obj_tmp] +=
        dynamic_simulation_B.TJ[dynamic_simulation_B.obj_tmp_tmp + 2] *
        obj->JointToParentTransform[dynamic_simulation_B.b_kstr + 8];
      dynamic_simulation_B.obj[dynamic_simulation_B.obj_tmp] +=
        dynamic_simulation_B.TJ[dynamic_simulation_B.obj_tmp_tmp + 3] *
        obj->JointToParentTransform[dynamic_simulation_B.b_kstr + 12];
    }

    for (dynamic_simulation_B.kstr = 0; dynamic_simulation_B.kstr < 4;
         dynamic_simulation_B.kstr++) {
      dynamic_simulation_B.obj_tmp_tmp = dynamic_simulation_B.kstr << 2;
      dynamic_simulation_B.obj_tmp = dynamic_simulation_B.b_kstr +
        dynamic_simulation_B.obj_tmp_tmp;
      T[dynamic_simulation_B.obj_tmp] = 0.0;
      T[dynamic_simulation_B.obj_tmp] += obj->
        ChildToJointTransform[dynamic_simulation_B.obj_tmp_tmp] *
        dynamic_simulation_B.obj[dynamic_simulation_B.b_kstr];
      T[dynamic_simulation_B.obj_tmp] += obj->
        ChildToJointTransform[dynamic_simulation_B.obj_tmp_tmp + 1] *
        dynamic_simulation_B.obj[dynamic_simulation_B.b_kstr + 4];
      T[dynamic_simulation_B.obj_tmp] += obj->
        ChildToJointTransform[dynamic_simulation_B.obj_tmp_tmp + 2] *
        dynamic_simulation_B.obj[dynamic_simulation_B.b_kstr + 8];
      T[dynamic_simulation_B.obj_tmp] += obj->
        ChildToJointTransform[dynamic_simulation_B.obj_tmp_tmp + 3] *
        dynamic_simulation_B.obj[dynamic_simulation_B.b_kstr + 12];
    }
  }
}

static void rigidBodyJoint_transformBodyToP(const
  c_rigidBodyJoint_dynamic_simu_T *obj, real_T T[16])
{
  static const char_T tmp[5] = { 'f', 'i', 'x', 'e', 'd' };

  static const char_T tmp_0[8] = { 'r', 'e', 'v', 'o', 'l', 'u', 't', 'e' };

  int32_T exitg1;
  for (dynamic_simulation_B.b_kstr_ct = 0; dynamic_simulation_B.b_kstr_ct < 5;
       dynamic_simulation_B.b_kstr_ct++) {
    dynamic_simulation_B.b_n[dynamic_simulation_B.b_kstr_ct] =
      tmp[dynamic_simulation_B.b_kstr_ct];
  }

  dynamic_simulation_B.b_bool_mj = false;
  if (obj->Type->size[1] == 5) {
    dynamic_simulation_B.b_kstr_ct = 1;
    do {
      exitg1 = 0;
      if (dynamic_simulation_B.b_kstr_ct - 1 < 5) {
        dynamic_simulation_B.kstr_c = dynamic_simulation_B.b_kstr_ct - 1;
        if (obj->Type->data[dynamic_simulation_B.kstr_c] !=
            dynamic_simulation_B.b_n[dynamic_simulation_B.kstr_c]) {
          exitg1 = 1;
        } else {
          dynamic_simulation_B.b_kstr_ct++;
        }
      } else {
        dynamic_simulation_B.b_bool_mj = true;
        exitg1 = 1;
      }
    } while (exitg1 == 0);
  }

  if (dynamic_simulation_B.b_bool_mj) {
    dynamic_simulation_B.b_kstr_ct = 0;
  } else {
    for (dynamic_simulation_B.b_kstr_ct = 0; dynamic_simulation_B.b_kstr_ct < 8;
         dynamic_simulation_B.b_kstr_ct++) {
      dynamic_simulation_B.b_b[dynamic_simulation_B.b_kstr_ct] =
        tmp_0[dynamic_simulation_B.b_kstr_ct];
    }

    dynamic_simulation_B.b_bool_mj = false;
    if (obj->Type->size[1] == 8) {
      dynamic_simulation_B.b_kstr_ct = 1;
      do {
        exitg1 = 0;
        if (dynamic_simulation_B.b_kstr_ct - 1 < 8) {
          dynamic_simulation_B.kstr_c = dynamic_simulation_B.b_kstr_ct - 1;
          if (obj->Type->data[dynamic_simulation_B.kstr_c] !=
              dynamic_simulation_B.b_b[dynamic_simulation_B.kstr_c]) {
            exitg1 = 1;
          } else {
            dynamic_simulation_B.b_kstr_ct++;
          }
        } else {
          dynamic_simulation_B.b_bool_mj = true;
          exitg1 = 1;
        }
      } while (exitg1 == 0);
    }

    if (dynamic_simulation_B.b_bool_mj) {
      dynamic_simulation_B.b_kstr_ct = 1;
    } else {
      dynamic_simulation_B.b_kstr_ct = -1;
    }
  }

  switch (dynamic_simulation_B.b_kstr_ct) {
   case 0:
    memset(&dynamic_simulation_B.TJ_c[0], 0, sizeof(real_T) << 4U);
    dynamic_simulation_B.TJ_c[0] = 1.0;
    dynamic_simulation_B.TJ_c[5] = 1.0;
    dynamic_simulation_B.TJ_c[10] = 1.0;
    dynamic_simulation_B.TJ_c[15] = 1.0;
    break;

   case 1:
    dy_rigidBodyJoint_get_JointAxis(obj, dynamic_simulation_B.v_d);
    dynamic_simulation_B.axang_idx_0 = dynamic_simulation_B.v_d[0];
    dynamic_simulation_B.axang_idx_1 = dynamic_simulation_B.v_d[1];
    dynamic_simulation_B.axang_idx_2 = dynamic_simulation_B.v_d[2];
    dynamic_simulation_B.b_a = 1.0 / sqrt((dynamic_simulation_B.axang_idx_0 *
      dynamic_simulation_B.axang_idx_0 + dynamic_simulation_B.axang_idx_1 *
      dynamic_simulation_B.axang_idx_1) + dynamic_simulation_B.axang_idx_2 *
      dynamic_simulation_B.axang_idx_2);
    dynamic_simulation_B.v_d[0] = dynamic_simulation_B.axang_idx_0 *
      dynamic_simulation_B.b_a;
    dynamic_simulation_B.v_d[1] = dynamic_simulation_B.axang_idx_1 *
      dynamic_simulation_B.b_a;
    dynamic_simulation_B.v_d[2] = dynamic_simulation_B.axang_idx_2 *
      dynamic_simulation_B.b_a;
    dynamic_simulation_B.axang_idx_0 = dynamic_simulation_B.v_d[1] *
      dynamic_simulation_B.v_d[0] * 0.0;
    dynamic_simulation_B.axang_idx_1 = dynamic_simulation_B.v_d[2] *
      dynamic_simulation_B.v_d[0] * 0.0;
    dynamic_simulation_B.axang_idx_2 = dynamic_simulation_B.v_d[2] *
      dynamic_simulation_B.v_d[1] * 0.0;
    dynamic_simulation_cat(dynamic_simulation_B.v_d[0] *
      dynamic_simulation_B.v_d[0] * 0.0 + 1.0, dynamic_simulation_B.axang_idx_0
      - dynamic_simulation_B.v_d[2] * 0.0, dynamic_simulation_B.axang_idx_1 +
      dynamic_simulation_B.v_d[1] * 0.0, dynamic_simulation_B.axang_idx_0 +
      dynamic_simulation_B.v_d[2] * 0.0, dynamic_simulation_B.v_d[1] *
      dynamic_simulation_B.v_d[1] * 0.0 + 1.0, dynamic_simulation_B.axang_idx_2
      - dynamic_simulation_B.v_d[0] * 0.0, dynamic_simulation_B.axang_idx_1 -
      dynamic_simulation_B.v_d[1] * 0.0, dynamic_simulation_B.axang_idx_2 +
      dynamic_simulation_B.v_d[0] * 0.0, dynamic_simulation_B.v_d[2] *
      dynamic_simulation_B.v_d[2] * 0.0 + 1.0, dynamic_simulation_B.tempR_f);
    for (dynamic_simulation_B.b_kstr_ct = 0; dynamic_simulation_B.b_kstr_ct < 3;
         dynamic_simulation_B.b_kstr_ct++) {
      dynamic_simulation_B.kstr_c = dynamic_simulation_B.b_kstr_ct + 1;
      dynamic_simulation_B.R_cv[dynamic_simulation_B.kstr_c - 1] =
        dynamic_simulation_B.tempR_f[(dynamic_simulation_B.kstr_c - 1) * 3];
      dynamic_simulation_B.kstr_c = dynamic_simulation_B.b_kstr_ct + 1;
      dynamic_simulation_B.R_cv[dynamic_simulation_B.kstr_c + 2] =
        dynamic_simulation_B.tempR_f[(dynamic_simulation_B.kstr_c - 1) * 3 + 1];
      dynamic_simulation_B.kstr_c = dynamic_simulation_B.b_kstr_ct + 1;
      dynamic_simulation_B.R_cv[dynamic_simulation_B.kstr_c + 5] =
        dynamic_simulation_B.tempR_f[(dynamic_simulation_B.kstr_c - 1) * 3 + 2];
    }

    memset(&dynamic_simulation_B.TJ_c[0], 0, sizeof(real_T) << 4U);
    for (dynamic_simulation_B.b_kstr_ct = 0; dynamic_simulation_B.b_kstr_ct < 3;
         dynamic_simulation_B.b_kstr_ct++) {
      dynamic_simulation_B.kstr_c = dynamic_simulation_B.b_kstr_ct << 2;
      dynamic_simulation_B.TJ_c[dynamic_simulation_B.kstr_c] =
        dynamic_simulation_B.R_cv[3 * dynamic_simulation_B.b_kstr_ct];
      dynamic_simulation_B.TJ_c[dynamic_simulation_B.kstr_c + 1] =
        dynamic_simulation_B.R_cv[3 * dynamic_simulation_B.b_kstr_ct + 1];
      dynamic_simulation_B.TJ_c[dynamic_simulation_B.kstr_c + 2] =
        dynamic_simulation_B.R_cv[3 * dynamic_simulation_B.b_kstr_ct + 2];
    }

    dynamic_simulation_B.TJ_c[15] = 1.0;
    break;

   default:
    dy_rigidBodyJoint_get_JointAxis(obj, dynamic_simulation_B.v_d);
    memset(&dynamic_simulation_B.tempR_f[0], 0, 9U * sizeof(real_T));
    dynamic_simulation_B.tempR_f[0] = 1.0;
    dynamic_simulation_B.tempR_f[4] = 1.0;
    dynamic_simulation_B.tempR_f[8] = 1.0;
    for (dynamic_simulation_B.b_kstr_ct = 0; dynamic_simulation_B.b_kstr_ct < 3;
         dynamic_simulation_B.b_kstr_ct++) {
      dynamic_simulation_B.kstr_c = dynamic_simulation_B.b_kstr_ct << 2;
      dynamic_simulation_B.TJ_c[dynamic_simulation_B.kstr_c] =
        dynamic_simulation_B.tempR_f[3 * dynamic_simulation_B.b_kstr_ct];
      dynamic_simulation_B.TJ_c[dynamic_simulation_B.kstr_c + 1] =
        dynamic_simulation_B.tempR_f[3 * dynamic_simulation_B.b_kstr_ct + 1];
      dynamic_simulation_B.TJ_c[dynamic_simulation_B.kstr_c + 2] =
        dynamic_simulation_B.tempR_f[3 * dynamic_simulation_B.b_kstr_ct + 2];
      dynamic_simulation_B.TJ_c[dynamic_simulation_B.b_kstr_ct + 12] =
        dynamic_simulation_B.v_d[dynamic_simulation_B.b_kstr_ct] * 0.0;
    }

    dynamic_simulation_B.TJ_c[3] = 0.0;
    dynamic_simulation_B.TJ_c[7] = 0.0;
    dynamic_simulation_B.TJ_c[11] = 0.0;
    dynamic_simulation_B.TJ_c[15] = 1.0;
    break;
  }

  for (dynamic_simulation_B.b_kstr_ct = 0; dynamic_simulation_B.b_kstr_ct < 4;
       dynamic_simulation_B.b_kstr_ct++) {
    for (dynamic_simulation_B.kstr_c = 0; dynamic_simulation_B.kstr_c < 4;
         dynamic_simulation_B.kstr_c++) {
      dynamic_simulation_B.obj_tmp_tmp_p = dynamic_simulation_B.kstr_c << 2;
      dynamic_simulation_B.obj_tmp_p = dynamic_simulation_B.b_kstr_ct +
        dynamic_simulation_B.obj_tmp_tmp_p;
      dynamic_simulation_B.obj_k[dynamic_simulation_B.obj_tmp_p] = 0.0;
      dynamic_simulation_B.obj_k[dynamic_simulation_B.obj_tmp_p] +=
        dynamic_simulation_B.TJ_c[dynamic_simulation_B.obj_tmp_tmp_p] *
        obj->JointToParentTransform[dynamic_simulation_B.b_kstr_ct];
      dynamic_simulation_B.obj_k[dynamic_simulation_B.obj_tmp_p] +=
        dynamic_simulation_B.TJ_c[dynamic_simulation_B.obj_tmp_tmp_p + 1] *
        obj->JointToParentTransform[dynamic_simulation_B.b_kstr_ct + 4];
      dynamic_simulation_B.obj_k[dynamic_simulation_B.obj_tmp_p] +=
        dynamic_simulation_B.TJ_c[dynamic_simulation_B.obj_tmp_tmp_p + 2] *
        obj->JointToParentTransform[dynamic_simulation_B.b_kstr_ct + 8];
      dynamic_simulation_B.obj_k[dynamic_simulation_B.obj_tmp_p] +=
        dynamic_simulation_B.TJ_c[dynamic_simulation_B.obj_tmp_tmp_p + 3] *
        obj->JointToParentTransform[dynamic_simulation_B.b_kstr_ct + 12];
    }

    for (dynamic_simulation_B.kstr_c = 0; dynamic_simulation_B.kstr_c < 4;
         dynamic_simulation_B.kstr_c++) {
      dynamic_simulation_B.obj_tmp_tmp_p = dynamic_simulation_B.kstr_c << 2;
      dynamic_simulation_B.obj_tmp_p = dynamic_simulation_B.b_kstr_ct +
        dynamic_simulation_B.obj_tmp_tmp_p;
      T[dynamic_simulation_B.obj_tmp_p] = 0.0;
      T[dynamic_simulation_B.obj_tmp_p] += obj->
        ChildToJointTransform[dynamic_simulation_B.obj_tmp_tmp_p] *
        dynamic_simulation_B.obj_k[dynamic_simulation_B.b_kstr_ct];
      T[dynamic_simulation_B.obj_tmp_p] += obj->
        ChildToJointTransform[dynamic_simulation_B.obj_tmp_tmp_p + 1] *
        dynamic_simulation_B.obj_k[dynamic_simulation_B.b_kstr_ct + 4];
      T[dynamic_simulation_B.obj_tmp_p] += obj->
        ChildToJointTransform[dynamic_simulation_B.obj_tmp_tmp_p + 2] *
        dynamic_simulation_B.obj_k[dynamic_simulation_B.b_kstr_ct + 8];
      T[dynamic_simulation_B.obj_tmp_p] += obj->
        ChildToJointTransform[dynamic_simulation_B.obj_tmp_tmp_p + 3] *
        dynamic_simulation_B.obj_k[dynamic_simulation_B.b_kstr_ct + 12];
    }
  }
}

static void dynamic_simulation_tforminv(const real_T T[16], real_T Tinv[16])
{
  for (dynamic_simulation_B.i3 = 0; dynamic_simulation_B.i3 < 3;
       dynamic_simulation_B.i3++) {
    dynamic_simulation_B.R_g[3 * dynamic_simulation_B.i3] =
      T[dynamic_simulation_B.i3];
    dynamic_simulation_B.R_g[3 * dynamic_simulation_B.i3 + 1] =
      T[dynamic_simulation_B.i3 + 4];
    dynamic_simulation_B.R_g[3 * dynamic_simulation_B.i3 + 2] =
      T[dynamic_simulation_B.i3 + 8];
  }

  for (dynamic_simulation_B.i3 = 0; dynamic_simulation_B.i3 < 9;
       dynamic_simulation_B.i3++) {
    dynamic_simulation_B.R_g1[dynamic_simulation_B.i3] =
      -dynamic_simulation_B.R_g[dynamic_simulation_B.i3];
  }

  for (dynamic_simulation_B.i3 = 0; dynamic_simulation_B.i3 < 3;
       dynamic_simulation_B.i3++) {
    dynamic_simulation_B.Tinv_tmp = dynamic_simulation_B.i3 << 2;
    Tinv[dynamic_simulation_B.Tinv_tmp] = dynamic_simulation_B.R_g[3 *
      dynamic_simulation_B.i3];
    Tinv[dynamic_simulation_B.Tinv_tmp + 1] = dynamic_simulation_B.R_g[3 *
      dynamic_simulation_B.i3 + 1];
    Tinv[dynamic_simulation_B.Tinv_tmp + 2] = dynamic_simulation_B.R_g[3 *
      dynamic_simulation_B.i3 + 2];
    Tinv[dynamic_simulation_B.i3 + 12] =
      dynamic_simulation_B.R_g1[dynamic_simulation_B.i3 + 6] * T[14] +
      (dynamic_simulation_B.R_g1[dynamic_simulation_B.i3 + 3] * T[13] +
       dynamic_simulation_B.R_g1[dynamic_simulation_B.i3] * T[12]);
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
    for (dynamic_simulation_B.X_tmp_a = 0; dynamic_simulation_B.X_tmp_a < 3;
         dynamic_simulation_B.X_tmp_a++) {
      dynamic_simulation_B.X_tmp_e = dynamic_simulation_B.i1 + 3 *
        dynamic_simulation_B.X_tmp_a;
      dynamic_simulation_B.dv3[dynamic_simulation_B.X_tmp_e] = 0.0;
      dynamic_simulation_B.i2 = dynamic_simulation_B.X_tmp_a << 2;
      dynamic_simulation_B.dv3[dynamic_simulation_B.X_tmp_e] +=
        T[dynamic_simulation_B.i2] *
        dynamic_simulation_B.dv2[dynamic_simulation_B.i1];
      dynamic_simulation_B.dv3[dynamic_simulation_B.X_tmp_e] +=
        T[dynamic_simulation_B.i2 + 1] *
        dynamic_simulation_B.dv2[dynamic_simulation_B.i1 + 3];
      dynamic_simulation_B.dv3[dynamic_simulation_B.X_tmp_e] +=
        T[dynamic_simulation_B.i2 + 2] *
        dynamic_simulation_B.dv2[dynamic_simulation_B.i1 + 6];
      X[dynamic_simulation_B.X_tmp_a + 6 * dynamic_simulation_B.i1] = T
        [(dynamic_simulation_B.i1 << 2) + dynamic_simulation_B.X_tmp_a];
      X[dynamic_simulation_B.X_tmp_a + 6 * (dynamic_simulation_B.i1 + 3)] = 0.0;
    }
  }

  for (dynamic_simulation_B.i1 = 0; dynamic_simulation_B.i1 < 3;
       dynamic_simulation_B.i1++) {
    X[6 * dynamic_simulation_B.i1 + 3] = dynamic_simulation_B.dv3[3 *
      dynamic_simulation_B.i1];
    dynamic_simulation_B.X_tmp_a = dynamic_simulation_B.i1 << 2;
    dynamic_simulation_B.X_tmp_e = 6 * (dynamic_simulation_B.i1 + 3);
    X[dynamic_simulation_B.X_tmp_e + 3] = T[dynamic_simulation_B.X_tmp_a];
    X[6 * dynamic_simulation_B.i1 + 4] = dynamic_simulation_B.dv3[3 *
      dynamic_simulation_B.i1 + 1];
    X[dynamic_simulation_B.X_tmp_e + 4] = T[dynamic_simulation_B.X_tmp_a + 1];
    X[6 * dynamic_simulation_B.i1 + 5] = dynamic_simulation_B.dv3[3 *
      dynamic_simulation_B.i1 + 2];
    X[dynamic_simulation_B.X_tmp_e + 5] = T[dynamic_simulation_B.X_tmp_a + 2];
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
  for (dynamic_simulation_B.i_o2 = 0; dynamic_simulation_B.i_o2 < numDimensions;
       dynamic_simulation_B.i_o2++) {
    emxArray->size[dynamic_simulation_B.i_o2] = 0;
  }
}

static void dynami_emxEnsureCapacity_char_T(emxArray_char_T_dynamic_simul_T
  *emxArray, int32_T oldNumel)
{
  void *newData;
  if (oldNumel < 0) {
    oldNumel = 0;
  }

  dynamic_simulation_B.newNumel_a = 1;
  for (dynamic_simulation_B.i_i = 0; dynamic_simulation_B.i_i <
       emxArray->numDimensions; dynamic_simulation_B.i_i++) {
    dynamic_simulation_B.newNumel_a *= emxArray->size[dynamic_simulation_B.i_i];
  }

  if (dynamic_simulation_B.newNumel_a > emxArray->allocatedSize) {
    dynamic_simulation_B.i_i = emxArray->allocatedSize;
    if (dynamic_simulation_B.i_i < 16) {
      dynamic_simulation_B.i_i = 16;
    }

    while (dynamic_simulation_B.i_i < dynamic_simulation_B.newNumel_a) {
      if (dynamic_simulation_B.i_i > 1073741823) {
        dynamic_simulation_B.i_i = MAX_int32_T;
      } else {
        dynamic_simulation_B.i_i <<= 1;
      }
    }

    newData = calloc(static_cast<uint32_T>(dynamic_simulation_B.i_i), sizeof
                     (char_T));
    if (emxArray->data != NULL) {
      memcpy(newData, emxArray->data, sizeof(char_T) * oldNumel);
      if (emxArray->canFreeData) {
        free(emxArray->data);
      }
    }

    emxArray->data = (char_T *)newData;
    emxArray->allocatedSize = dynamic_simulation_B.i_i;
    emxArray->canFreeData = true;
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

static void RigidBodyTreeDynamics_massMatri(p_robotics_manip_internal_Rig_T
  *robot, const real_T q[6], emxArray_real_T_dynamic_simul_T *H,
  emxArray_real_T_dynamic_simul_T *lambda)
{
  emxArray_f_cell_wrap_dynamic__T *Ic;
  emxArray_f_cell_wrap_dynamic__T *X;
  emxArray_real_T_dynamic_simul_T *lambda_;
  emxArray_real_T_dynamic_simul_T *Si;
  emxArray_real_T_dynamic_simul_T *Fi;
  emxArray_real_T_dynamic_simul_T *Hji;
  emxArray_real_T_dynamic_simul_T *s;
  n_robotics_manip_internal_Rig_T *obj;
  emxArray_char_T_dynamic_simul_T *a;
  static const char_T tmp[5] = { 'f', 'i', 'x', 'e', 'd' };

  boolean_T exitg1;
  int32_T exitg2;
  dynamic_simulation_B.nb_e = robot->NumBodies;
  dynamic_simulation_B.vNum_b = robot->VelocityNumber;
  dynamic_simulation_B.f = H->size[0] * H->size[1];
  dynamic_simulation_B.b_i = static_cast<int32_T>(dynamic_simulation_B.vNum_b);
  H->size[0] = dynamic_simulation_B.b_i;
  H->size[1] = dynamic_simulation_B.b_i;
  dynami_emxEnsureCapacity_real_T(H, dynamic_simulation_B.f);
  dynamic_simulation_B.loop_ub = dynamic_simulation_B.b_i *
    dynamic_simulation_B.b_i - 1;
  for (dynamic_simulation_B.f = 0; dynamic_simulation_B.f <=
       dynamic_simulation_B.loop_ub; dynamic_simulation_B.f++) {
    H->data[dynamic_simulation_B.f] = 0.0;
  }

  dynamic_simulati_emxInit_real_T(&lambda_, 2);
  dynamic_simulation_B.f = lambda_->size[0] * lambda_->size[1];
  lambda_->size[0] = 1;
  dynamic_simulation_B.nm1d2 = static_cast<int32_T>(dynamic_simulation_B.nb_e);
  lambda_->size[1] = dynamic_simulation_B.nm1d2;
  dynami_emxEnsureCapacity_real_T(lambda_, dynamic_simulation_B.f);
  dynamic_simulation_B.idx = dynamic_simulation_B.nm1d2 - 1;
  for (dynamic_simulation_B.f = 0; dynamic_simulation_B.f <=
       dynamic_simulation_B.idx; dynamic_simulation_B.f++) {
    lambda_->data[dynamic_simulation_B.f] = 0.0;
  }

  dynamic_simulation_B.f = lambda->size[0] * lambda->size[1];
  lambda->size[0] = 1;
  lambda->size[1] = dynamic_simulation_B.b_i;
  dynami_emxEnsureCapacity_real_T(lambda, dynamic_simulation_B.f);
  dynamic_simulation_B.loop_ub = dynamic_simulation_B.b_i - 1;
  for (dynamic_simulation_B.f = 0; dynamic_simulation_B.f <=
       dynamic_simulation_B.loop_ub; dynamic_simulation_B.f++) {
    lambda->data[dynamic_simulation_B.f] = 0.0;
  }

  dynamic_sim_emxInit_f_cell_wrap(&Ic, 2);
  dynamic_sim_emxInit_f_cell_wrap(&X, 2);
  dynamic_simulation_B.f = Ic->size[0] * Ic->size[1];
  Ic->size[0] = 1;
  Ic->size[1] = dynamic_simulation_B.nm1d2;
  d_emxEnsureCapacity_f_cell_wrap(Ic, dynamic_simulation_B.f);
  dynamic_simulation_B.f = X->size[0] * X->size[1];
  X->size[0] = 1;
  X->size[1] = dynamic_simulation_B.nm1d2;
  d_emxEnsureCapacity_f_cell_wrap(X, dynamic_simulation_B.f);
  for (dynamic_simulation_B.b_i = 0; dynamic_simulation_B.b_i <=
       dynamic_simulation_B.idx; dynamic_simulation_B.b_i++) {
    for (dynamic_simulation_B.f = 0; dynamic_simulation_B.f < 36;
         dynamic_simulation_B.f++) {
      Ic->data[dynamic_simulation_B.b_i].f1[dynamic_simulation_B.f] =
        robot->Bodies[dynamic_simulation_B.b_i]->
        SpatialInertia[dynamic_simulation_B.f];
    }

    dynamic_simulation_B.vNum_b = robot->PositionDoFMap[dynamic_simulation_B.b_i];
    dynamic_simulation_B.p_idx_1 = robot->
      PositionDoFMap[dynamic_simulation_B.b_i + 8];
    if (dynamic_simulation_B.p_idx_1 < dynamic_simulation_B.vNum_b) {
      obj = robot->Bodies[dynamic_simulation_B.b_i];
      rigidBodyJoint_transformBodyToP(&obj->JointInternal,
        dynamic_simulation_B.T_m);
    } else {
      if (dynamic_simulation_B.vNum_b > dynamic_simulation_B.p_idx_1) {
        dynamic_simulation_B.nm1d2 = 0;
        dynamic_simulation_B.f = -1;
      } else {
        dynamic_simulation_B.nm1d2 = static_cast<int32_T>
          (dynamic_simulation_B.vNum_b) - 1;
        dynamic_simulation_B.f = static_cast<int32_T>
          (dynamic_simulation_B.p_idx_1) - 1;
      }

      obj = robot->Bodies[dynamic_simulation_B.b_i];
      dynamic_simulation_B.loop_ub = dynamic_simulation_B.f -
        dynamic_simulation_B.nm1d2;
      dynamic_simulation_B.q_size_h = dynamic_simulation_B.loop_ub + 1;
      for (dynamic_simulation_B.f = 0; dynamic_simulation_B.f <=
           dynamic_simulation_B.loop_ub; dynamic_simulation_B.f++) {
        dynamic_simulation_B.q_data_p[dynamic_simulation_B.f] =
          q[dynamic_simulation_B.nm1d2 + dynamic_simulation_B.f];
      }

      rigidBodyJoint_transformBodyT_k(&obj->JointInternal,
        dynamic_simulation_B.q_data_p, &dynamic_simulation_B.q_size_h,
        dynamic_simulation_B.T_m);
    }

    dynamic_simulation_tforminv(dynamic_simulation_B.T_m,
      dynamic_simulation_B.dv);
    dynamic_sim_tformToSpatialXform(dynamic_simulation_B.dv, X->
      data[dynamic_simulation_B.b_i].f1);
  }

  dynamic_simulation_B.nm1d2 = static_cast<int32_T>(((-1.0 -
    dynamic_simulation_B.nb_e) + 1.0) / -1.0) - 1;
  dynamic_simulati_emxInit_real_T(&Si, 2);
  dynamic_simulati_emxInit_real_T(&Fi, 2);
  dynamic_simulati_emxInit_real_T(&Hji, 2);
  dynamic_simulati_emxInit_char_T(&a, 2);
  for (dynamic_simulation_B.idx = 0; dynamic_simulation_B.idx <=
       dynamic_simulation_B.nm1d2; dynamic_simulation_B.idx++) {
    dynamic_simulation_B.n_m = static_cast<int32_T>(dynamic_simulation_B.nb_e +
      -static_cast<real_T>(dynamic_simulation_B.idx));
    dynamic_simulation_B.pid_tmp = dynamic_simulation_B.n_m - 1;
    dynamic_simulation_B.pid = robot->Bodies[dynamic_simulation_B.pid_tmp]
      ->ParentIndex;
    dynamic_simulation_B.vNum_b = robot->VelocityDoFMap[dynamic_simulation_B.n_m
      - 1];
    dynamic_simulation_B.p_idx_1 = robot->
      VelocityDoFMap[dynamic_simulation_B.n_m + 7];
    if (dynamic_simulation_B.pid > 0.0) {
      for (dynamic_simulation_B.f = 0; dynamic_simulation_B.f < 6;
           dynamic_simulation_B.f++) {
        for (dynamic_simulation_B.b_i = 0; dynamic_simulation_B.b_i < 6;
             dynamic_simulation_B.b_i++) {
          dynamic_simulation_B.X_tmp = dynamic_simulation_B.f + 6 *
            dynamic_simulation_B.b_i;
          dynamic_simulation_B.X[dynamic_simulation_B.X_tmp] = 0.0;
          for (dynamic_simulation_B.loop_ub = 0; dynamic_simulation_B.loop_ub <
               6; dynamic_simulation_B.loop_ub++) {
            dynamic_simulation_B.X[dynamic_simulation_B.X_tmp] += X->
              data[dynamic_simulation_B.pid_tmp].f1[6 * dynamic_simulation_B.f +
              dynamic_simulation_B.loop_ub] * Ic->
              data[dynamic_simulation_B.pid_tmp].f1[6 * dynamic_simulation_B.b_i
              + dynamic_simulation_B.loop_ub];
          }
        }
      }

      for (dynamic_simulation_B.f = 0; dynamic_simulation_B.f < 6;
           dynamic_simulation_B.f++) {
        for (dynamic_simulation_B.b_i = 0; dynamic_simulation_B.b_i < 6;
             dynamic_simulation_B.b_i++) {
          dynamic_simulation_B.b_idx_0_j = 0.0;
          for (dynamic_simulation_B.loop_ub = 0; dynamic_simulation_B.loop_ub <
               6; dynamic_simulation_B.loop_ub++) {
            dynamic_simulation_B.b_idx_0_j += dynamic_simulation_B.X[6 *
              dynamic_simulation_B.loop_ub + dynamic_simulation_B.f] * X->
              data[dynamic_simulation_B.pid_tmp].f1[6 * dynamic_simulation_B.b_i
              + dynamic_simulation_B.loop_ub];
          }

          dynamic_simulation_B.loop_ub = 6 * dynamic_simulation_B.b_i +
            dynamic_simulation_B.f;
          Ic->data[static_cast<int32_T>(dynamic_simulation_B.pid) - 1]
            .f1[dynamic_simulation_B.loop_ub] += dynamic_simulation_B.b_idx_0_j;
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
        dynamic_simulation_B.loop_ub = obj->JointInternal.Type->size[0] *
          obj->JointInternal.Type->size[1] - 1;
        for (dynamic_simulation_B.f = 0; dynamic_simulation_B.f <=
             dynamic_simulation_B.loop_ub; dynamic_simulation_B.f++) {
          a->data[dynamic_simulation_B.f] = obj->JointInternal.Type->
            data[dynamic_simulation_B.f];
        }

        for (dynamic_simulation_B.f = 0; dynamic_simulation_B.f < 5;
             dynamic_simulation_B.f++) {
          dynamic_simulation_B.b_o4[dynamic_simulation_B.f] =
            tmp[dynamic_simulation_B.f];
        }

        dynamic_simulation_B.b_bool_m = false;
        if (a->size[1] == 5) {
          dynamic_simulation_B.f = 1;
          do {
            exitg2 = 0;
            if (dynamic_simulation_B.f - 1 < 5) {
              dynamic_simulation_B.b_i = dynamic_simulation_B.f - 1;
              if (a->data[dynamic_simulation_B.b_i] !=
                  dynamic_simulation_B.b_o4[dynamic_simulation_B.b_i]) {
                exitg2 = 1;
              } else {
                dynamic_simulation_B.f++;
              }
            } else {
              dynamic_simulation_B.b_bool_m = true;
              exitg2 = 1;
            }
          } while (exitg2 == 0);
        }

        if (dynamic_simulation_B.b_bool_m) {
          lambda_->data[dynamic_simulation_B.pid_tmp] = robot->Bodies[
            static_cast<int32_T>(lambda_->data[dynamic_simulation_B.pid_tmp]) -
            1]->ParentIndex;
        } else {
          exitg1 = true;
        }
      }
    }

    dynamic_simulation_B.b_idx_0_j = robot->
      VelocityDoFMap[dynamic_simulation_B.n_m - 1];
    dynamic_simulation_B.b_idx_1_f = robot->
      VelocityDoFMap[dynamic_simulation_B.n_m + 7];
    if (dynamic_simulation_B.b_idx_0_j <= dynamic_simulation_B.b_idx_1_f) {
      obj = robot->Bodies[dynamic_simulation_B.pid_tmp];
      dynamic_simulation_B.f = Si->size[0] * Si->size[1];
      Si->size[0] = 6;
      Si->size[1] = obj->JointInternal.MotionSubspace->size[1];
      dynami_emxEnsureCapacity_real_T(Si, dynamic_simulation_B.f);
      dynamic_simulation_B.loop_ub = obj->JointInternal.MotionSubspace->size[0] *
        obj->JointInternal.MotionSubspace->size[1] - 1;
      for (dynamic_simulation_B.f = 0; dynamic_simulation_B.f <=
           dynamic_simulation_B.loop_ub; dynamic_simulation_B.f++) {
        Si->data[dynamic_simulation_B.f] = obj->
          JointInternal.MotionSubspace->data[dynamic_simulation_B.f];
      }

      dynamic_simulation_B.n_m = Si->size[1] - 1;
      dynamic_simulation_B.f = Fi->size[0] * Fi->size[1];
      Fi->size[0] = 6;
      Fi->size[1] = Si->size[1];
      dynami_emxEnsureCapacity_real_T(Fi, dynamic_simulation_B.f);
      for (dynamic_simulation_B.loop_ub = 0; dynamic_simulation_B.loop_ub <=
           dynamic_simulation_B.n_m; dynamic_simulation_B.loop_ub++) {
        dynamic_simulation_B.coffset_tmp = dynamic_simulation_B.loop_ub * 6 - 1;
        for (dynamic_simulation_B.b_i = 0; dynamic_simulation_B.b_i < 6;
             dynamic_simulation_B.b_i++) {
          dynamic_simulation_B.s = 0.0;
          for (dynamic_simulation_B.f = 0; dynamic_simulation_B.f < 6;
               dynamic_simulation_B.f++) {
            dynamic_simulation_B.s += Ic->data[dynamic_simulation_B.pid_tmp]
              .f1[dynamic_simulation_B.f * 6 + dynamic_simulation_B.b_i] *
              Si->data[(dynamic_simulation_B.coffset_tmp +
                        dynamic_simulation_B.f) + 1];
          }

          Fi->data[(dynamic_simulation_B.coffset_tmp + dynamic_simulation_B.b_i)
            + 1] = dynamic_simulation_B.s;
        }
      }

      if (dynamic_simulation_B.vNum_b > dynamic_simulation_B.p_idx_1) {
        dynamic_simulation_B.coffset_tmp = 0;
        dynamic_simulation_B.cb = 0;
      } else {
        dynamic_simulation_B.coffset_tmp = static_cast<int32_T>
          (dynamic_simulation_B.vNum_b) - 1;
        dynamic_simulation_B.cb = static_cast<int32_T>
          (dynamic_simulation_B.vNum_b) - 1;
      }

      dynamic_simulation_B.m_m = Si->size[1];
      dynamic_simulation_B.n_m = Fi->size[1] - 1;
      dynamic_simulation_B.f = Hji->size[0] * Hji->size[1];
      Hji->size[0] = Si->size[1];
      Hji->size[1] = Fi->size[1];
      dynami_emxEnsureCapacity_real_T(Hji, dynamic_simulation_B.f);
      for (dynamic_simulation_B.loop_ub = 0; dynamic_simulation_B.loop_ub <=
           dynamic_simulation_B.n_m; dynamic_simulation_B.loop_ub++) {
        dynamic_simulation_B.coffset = dynamic_simulation_B.loop_ub *
          dynamic_simulation_B.m_m - 1;
        dynamic_simulation_B.boffset = dynamic_simulation_B.loop_ub * 6 - 1;
        for (dynamic_simulation_B.b_i = 0; dynamic_simulation_B.b_i <
             dynamic_simulation_B.m_m; dynamic_simulation_B.b_i++) {
          dynamic_simulation_B.aoffset_j = dynamic_simulation_B.b_i * 6 - 1;
          dynamic_simulation_B.s = 0.0;
          for (dynamic_simulation_B.f = 0; dynamic_simulation_B.f < 6;
               dynamic_simulation_B.f++) {
            dynamic_simulation_B.X_tmp = dynamic_simulation_B.f + 1;
            dynamic_simulation_B.s += Si->data[dynamic_simulation_B.aoffset_j +
              dynamic_simulation_B.X_tmp] * Fi->
              data[dynamic_simulation_B.boffset + dynamic_simulation_B.X_tmp];
          }

          Hji->data[(dynamic_simulation_B.coffset + dynamic_simulation_B.b_i) +
            1] = dynamic_simulation_B.s;
        }
      }

      dynamic_simulation_B.loop_ub = Hji->size[1];
      for (dynamic_simulation_B.f = 0; dynamic_simulation_B.f <
           dynamic_simulation_B.loop_ub; dynamic_simulation_B.f++) {
        dynamic_simulation_B.n_m = Hji->size[0];
        for (dynamic_simulation_B.b_i = 0; dynamic_simulation_B.b_i <
             dynamic_simulation_B.n_m; dynamic_simulation_B.b_i++) {
          H->data[(dynamic_simulation_B.coffset_tmp + dynamic_simulation_B.b_i)
            + H->size[0] * (dynamic_simulation_B.cb + dynamic_simulation_B.f)] =
            Hji->data[Hji->size[0] * dynamic_simulation_B.f +
            dynamic_simulation_B.b_i];
        }
      }

      dynamic_simulation_B.n_m = Fi->size[1];
      dynamic_simulation_B.f = Si->size[0] * Si->size[1];
      Si->size[0] = 6;
      Si->size[1] = Fi->size[1];
      dynami_emxEnsureCapacity_real_T(Si, dynamic_simulation_B.f);
      dynamic_simulation_B.loop_ub = Fi->size[0] * Fi->size[1] - 1;
      for (dynamic_simulation_B.f = 0; dynamic_simulation_B.f <=
           dynamic_simulation_B.loop_ub; dynamic_simulation_B.f++) {
        Si->data[dynamic_simulation_B.f] = Fi->data[dynamic_simulation_B.f];
      }

      dynamic_simulation_B.f = Fi->size[0] * Fi->size[1];
      Fi->size[0] = 6;
      Fi->size[1] = dynamic_simulation_B.n_m;
      dynami_emxEnsureCapacity_real_T(Fi, dynamic_simulation_B.f);
      for (dynamic_simulation_B.loop_ub = 0; dynamic_simulation_B.loop_ub <
           dynamic_simulation_B.n_m; dynamic_simulation_B.loop_ub++) {
        dynamic_simulation_B.coffset_tmp = dynamic_simulation_B.loop_ub * 6 - 1;
        for (dynamic_simulation_B.b_i = 0; dynamic_simulation_B.b_i < 6;
             dynamic_simulation_B.b_i++) {
          dynamic_simulation_B.aoffset_j = dynamic_simulation_B.b_i * 6 - 1;
          dynamic_simulation_B.s = 0.0;
          for (dynamic_simulation_B.f = 0; dynamic_simulation_B.f < 6;
               dynamic_simulation_B.f++) {
            dynamic_simulation_B.X_tmp = dynamic_simulation_B.f + 1;
            dynamic_simulation_B.s += X->data[dynamic_simulation_B.pid_tmp]
              .f1[dynamic_simulation_B.aoffset_j + dynamic_simulation_B.X_tmp] *
              Si->data[dynamic_simulation_B.coffset_tmp +
              dynamic_simulation_B.X_tmp];
          }

          Fi->data[(dynamic_simulation_B.coffset_tmp + dynamic_simulation_B.b_i)
            + 1] = dynamic_simulation_B.s;
        }
      }

      while (dynamic_simulation_B.pid > 0.0) {
        dynamic_simulation_B.b_i = static_cast<int32_T>(dynamic_simulation_B.pid);
        dynamic_simulation_B.pid_tmp = dynamic_simulation_B.b_i - 1;
        obj = robot->Bodies[dynamic_simulation_B.pid_tmp];
        dynamic_simulation_B.f = Si->size[0] * Si->size[1];
        Si->size[0] = 6;
        Si->size[1] = obj->JointInternal.MotionSubspace->size[1];
        dynami_emxEnsureCapacity_real_T(Si, dynamic_simulation_B.f);
        dynamic_simulation_B.loop_ub = obj->JointInternal.MotionSubspace->size[0]
          * obj->JointInternal.MotionSubspace->size[1] - 1;
        for (dynamic_simulation_B.f = 0; dynamic_simulation_B.f <=
             dynamic_simulation_B.loop_ub; dynamic_simulation_B.f++) {
          Si->data[dynamic_simulation_B.f] = obj->
            JointInternal.MotionSubspace->data[dynamic_simulation_B.f];
        }

        dynamic_simulation_B.b_idx_0_j = robot->
          VelocityDoFMap[dynamic_simulation_B.b_i - 1];
        dynamic_simulation_B.b_idx_1_f = robot->
          VelocityDoFMap[dynamic_simulation_B.b_i + 7];
        if (dynamic_simulation_B.b_idx_0_j <= dynamic_simulation_B.b_idx_1_f) {
          dynamic_simulation_B.m_m = Si->size[1];
          dynamic_simulation_B.n_m = Fi->size[1] - 1;
          dynamic_simulation_B.f = Hji->size[0] * Hji->size[1];
          Hji->size[0] = Si->size[1];
          Hji->size[1] = Fi->size[1];
          dynami_emxEnsureCapacity_real_T(Hji, dynamic_simulation_B.f);
          for (dynamic_simulation_B.loop_ub = 0; dynamic_simulation_B.loop_ub <=
               dynamic_simulation_B.n_m; dynamic_simulation_B.loop_ub++) {
            dynamic_simulation_B.coffset = dynamic_simulation_B.loop_ub *
              dynamic_simulation_B.m_m - 1;
            dynamic_simulation_B.boffset = dynamic_simulation_B.loop_ub * 6 - 1;
            for (dynamic_simulation_B.b_i = 0; dynamic_simulation_B.b_i <
                 dynamic_simulation_B.m_m; dynamic_simulation_B.b_i++) {
              dynamic_simulation_B.aoffset_j = dynamic_simulation_B.b_i * 6 - 1;
              dynamic_simulation_B.s = 0.0;
              for (dynamic_simulation_B.f = 0; dynamic_simulation_B.f < 6;
                   dynamic_simulation_B.f++) {
                dynamic_simulation_B.X_tmp = dynamic_simulation_B.f + 1;
                dynamic_simulation_B.s += Si->
                  data[dynamic_simulation_B.aoffset_j +
                  dynamic_simulation_B.X_tmp] * Fi->
                  data[dynamic_simulation_B.boffset + dynamic_simulation_B.X_tmp];
              }

              Hji->data[(dynamic_simulation_B.coffset + dynamic_simulation_B.b_i)
                + 1] = dynamic_simulation_B.s;
            }
          }

          if (dynamic_simulation_B.b_idx_0_j > dynamic_simulation_B.b_idx_1_f) {
            dynamic_simulation_B.X_tmp = 0;
          } else {
            dynamic_simulation_B.X_tmp = static_cast<int32_T>
              (dynamic_simulation_B.b_idx_0_j) - 1;
          }

          if (dynamic_simulation_B.vNum_b > dynamic_simulation_B.p_idx_1) {
            dynamic_simulation_B.coffset_tmp = 0;
          } else {
            dynamic_simulation_B.coffset_tmp = static_cast<int32_T>
              (dynamic_simulation_B.vNum_b) - 1;
          }

          dynamic_simulation_B.loop_ub = Hji->size[1];
          for (dynamic_simulation_B.f = 0; dynamic_simulation_B.f <
               dynamic_simulation_B.loop_ub; dynamic_simulation_B.f++) {
            dynamic_simulation_B.n_m = Hji->size[0];
            for (dynamic_simulation_B.b_i = 0; dynamic_simulation_B.b_i <
                 dynamic_simulation_B.n_m; dynamic_simulation_B.b_i++) {
              H->data[(dynamic_simulation_B.X_tmp + dynamic_simulation_B.b_i) +
                H->size[0] * (dynamic_simulation_B.coffset_tmp +
                              dynamic_simulation_B.f)] = Hji->data[Hji->size[0] *
                dynamic_simulation_B.f + dynamic_simulation_B.b_i];
            }
          }

          if (dynamic_simulation_B.vNum_b > dynamic_simulation_B.p_idx_1) {
            dynamic_simulation_B.X_tmp = 0;
          } else {
            dynamic_simulation_B.X_tmp = static_cast<int32_T>
              (dynamic_simulation_B.vNum_b) - 1;
          }

          if (dynamic_simulation_B.b_idx_0_j > dynamic_simulation_B.b_idx_1_f) {
            dynamic_simulation_B.coffset_tmp = 0;
          } else {
            dynamic_simulation_B.coffset_tmp = static_cast<int32_T>
              (dynamic_simulation_B.b_idx_0_j) - 1;
          }

          dynamic_simulation_B.loop_ub = Hji->size[0];
          for (dynamic_simulation_B.f = 0; dynamic_simulation_B.f <
               dynamic_simulation_B.loop_ub; dynamic_simulation_B.f++) {
            dynamic_simulation_B.n_m = Hji->size[1];
            for (dynamic_simulation_B.b_i = 0; dynamic_simulation_B.b_i <
                 dynamic_simulation_B.n_m; dynamic_simulation_B.b_i++) {
              H->data[(dynamic_simulation_B.X_tmp + dynamic_simulation_B.b_i) +
                H->size[0] * (dynamic_simulation_B.coffset_tmp +
                              dynamic_simulation_B.f)] = Hji->data[Hji->size[0] *
                dynamic_simulation_B.b_i + dynamic_simulation_B.f];
            }
          }
        }

        dynamic_simulation_B.n_m = Fi->size[1];
        dynamic_simulation_B.f = Si->size[0] * Si->size[1];
        Si->size[0] = 6;
        Si->size[1] = Fi->size[1];
        dynami_emxEnsureCapacity_real_T(Si, dynamic_simulation_B.f);
        dynamic_simulation_B.loop_ub = Fi->size[0] * Fi->size[1] - 1;
        for (dynamic_simulation_B.f = 0; dynamic_simulation_B.f <=
             dynamic_simulation_B.loop_ub; dynamic_simulation_B.f++) {
          Si->data[dynamic_simulation_B.f] = Fi->data[dynamic_simulation_B.f];
        }

        dynamic_simulation_B.f = Fi->size[0] * Fi->size[1];
        Fi->size[0] = 6;
        Fi->size[1] = dynamic_simulation_B.n_m;
        dynami_emxEnsureCapacity_real_T(Fi, dynamic_simulation_B.f);
        for (dynamic_simulation_B.loop_ub = 0; dynamic_simulation_B.loop_ub <
             dynamic_simulation_B.n_m; dynamic_simulation_B.loop_ub++) {
          dynamic_simulation_B.coffset_tmp = dynamic_simulation_B.loop_ub * 6 -
            1;
          for (dynamic_simulation_B.b_i = 0; dynamic_simulation_B.b_i < 6;
               dynamic_simulation_B.b_i++) {
            dynamic_simulation_B.aoffset_j = dynamic_simulation_B.b_i * 6 - 1;
            dynamic_simulation_B.s = 0.0;
            for (dynamic_simulation_B.f = 0; dynamic_simulation_B.f < 6;
                 dynamic_simulation_B.f++) {
              dynamic_simulation_B.X_tmp = dynamic_simulation_B.f + 1;
              dynamic_simulation_B.s += X->data[dynamic_simulation_B.pid_tmp]
                .f1[dynamic_simulation_B.aoffset_j + dynamic_simulation_B.X_tmp]
                * Si->data[dynamic_simulation_B.coffset_tmp +
                dynamic_simulation_B.X_tmp];
            }

            Fi->data[(dynamic_simulation_B.coffset_tmp +
                      dynamic_simulation_B.b_i) + 1] = dynamic_simulation_B.s;
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
  dynamic_sim_emxFree_f_cell_wrap(&X);
  dynamic_sim_emxFree_f_cell_wrap(&Ic);
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

  dynamic_simulation_B.b_i = dynamic_simulation_B.idx - 1;
  dynamic_simulati_emxInit_real_T(&s, 2);
  for (dynamic_simulation_B.idx = 0; dynamic_simulation_B.idx <=
       dynamic_simulation_B.b_i; dynamic_simulation_B.idx++) {
    dynamic_simulation_B.vNum_b = robot->
      VelocityDoFMap[dynamic_simulation_B.nonFixedIndices_data[dynamic_simulation_B.idx]
      - 1];
    dynamic_simulation_B.p_idx_1 = robot->
      VelocityDoFMap[dynamic_simulation_B.nonFixedIndices_data[dynamic_simulation_B.idx]
      + 7];
    if (rtIsNaN(dynamic_simulation_B.vNum_b) || rtIsNaN
        (dynamic_simulation_B.p_idx_1)) {
      dynamic_simulation_B.f = s->size[0] * s->size[1];
      s->size[0] = 1;
      s->size[1] = 1;
      dynami_emxEnsureCapacity_real_T(s, dynamic_simulation_B.f);
      s->data[0] = (rtNaN);
    } else if (dynamic_simulation_B.p_idx_1 < dynamic_simulation_B.vNum_b) {
      s->size[0] = 1;
      s->size[1] = 0;
    } else if ((rtIsInf(dynamic_simulation_B.vNum_b) || rtIsInf
                (dynamic_simulation_B.p_idx_1)) && (dynamic_simulation_B.vNum_b ==
                dynamic_simulation_B.p_idx_1)) {
      dynamic_simulation_B.f = s->size[0] * s->size[1];
      s->size[0] = 1;
      s->size[1] = 1;
      dynami_emxEnsureCapacity_real_T(s, dynamic_simulation_B.f);
      s->data[0] = (rtNaN);
    } else if (floor(dynamic_simulation_B.vNum_b) == dynamic_simulation_B.vNum_b)
    {
      dynamic_simulation_B.f = s->size[0] * s->size[1];
      s->size[0] = 1;
      dynamic_simulation_B.loop_ub = static_cast<int32_T>(floor
        (dynamic_simulation_B.p_idx_1 - dynamic_simulation_B.vNum_b));
      s->size[1] = dynamic_simulation_B.loop_ub + 1;
      dynami_emxEnsureCapacity_real_T(s, dynamic_simulation_B.f);
      for (dynamic_simulation_B.f = 0; dynamic_simulation_B.f <=
           dynamic_simulation_B.loop_ub; dynamic_simulation_B.f++) {
        s->data[dynamic_simulation_B.f] = dynamic_simulation_B.vNum_b +
          static_cast<real_T>(dynamic_simulation_B.f);
      }
    } else {
      dynamic_simulation_B.nb_e = floor((dynamic_simulation_B.p_idx_1 -
        dynamic_simulation_B.vNum_b) + 0.5);
      dynamic_simulation_B.pid = dynamic_simulation_B.vNum_b +
        dynamic_simulation_B.nb_e;
      dynamic_simulation_B.b_idx_0_j = dynamic_simulation_B.pid -
        dynamic_simulation_B.p_idx_1;
      dynamic_simulation_B.b_idx_1_f = fabs(dynamic_simulation_B.vNum_b);
      dynamic_simulation_B.s = fabs(dynamic_simulation_B.p_idx_1);
      if ((dynamic_simulation_B.b_idx_1_f > dynamic_simulation_B.s) || rtIsNaN
          (dynamic_simulation_B.s)) {
        dynamic_simulation_B.s = dynamic_simulation_B.b_idx_1_f;
      }

      if (fabs(dynamic_simulation_B.b_idx_0_j) < 4.4408920985006262E-16 *
          dynamic_simulation_B.s) {
        dynamic_simulation_B.nb_e++;
        dynamic_simulation_B.pid = dynamic_simulation_B.p_idx_1;
      } else if (dynamic_simulation_B.b_idx_0_j > 0.0) {
        dynamic_simulation_B.pid = (dynamic_simulation_B.nb_e - 1.0) +
          dynamic_simulation_B.vNum_b;
      } else {
        dynamic_simulation_B.nb_e++;
      }

      if (dynamic_simulation_B.nb_e >= 0.0) {
        dynamic_simulation_B.f = static_cast<int32_T>(dynamic_simulation_B.nb_e);
      } else {
        dynamic_simulation_B.f = 0;
      }

      dynamic_simulation_B.n_m = dynamic_simulation_B.f - 1;
      dynamic_simulation_B.f = s->size[0] * s->size[1];
      s->size[0] = 1;
      s->size[1] = dynamic_simulation_B.n_m + 1;
      dynami_emxEnsureCapacity_real_T(s, dynamic_simulation_B.f);
      if (dynamic_simulation_B.n_m + 1 > 0) {
        s->data[0] = dynamic_simulation_B.vNum_b;
        if (dynamic_simulation_B.n_m + 1 > 1) {
          s->data[dynamic_simulation_B.n_m] = dynamic_simulation_B.pid;
          dynamic_simulation_B.nm1d2 = ((dynamic_simulation_B.n_m < 0) +
            dynamic_simulation_B.n_m) >> 1;
          dynamic_simulation_B.loop_ub = dynamic_simulation_B.nm1d2 - 2;
          for (dynamic_simulation_B.f = 0; dynamic_simulation_B.f <=
               dynamic_simulation_B.loop_ub; dynamic_simulation_B.f++) {
            dynamic_simulation_B.X_tmp = dynamic_simulation_B.f + 1;
            s->data[dynamic_simulation_B.X_tmp] = dynamic_simulation_B.vNum_b +
              static_cast<real_T>(dynamic_simulation_B.X_tmp);
            s->data[dynamic_simulation_B.n_m - dynamic_simulation_B.X_tmp] =
              dynamic_simulation_B.pid - static_cast<real_T>
              (dynamic_simulation_B.X_tmp);
          }

          if (dynamic_simulation_B.nm1d2 << 1 == dynamic_simulation_B.n_m) {
            s->data[dynamic_simulation_B.nm1d2] = (dynamic_simulation_B.vNum_b +
              dynamic_simulation_B.pid) / 2.0;
          } else {
            s->data[dynamic_simulation_B.nm1d2] = dynamic_simulation_B.vNum_b +
              static_cast<real_T>(dynamic_simulation_B.nm1d2);
            s->data[dynamic_simulation_B.nm1d2 + 1] = dynamic_simulation_B.pid -
              static_cast<real_T>(dynamic_simulation_B.nm1d2);
          }
        }
      }
    }

    if (dynamic_simulation_B.vNum_b > dynamic_simulation_B.p_idx_1) {
      dynamic_simulation_B.nm1d2 = 0;
    } else {
      dynamic_simulation_B.nm1d2 = static_cast<int32_T>
        (dynamic_simulation_B.vNum_b) - 1;
    }

    dynamic_simulation_B.loop_ub = s->size[1];
    for (dynamic_simulation_B.f = 0; dynamic_simulation_B.f <
         dynamic_simulation_B.loop_ub; dynamic_simulation_B.f++) {
      lambda->data[dynamic_simulation_B.nm1d2 + dynamic_simulation_B.f] =
        s->data[dynamic_simulation_B.f] - 1.0;
    }

    if (lambda_->
        data[dynamic_simulation_B.nonFixedIndices_data[dynamic_simulation_B.idx]
        - 1] == 0.0) {
      lambda->data[static_cast<int32_T>(dynamic_simulation_B.vNum_b) - 1] = 0.0;
    } else {
      dynamic_simulation_B.f = static_cast<int32_T>(lambda_->
        data[dynamic_simulation_B.nonFixedIndices_data[dynamic_simulation_B.idx]
        - 1]);
      dynamic_simulation_B.b_idx_1_f = robot->
        VelocityDoFMap[dynamic_simulation_B.f + 7];
      lambda->data[static_cast<int32_T>(dynamic_simulation_B.vNum_b) - 1] =
        dynamic_simulation_B.b_idx_1_f;
    }
  }

  dynamic_simulati_emxFree_real_T(&s);
  dynamic_simulati_emxFree_real_T(&lambda_);
}

static void RigidBodyTreeDynamics_inverseDy(p_robotics_manip_internal_Rig_T
  *robot, const real_T q[6], const real_T qdot[6], const real_T fext[48], real_T
  tau[6])
{
  emxArray_f_cell_wrap_dynamic__T *X;
  emxArray_f_cell_wrap_dynamic__T *Xtree;
  emxArray_real_T_dynamic_simul_T *vJ;
  emxArray_real_T_dynamic_simul_T *vB;
  emxArray_real_T_dynamic_simul_T *aB;
  emxArray_real_T_dynamic_simul_T *f;
  emxArray_real_T_dynamic_simul_T *S;
  emxArray_real_T_dynamic_simul_T *taui;
  n_robotics_manip_internal_Rig_T *obj;
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
  dynamic_simulation_B.i_n = vJ->size[0] * vJ->size[1];
  vJ->size[0] = 6;
  dynamic_simulation_B.unnamed_idx_1 = static_cast<int32_T>
    (dynamic_simulation_B.nb);
  vJ->size[1] = dynamic_simulation_B.unnamed_idx_1;
  dynami_emxEnsureCapacity_real_T(vJ, dynamic_simulation_B.i_n);
  dynamic_simulation_B.loop_ub_tmp = 6 * dynamic_simulation_B.unnamed_idx_1 - 1;
  for (dynamic_simulation_B.i_n = 0; dynamic_simulation_B.i_n <=
       dynamic_simulation_B.loop_ub_tmp; dynamic_simulation_B.i_n++) {
    vJ->data[dynamic_simulation_B.i_n] = 0.0;
  }

  dynamic_simulati_emxInit_real_T(&vB, 2);
  dynamic_simulation_B.i_n = vB->size[0] * vB->size[1];
  vB->size[0] = 6;
  vB->size[1] = dynamic_simulation_B.unnamed_idx_1;
  dynami_emxEnsureCapacity_real_T(vB, dynamic_simulation_B.i_n);
  for (dynamic_simulation_B.i_n = 0; dynamic_simulation_B.i_n <=
       dynamic_simulation_B.loop_ub_tmp; dynamic_simulation_B.i_n++) {
    vB->data[dynamic_simulation_B.i_n] = 0.0;
  }

  dynamic_simulati_emxInit_real_T(&aB, 2);
  dynamic_simulation_B.i_n = aB->size[0] * aB->size[1];
  aB->size[0] = 6;
  aB->size[1] = dynamic_simulation_B.unnamed_idx_1;
  dynami_emxEnsureCapacity_real_T(aB, dynamic_simulation_B.i_n);
  for (dynamic_simulation_B.i_n = 0; dynamic_simulation_B.i_n <=
       dynamic_simulation_B.loop_ub_tmp; dynamic_simulation_B.i_n++) {
    aB->data[dynamic_simulation_B.i_n] = 0.0;
  }

  for (dynamic_simulation_B.i_n = 0; dynamic_simulation_B.i_n < 6;
       dynamic_simulation_B.i_n++) {
    tau[dynamic_simulation_B.i_n] = 0.0;
  }

  dynamic_sim_emxInit_f_cell_wrap(&X, 2);
  dynamic_sim_emxInit_f_cell_wrap(&Xtree, 2);
  dynamic_simulation_B.loop_ub_tmp = dynamic_simulation_B.unnamed_idx_1 - 1;
  dynamic_simulation_B.i_n = Xtree->size[0] * Xtree->size[1];
  Xtree->size[0] = 1;
  Xtree->size[1] = dynamic_simulation_B.unnamed_idx_1;
  d_emxEnsureCapacity_f_cell_wrap(Xtree, dynamic_simulation_B.i_n);
  dynamic_simulation_B.i_n = X->size[0] * X->size[1];
  X->size[0] = 1;
  X->size[1] = dynamic_simulation_B.unnamed_idx_1;
  d_emxEnsureCapacity_f_cell_wrap(X, dynamic_simulation_B.i_n);
  for (dynamic_simulation_B.b_k = 0; dynamic_simulation_B.b_k <=
       dynamic_simulation_B.loop_ub_tmp; dynamic_simulation_B.b_k++) {
    for (dynamic_simulation_B.i_n = 0; dynamic_simulation_B.i_n < 36;
         dynamic_simulation_B.i_n++) {
      Xtree->data[dynamic_simulation_B.b_k].f1[dynamic_simulation_B.i_n] = 0.0;
    }

    for (dynamic_simulation_B.i_n = 0; dynamic_simulation_B.i_n < 6;
         dynamic_simulation_B.i_n++) {
      Xtree->data[dynamic_simulation_B.b_k].f1[dynamic_simulation_B.i_n + 6 *
        dynamic_simulation_B.i_n] = 1.0;
    }

    for (dynamic_simulation_B.i_n = 0; dynamic_simulation_B.i_n < 36;
         dynamic_simulation_B.i_n++) {
      X->data[dynamic_simulation_B.b_k].f1[dynamic_simulation_B.i_n] = 0.0;
    }

    for (dynamic_simulation_B.i_n = 0; dynamic_simulation_B.i_n < 6;
         dynamic_simulation_B.i_n++) {
      X->data[dynamic_simulation_B.b_k].f1[dynamic_simulation_B.i_n + 6 *
        dynamic_simulation_B.i_n] = 1.0;
    }
  }

  dynamic_simulati_emxInit_real_T(&f, 2);
  dynamic_simulation_B.i_n = f->size[0] * f->size[1];
  f->size[0] = 6;
  f->size[1] = dynamic_simulation_B.unnamed_idx_1;
  dynami_emxEnsureCapacity_real_T(f, dynamic_simulation_B.i_n);
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
    dynamic_simulation_B.i_n = S->size[0] * S->size[1];
    S->size[0] = 6;
    S->size[1] = obj->JointInternal.MotionSubspace->size[1];
    dynami_emxEnsureCapacity_real_T(S, dynamic_simulation_B.i_n);
    dynamic_simulation_B.b_k = obj->JointInternal.MotionSubspace->size[0] *
      obj->JointInternal.MotionSubspace->size[1] - 1;
    for (dynamic_simulation_B.i_n = 0; dynamic_simulation_B.i_n <=
         dynamic_simulation_B.b_k; dynamic_simulation_B.i_n++) {
      S->data[dynamic_simulation_B.i_n] = obj->
        JointInternal.MotionSubspace->data[dynamic_simulation_B.i_n];
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
        dynamic_simulation_B.T);
      for (dynamic_simulation_B.i_n = 0; dynamic_simulation_B.i_n < 6;
           dynamic_simulation_B.i_n++) {
        vJ->data[dynamic_simulation_B.i_n + 6 *
          dynamic_simulation_B.unnamed_idx_1] = 0.0;
      }
    } else {
      if (dynamic_simulation_B.a_idx_0 > dynamic_simulation_B.a_idx_1) {
        dynamic_simulation_B.b_k = 0;
        dynamic_simulation_B.i_n = -1;
      } else {
        dynamic_simulation_B.b_k = static_cast<int32_T>
          (dynamic_simulation_B.a_idx_0) - 1;
        dynamic_simulation_B.i_n = static_cast<int32_T>
          (dynamic_simulation_B.a_idx_1) - 1;
      }

      if (dynamic_simulation_B.b_idx_0 > dynamic_simulation_B.b_idx_1) {
        dynamic_simulation_B.p = -1;
      } else {
        dynamic_simulation_B.p = static_cast<int32_T>
          (dynamic_simulation_B.b_idx_0) - 2;
      }

      obj = robot->Bodies[dynamic_simulation_B.unnamed_idx_1];
      dynamic_simulation_B.q_size_tmp = dynamic_simulation_B.i_n -
        dynamic_simulation_B.b_k;
      dynamic_simulation_B.q_size = dynamic_simulation_B.q_size_tmp + 1;
      for (dynamic_simulation_B.i_n = 0; dynamic_simulation_B.i_n <=
           dynamic_simulation_B.q_size_tmp; dynamic_simulation_B.i_n++) {
        dynamic_simulation_B.q_data[dynamic_simulation_B.i_n] =
          q[dynamic_simulation_B.b_k + dynamic_simulation_B.i_n];
      }

      rigidBodyJoint_transformBodyT_k(&obj->JointInternal,
        dynamic_simulation_B.q_data, &dynamic_simulation_B.q_size,
        dynamic_simulation_B.T);
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
          dynamic_simulation_B.i_n = 6 * dynamic_simulation_B.unnamed_idx_1 +
            dynamic_simulation_B.q_size_tmp;
          vJ->data[dynamic_simulation_B.i_n] += S->data
            [(dynamic_simulation_B.aoffset + dynamic_simulation_B.q_size_tmp) +
            1] * qdot[(dynamic_simulation_B.p + dynamic_simulation_B.b_k) + 1];
        }
      }
    }

    for (dynamic_simulation_B.i_n = 0; dynamic_simulation_B.i_n < 3;
         dynamic_simulation_B.i_n++) {
      dynamic_simulation_B.R_c[3 * dynamic_simulation_B.i_n] =
        dynamic_simulation_B.T[dynamic_simulation_B.i_n];
      dynamic_simulation_B.R_c[3 * dynamic_simulation_B.i_n + 1] =
        dynamic_simulation_B.T[dynamic_simulation_B.i_n + 4];
      dynamic_simulation_B.R_c[3 * dynamic_simulation_B.i_n + 2] =
        dynamic_simulation_B.T[dynamic_simulation_B.i_n + 8];
    }

    for (dynamic_simulation_B.i_n = 0; dynamic_simulation_B.i_n < 9;
         dynamic_simulation_B.i_n++) {
      dynamic_simulation_B.R_b[dynamic_simulation_B.i_n] =
        -dynamic_simulation_B.R_c[dynamic_simulation_B.i_n];
    }

    for (dynamic_simulation_B.i_n = 0; dynamic_simulation_B.i_n < 3;
         dynamic_simulation_B.i_n++) {
      dynamic_simulation_B.b_k = dynamic_simulation_B.i_n << 2;
      dynamic_simulation_B.Tinv[dynamic_simulation_B.b_k] =
        dynamic_simulation_B.R_c[3 * dynamic_simulation_B.i_n];
      dynamic_simulation_B.Tinv[dynamic_simulation_B.b_k + 1] =
        dynamic_simulation_B.R_c[3 * dynamic_simulation_B.i_n + 1];
      dynamic_simulation_B.Tinv[dynamic_simulation_B.b_k + 2] =
        dynamic_simulation_B.R_c[3 * dynamic_simulation_B.i_n + 2];
      dynamic_simulation_B.Tinv[dynamic_simulation_B.i_n + 12] =
        dynamic_simulation_B.R_b[dynamic_simulation_B.i_n + 6] *
        dynamic_simulation_B.T[14] +
        (dynamic_simulation_B.R_b[dynamic_simulation_B.i_n + 3] *
         dynamic_simulation_B.T[13] +
         dynamic_simulation_B.R_b[dynamic_simulation_B.i_n] *
         dynamic_simulation_B.T[12]);
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
    for (dynamic_simulation_B.i_n = 0; dynamic_simulation_B.i_n < 3;
         dynamic_simulation_B.i_n++) {
      for (dynamic_simulation_B.b_k = 0; dynamic_simulation_B.b_k < 3;
           dynamic_simulation_B.b_k++) {
        dynamic_simulation_B.q_size_tmp = dynamic_simulation_B.i_n + 3 *
          dynamic_simulation_B.b_k;
        dynamic_simulation_B.R_c[dynamic_simulation_B.q_size_tmp] = 0.0;
        dynamic_simulation_B.p = dynamic_simulation_B.b_k << 2;
        dynamic_simulation_B.R_c[dynamic_simulation_B.q_size_tmp] +=
          dynamic_simulation_B.Tinv[dynamic_simulation_B.p] *
          dynamic_simulation_B.dv1[dynamic_simulation_B.i_n];
        dynamic_simulation_B.R_c[dynamic_simulation_B.q_size_tmp] +=
          dynamic_simulation_B.Tinv[dynamic_simulation_B.p + 1] *
          dynamic_simulation_B.dv1[dynamic_simulation_B.i_n + 3];
        dynamic_simulation_B.R_c[dynamic_simulation_B.q_size_tmp] +=
          dynamic_simulation_B.Tinv[dynamic_simulation_B.p + 2] *
          dynamic_simulation_B.dv1[dynamic_simulation_B.i_n + 6];
        X->data[dynamic_simulation_B.unnamed_idx_1].f1[dynamic_simulation_B.b_k
          + 6 * dynamic_simulation_B.i_n] = dynamic_simulation_B.Tinv
          [(dynamic_simulation_B.i_n << 2) + dynamic_simulation_B.b_k];
        X->data[dynamic_simulation_B.unnamed_idx_1].f1[dynamic_simulation_B.b_k
          + 6 * (dynamic_simulation_B.i_n + 3)] = 0.0;
      }
    }

    for (dynamic_simulation_B.i_n = 0; dynamic_simulation_B.i_n < 3;
         dynamic_simulation_B.i_n++) {
      X->data[dynamic_simulation_B.unnamed_idx_1].f1[6 *
        dynamic_simulation_B.i_n + 3] = dynamic_simulation_B.R_c[3 *
        dynamic_simulation_B.i_n];
      dynamic_simulation_B.b_k = dynamic_simulation_B.i_n << 2;
      dynamic_simulation_B.q_size_tmp = 6 * (dynamic_simulation_B.i_n + 3);
      X->data[dynamic_simulation_B.unnamed_idx_1]
        .f1[dynamic_simulation_B.q_size_tmp + 3] =
        dynamic_simulation_B.Tinv[dynamic_simulation_B.b_k];
      X->data[dynamic_simulation_B.unnamed_idx_1].f1[6 *
        dynamic_simulation_B.i_n + 4] = dynamic_simulation_B.R_c[3 *
        dynamic_simulation_B.i_n + 1];
      X->data[dynamic_simulation_B.unnamed_idx_1]
        .f1[dynamic_simulation_B.q_size_tmp + 4] =
        dynamic_simulation_B.Tinv[dynamic_simulation_B.b_k + 1];
      X->data[dynamic_simulation_B.unnamed_idx_1].f1[6 *
        dynamic_simulation_B.i_n + 5] = dynamic_simulation_B.R_c[3 *
        dynamic_simulation_B.i_n + 2];
      X->data[dynamic_simulation_B.unnamed_idx_1]
        .f1[dynamic_simulation_B.q_size_tmp + 5] =
        dynamic_simulation_B.Tinv[dynamic_simulation_B.b_k + 2];
    }

    dynamic_simulation_B.a_idx_0 = robot->
      Bodies[dynamic_simulation_B.unnamed_idx_1]->ParentIndex;
    if (dynamic_simulation_B.a_idx_0 > 0.0) {
      dynamic_simulation_B.m = static_cast<int32_T>(dynamic_simulation_B.a_idx_0);
      for (dynamic_simulation_B.i_n = 0; dynamic_simulation_B.i_n < 6;
           dynamic_simulation_B.i_n++) {
        dynamic_simulation_B.a_idx_1 = 0.0;
        for (dynamic_simulation_B.b_k = 0; dynamic_simulation_B.b_k < 6;
             dynamic_simulation_B.b_k++) {
          dynamic_simulation_B.a_idx_1 += vB->data[(dynamic_simulation_B.m - 1) *
            6 + dynamic_simulation_B.b_k] * X->
            data[dynamic_simulation_B.unnamed_idx_1].f1[6 *
            dynamic_simulation_B.b_k + dynamic_simulation_B.i_n];
        }

        dynamic_simulation_B.q_data[dynamic_simulation_B.i_n] = vJ->data[6 *
          dynamic_simulation_B.unnamed_idx_1 + dynamic_simulation_B.i_n] +
          dynamic_simulation_B.a_idx_1;
      }

      for (dynamic_simulation_B.i_n = 0; dynamic_simulation_B.i_n < 6;
           dynamic_simulation_B.i_n++) {
        vB->data[dynamic_simulation_B.i_n + 6 *
          dynamic_simulation_B.unnamed_idx_1] =
          dynamic_simulation_B.q_data[dynamic_simulation_B.i_n];
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

      dynamic_simulation_B.R_c[0] = 0.0;
      dynamic_simulation_B.b_k = 6 * dynamic_simulation_B.unnamed_idx_1 + 2;
      dynamic_simulation_B.R_c[3] = -vB->data[dynamic_simulation_B.b_k];
      dynamic_simulation_B.i_n = 6 * dynamic_simulation_B.unnamed_idx_1 + 1;
      dynamic_simulation_B.R_c[6] = vB->data[dynamic_simulation_B.i_n];
      dynamic_simulation_B.R_c[1] = vB->data[dynamic_simulation_B.b_k];
      dynamic_simulation_B.R_c[4] = 0.0;
      dynamic_simulation_B.R_c[7] = -vB->data[6 *
        dynamic_simulation_B.unnamed_idx_1];
      dynamic_simulation_B.R_c[2] = -vB->data[dynamic_simulation_B.i_n];
      dynamic_simulation_B.R_c[5] = vB->data[6 *
        dynamic_simulation_B.unnamed_idx_1];
      dynamic_simulation_B.R_c[8] = 0.0;
      dynamic_simulation_B.R[3] = 0.0;
      dynamic_simulation_B.b_k = 6 * dynamic_simulation_B.unnamed_idx_1 + 5;
      dynamic_simulation_B.R[9] = -vB->data[dynamic_simulation_B.b_k];
      dynamic_simulation_B.i_n = 6 * dynamic_simulation_B.unnamed_idx_1 + 4;
      dynamic_simulation_B.R[15] = vB->data[dynamic_simulation_B.i_n];
      dynamic_simulation_B.R[4] = vB->data[dynamic_simulation_B.b_k];
      dynamic_simulation_B.R[10] = 0.0;
      dynamic_simulation_B.b_k = 6 * dynamic_simulation_B.unnamed_idx_1 + 3;
      dynamic_simulation_B.R[16] = -vB->data[dynamic_simulation_B.b_k];
      dynamic_simulation_B.R[5] = -vB->data[dynamic_simulation_B.i_n];
      dynamic_simulation_B.R[11] = vB->data[dynamic_simulation_B.b_k];
      dynamic_simulation_B.R[17] = 0.0;
      for (dynamic_simulation_B.i_n = 0; dynamic_simulation_B.i_n < 3;
           dynamic_simulation_B.i_n++) {
        dynamic_simulation_B.a_idx_1 = dynamic_simulation_B.R_c[3 *
          dynamic_simulation_B.i_n];
        dynamic_simulation_B.R[6 * dynamic_simulation_B.i_n] =
          dynamic_simulation_B.a_idx_1;
        dynamic_simulation_B.b_k = 6 * (dynamic_simulation_B.i_n + 3);
        dynamic_simulation_B.R[dynamic_simulation_B.b_k] = 0.0;
        dynamic_simulation_B.R[dynamic_simulation_B.b_k + 3] =
          dynamic_simulation_B.a_idx_1;
        dynamic_simulation_B.a_idx_1 = dynamic_simulation_B.R_c[3 *
          dynamic_simulation_B.i_n + 1];
        dynamic_simulation_B.R[6 * dynamic_simulation_B.i_n + 1] =
          dynamic_simulation_B.a_idx_1;
        dynamic_simulation_B.R[dynamic_simulation_B.b_k + 1] = 0.0;
        dynamic_simulation_B.R[dynamic_simulation_B.b_k + 4] =
          dynamic_simulation_B.a_idx_1;
        dynamic_simulation_B.a_idx_1 = dynamic_simulation_B.R_c[3 *
          dynamic_simulation_B.i_n + 2];
        dynamic_simulation_B.R[6 * dynamic_simulation_B.i_n + 2] =
          dynamic_simulation_B.a_idx_1;
        dynamic_simulation_B.R[dynamic_simulation_B.b_k + 2] = 0.0;
        dynamic_simulation_B.R[dynamic_simulation_B.b_k + 5] =
          dynamic_simulation_B.a_idx_1;
      }

      for (dynamic_simulation_B.i_n = 0; dynamic_simulation_B.i_n < 6;
           dynamic_simulation_B.i_n++) {
        dynamic_simulation_B.a_idx_1 = 0.0;
        for (dynamic_simulation_B.b_k = 0; dynamic_simulation_B.b_k < 6;
             dynamic_simulation_B.b_k++) {
          dynamic_simulation_B.a_idx_1 += aB->data[(dynamic_simulation_B.m - 1) *
            6 + dynamic_simulation_B.b_k] * X->
            data[dynamic_simulation_B.unnamed_idx_1].f1[6 *
            dynamic_simulation_B.b_k + dynamic_simulation_B.i_n];
        }

        dynamic_simulation_B.X_m[dynamic_simulation_B.i_n] =
          dynamic_simulation_B.a_idx_1 +
          dynamic_simulation_B.q_data[dynamic_simulation_B.i_n];
      }

      for (dynamic_simulation_B.i_n = 0; dynamic_simulation_B.i_n < 6;
           dynamic_simulation_B.i_n++) {
        dynamic_simulation_B.q_data[dynamic_simulation_B.i_n] = 0.0;
        for (dynamic_simulation_B.b_k = 0; dynamic_simulation_B.b_k < 6;
             dynamic_simulation_B.b_k++) {
          dynamic_simulation_B.a_idx_1 = dynamic_simulation_B.R[6 *
            dynamic_simulation_B.b_k + dynamic_simulation_B.i_n] * vJ->data[6 *
            dynamic_simulation_B.unnamed_idx_1 + dynamic_simulation_B.b_k] +
            dynamic_simulation_B.q_data[dynamic_simulation_B.i_n];
          dynamic_simulation_B.q_data[dynamic_simulation_B.i_n] =
            dynamic_simulation_B.a_idx_1;
        }

        aB->data[dynamic_simulation_B.i_n + 6 *
          dynamic_simulation_B.unnamed_idx_1] =
          dynamic_simulation_B.X_m[dynamic_simulation_B.i_n] +
          dynamic_simulation_B.q_data[dynamic_simulation_B.i_n];
      }

      dynamic_simulation_B.R_c[0] = 0.0;
      dynamic_simulation_B.R_c[3] = -dynamic_simulation_B.T[14];
      dynamic_simulation_B.R_c[6] = dynamic_simulation_B.T[13];
      dynamic_simulation_B.R_c[1] = dynamic_simulation_B.T[14];
      dynamic_simulation_B.R_c[4] = 0.0;
      dynamic_simulation_B.R_c[7] = -dynamic_simulation_B.T[12];
      dynamic_simulation_B.R_c[2] = -dynamic_simulation_B.T[13];
      dynamic_simulation_B.R_c[5] = dynamic_simulation_B.T[12];
      dynamic_simulation_B.R_c[8] = 0.0;
      for (dynamic_simulation_B.i_n = 0; dynamic_simulation_B.i_n < 3;
           dynamic_simulation_B.i_n++) {
        for (dynamic_simulation_B.b_k = 0; dynamic_simulation_B.b_k < 3;
             dynamic_simulation_B.b_k++) {
          dynamic_simulation_B.q_size_tmp = dynamic_simulation_B.i_n + 3 *
            dynamic_simulation_B.b_k;
          dynamic_simulation_B.R_b[dynamic_simulation_B.q_size_tmp] = 0.0;
          dynamic_simulation_B.p = dynamic_simulation_B.b_k << 2;
          dynamic_simulation_B.R_b[dynamic_simulation_B.q_size_tmp] +=
            dynamic_simulation_B.T[dynamic_simulation_B.p] *
            dynamic_simulation_B.R_c[dynamic_simulation_B.i_n];
          dynamic_simulation_B.R_b[dynamic_simulation_B.q_size_tmp] +=
            dynamic_simulation_B.T[dynamic_simulation_B.p + 1] *
            dynamic_simulation_B.R_c[dynamic_simulation_B.i_n + 3];
          dynamic_simulation_B.R_b[dynamic_simulation_B.q_size_tmp] +=
            dynamic_simulation_B.T[dynamic_simulation_B.p + 2] *
            dynamic_simulation_B.R_c[dynamic_simulation_B.i_n + 6];
          dynamic_simulation_B.R[dynamic_simulation_B.b_k + 6 *
            dynamic_simulation_B.i_n] = dynamic_simulation_B.T
            [(dynamic_simulation_B.i_n << 2) + dynamic_simulation_B.b_k];
          dynamic_simulation_B.R[dynamic_simulation_B.b_k + 6 *
            (dynamic_simulation_B.i_n + 3)] = 0.0;
        }
      }

      for (dynamic_simulation_B.i_n = 0; dynamic_simulation_B.i_n < 3;
           dynamic_simulation_B.i_n++) {
        dynamic_simulation_B.R[6 * dynamic_simulation_B.i_n + 3] =
          dynamic_simulation_B.R_b[3 * dynamic_simulation_B.i_n];
        dynamic_simulation_B.b_k = dynamic_simulation_B.i_n << 2;
        dynamic_simulation_B.q_size_tmp = 6 * (dynamic_simulation_B.i_n + 3);
        dynamic_simulation_B.R[dynamic_simulation_B.q_size_tmp + 3] =
          dynamic_simulation_B.T[dynamic_simulation_B.b_k];
        dynamic_simulation_B.R[6 * dynamic_simulation_B.i_n + 4] =
          dynamic_simulation_B.R_b[3 * dynamic_simulation_B.i_n + 1];
        dynamic_simulation_B.R[dynamic_simulation_B.q_size_tmp + 4] =
          dynamic_simulation_B.T[dynamic_simulation_B.b_k + 1];
        dynamic_simulation_B.R[6 * dynamic_simulation_B.i_n + 5] =
          dynamic_simulation_B.R_b[3 * dynamic_simulation_B.i_n + 2];
        dynamic_simulation_B.R[dynamic_simulation_B.q_size_tmp + 5] =
          dynamic_simulation_B.T[dynamic_simulation_B.b_k + 2];
      }

      for (dynamic_simulation_B.i_n = 0; dynamic_simulation_B.i_n < 6;
           dynamic_simulation_B.i_n++) {
        for (dynamic_simulation_B.b_k = 0; dynamic_simulation_B.b_k < 6;
             dynamic_simulation_B.b_k++) {
          dynamic_simulation_B.p = dynamic_simulation_B.i_n + 6 *
            dynamic_simulation_B.b_k;
          dynamic_simulation_B.b_I[dynamic_simulation_B.p] = 0.0;
          for (dynamic_simulation_B.q_size_tmp = 0;
               dynamic_simulation_B.q_size_tmp < 6;
               dynamic_simulation_B.q_size_tmp++) {
            dynamic_simulation_B.b_I[dynamic_simulation_B.p] += Xtree->data[
              static_cast<int32_T>(dynamic_simulation_B.a_idx_0) - 1].f1[6 *
              dynamic_simulation_B.q_size_tmp + dynamic_simulation_B.i_n] *
              dynamic_simulation_B.R[6 * dynamic_simulation_B.b_k +
              dynamic_simulation_B.q_size_tmp];
          }
        }
      }

      for (dynamic_simulation_B.i_n = 0; dynamic_simulation_B.i_n < 36;
           dynamic_simulation_B.i_n++) {
        Xtree->data[dynamic_simulation_B.unnamed_idx_1]
          .f1[dynamic_simulation_B.i_n] =
          dynamic_simulation_B.b_I[dynamic_simulation_B.i_n];
      }
    } else {
      dynamic_simulation_B.inner = S->size[1] - 1;
      for (dynamic_simulation_B.b_k = 0; dynamic_simulation_B.b_k < 6;
           dynamic_simulation_B.b_k++) {
        dynamic_simulation_B.i_n = 6 * dynamic_simulation_B.unnamed_idx_1 +
          dynamic_simulation_B.b_k;
        vB->data[dynamic_simulation_B.i_n] = vJ->data[dynamic_simulation_B.i_n];
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

      for (dynamic_simulation_B.i_n = 0; dynamic_simulation_B.i_n < 6;
           dynamic_simulation_B.i_n++) {
        dynamic_simulation_B.a_idx_1 = 0.0;
        for (dynamic_simulation_B.b_k = 0; dynamic_simulation_B.b_k < 6;
             dynamic_simulation_B.b_k++) {
          dynamic_simulation_B.a_idx_1 += X->
            data[dynamic_simulation_B.unnamed_idx_1].f1[6 *
            dynamic_simulation_B.b_k + dynamic_simulation_B.i_n] *
            dynamic_simulation_B.a0[dynamic_simulation_B.b_k];
        }

        aB->data[dynamic_simulation_B.i_n + 6 *
          dynamic_simulation_B.unnamed_idx_1] = dynamic_simulation_B.a_idx_1 +
          dynamic_simulation_B.q_data[dynamic_simulation_B.i_n];
      }

      dynamic_simulation_B.R_c[0] = 0.0;
      dynamic_simulation_B.R_c[3] = -dynamic_simulation_B.T[14];
      dynamic_simulation_B.R_c[6] = dynamic_simulation_B.T[13];
      dynamic_simulation_B.R_c[1] = dynamic_simulation_B.T[14];
      dynamic_simulation_B.R_c[4] = 0.0;
      dynamic_simulation_B.R_c[7] = -dynamic_simulation_B.T[12];
      dynamic_simulation_B.R_c[2] = -dynamic_simulation_B.T[13];
      dynamic_simulation_B.R_c[5] = dynamic_simulation_B.T[12];
      dynamic_simulation_B.R_c[8] = 0.0;
      for (dynamic_simulation_B.i_n = 0; dynamic_simulation_B.i_n < 3;
           dynamic_simulation_B.i_n++) {
        for (dynamic_simulation_B.b_k = 0; dynamic_simulation_B.b_k < 3;
             dynamic_simulation_B.b_k++) {
          dynamic_simulation_B.q_size_tmp = dynamic_simulation_B.i_n + 3 *
            dynamic_simulation_B.b_k;
          dynamic_simulation_B.R_b[dynamic_simulation_B.q_size_tmp] = 0.0;
          dynamic_simulation_B.p = dynamic_simulation_B.b_k << 2;
          dynamic_simulation_B.R_b[dynamic_simulation_B.q_size_tmp] +=
            dynamic_simulation_B.T[dynamic_simulation_B.p] *
            dynamic_simulation_B.R_c[dynamic_simulation_B.i_n];
          dynamic_simulation_B.R_b[dynamic_simulation_B.q_size_tmp] +=
            dynamic_simulation_B.T[dynamic_simulation_B.p + 1] *
            dynamic_simulation_B.R_c[dynamic_simulation_B.i_n + 3];
          dynamic_simulation_B.R_b[dynamic_simulation_B.q_size_tmp] +=
            dynamic_simulation_B.T[dynamic_simulation_B.p + 2] *
            dynamic_simulation_B.R_c[dynamic_simulation_B.i_n + 6];
          Xtree->data[dynamic_simulation_B.unnamed_idx_1]
            .f1[dynamic_simulation_B.b_k + 6 * dynamic_simulation_B.i_n] =
            dynamic_simulation_B.T[(dynamic_simulation_B.i_n << 2) +
            dynamic_simulation_B.b_k];
          Xtree->data[dynamic_simulation_B.unnamed_idx_1]
            .f1[dynamic_simulation_B.b_k + 6 * (dynamic_simulation_B.i_n + 3)] =
            0.0;
        }
      }

      for (dynamic_simulation_B.i_n = 0; dynamic_simulation_B.i_n < 3;
           dynamic_simulation_B.i_n++) {
        Xtree->data[dynamic_simulation_B.unnamed_idx_1].f1[6 *
          dynamic_simulation_B.i_n + 3] = dynamic_simulation_B.R_b[3 *
          dynamic_simulation_B.i_n];
        dynamic_simulation_B.b_k = dynamic_simulation_B.i_n << 2;
        dynamic_simulation_B.q_size_tmp = 6 * (dynamic_simulation_B.i_n + 3);
        Xtree->data[dynamic_simulation_B.unnamed_idx_1]
          .f1[dynamic_simulation_B.q_size_tmp + 3] =
          dynamic_simulation_B.T[dynamic_simulation_B.b_k];
        Xtree->data[dynamic_simulation_B.unnamed_idx_1].f1[6 *
          dynamic_simulation_B.i_n + 4] = dynamic_simulation_B.R_b[3 *
          dynamic_simulation_B.i_n + 1];
        Xtree->data[dynamic_simulation_B.unnamed_idx_1]
          .f1[dynamic_simulation_B.q_size_tmp + 4] =
          dynamic_simulation_B.T[dynamic_simulation_B.b_k + 1];
        Xtree->data[dynamic_simulation_B.unnamed_idx_1].f1[6 *
          dynamic_simulation_B.i_n + 5] = dynamic_simulation_B.R_b[3 *
          dynamic_simulation_B.i_n + 2];
        Xtree->data[dynamic_simulation_B.unnamed_idx_1]
          .f1[dynamic_simulation_B.q_size_tmp + 5] =
          dynamic_simulation_B.T[dynamic_simulation_B.b_k + 2];
      }
    }

    for (dynamic_simulation_B.i_n = 0; dynamic_simulation_B.i_n < 36;
         dynamic_simulation_B.i_n++) {
      dynamic_simulation_B.b_I[dynamic_simulation_B.i_n] = robot->
        Bodies[dynamic_simulation_B.unnamed_idx_1]->
        SpatialInertia[dynamic_simulation_B.i_n];
    }

    dynamic_simulation_B.R_c[0] = 0.0;
    dynamic_simulation_B.b_k = 6 * dynamic_simulation_B.unnamed_idx_1 + 2;
    dynamic_simulation_B.R_c[3] = -vB->data[dynamic_simulation_B.b_k];
    dynamic_simulation_B.i_n = 6 * dynamic_simulation_B.unnamed_idx_1 + 1;
    dynamic_simulation_B.R_c[6] = vB->data[dynamic_simulation_B.i_n];
    dynamic_simulation_B.R_c[1] = vB->data[dynamic_simulation_B.b_k];
    dynamic_simulation_B.R_c[4] = 0.0;
    dynamic_simulation_B.R_c[7] = -vB->data[6 *
      dynamic_simulation_B.unnamed_idx_1];
    dynamic_simulation_B.R_c[2] = -vB->data[dynamic_simulation_B.i_n];
    dynamic_simulation_B.R_c[5] = vB->data[6 *
      dynamic_simulation_B.unnamed_idx_1];
    dynamic_simulation_B.R_c[8] = 0.0;
    dynamic_simulation_B.R[18] = 0.0;
    dynamic_simulation_B.b_k = 6 * dynamic_simulation_B.unnamed_idx_1 + 5;
    dynamic_simulation_B.R[24] = -vB->data[dynamic_simulation_B.b_k];
    dynamic_simulation_B.i_n = 6 * dynamic_simulation_B.unnamed_idx_1 + 4;
    dynamic_simulation_B.R[30] = vB->data[dynamic_simulation_B.i_n];
    dynamic_simulation_B.R[19] = vB->data[dynamic_simulation_B.b_k];
    dynamic_simulation_B.R[25] = 0.0;
    dynamic_simulation_B.b_k = 6 * dynamic_simulation_B.unnamed_idx_1 + 3;
    dynamic_simulation_B.R[31] = -vB->data[dynamic_simulation_B.b_k];
    dynamic_simulation_B.R[20] = -vB->data[dynamic_simulation_B.i_n];
    dynamic_simulation_B.R[26] = vB->data[dynamic_simulation_B.b_k];
    dynamic_simulation_B.R[32] = 0.0;
    for (dynamic_simulation_B.i_n = 0; dynamic_simulation_B.i_n < 3;
         dynamic_simulation_B.i_n++) {
      dynamic_simulation_B.a_idx_1 = dynamic_simulation_B.R_c[3 *
        dynamic_simulation_B.i_n];
      dynamic_simulation_B.R[6 * dynamic_simulation_B.i_n] =
        dynamic_simulation_B.a_idx_1;
      dynamic_simulation_B.R[6 * dynamic_simulation_B.i_n + 3] = 0.0;
      dynamic_simulation_B.b_k = 6 * (dynamic_simulation_B.i_n + 3);
      dynamic_simulation_B.R[dynamic_simulation_B.b_k + 3] =
        dynamic_simulation_B.a_idx_1;
      dynamic_simulation_B.a_idx_1 = dynamic_simulation_B.R_c[3 *
        dynamic_simulation_B.i_n + 1];
      dynamic_simulation_B.R[6 * dynamic_simulation_B.i_n + 1] =
        dynamic_simulation_B.a_idx_1;
      dynamic_simulation_B.R[6 * dynamic_simulation_B.i_n + 4] = 0.0;
      dynamic_simulation_B.R[dynamic_simulation_B.b_k + 4] =
        dynamic_simulation_B.a_idx_1;
      dynamic_simulation_B.a_idx_1 = dynamic_simulation_B.R_c[3 *
        dynamic_simulation_B.i_n + 2];
      dynamic_simulation_B.R[6 * dynamic_simulation_B.i_n + 2] =
        dynamic_simulation_B.a_idx_1;
      dynamic_simulation_B.R[6 * dynamic_simulation_B.i_n + 5] = 0.0;
      dynamic_simulation_B.R[dynamic_simulation_B.b_k + 5] =
        dynamic_simulation_B.a_idx_1;
    }

    for (dynamic_simulation_B.i_n = 0; dynamic_simulation_B.i_n < 6;
         dynamic_simulation_B.i_n++) {
      dynamic_simulation_B.X_m[dynamic_simulation_B.i_n] = 0.0;
      dynamic_simulation_B.b_I_n[dynamic_simulation_B.i_n] = 0.0;
      for (dynamic_simulation_B.b_k = 0; dynamic_simulation_B.b_k < 6;
           dynamic_simulation_B.b_k++) {
        dynamic_simulation_B.a_idx_0 = dynamic_simulation_B.b_I[6 *
          dynamic_simulation_B.b_k + dynamic_simulation_B.i_n];
        dynamic_simulation_B.q_size_tmp = 6 * dynamic_simulation_B.unnamed_idx_1
          + dynamic_simulation_B.b_k;
        dynamic_simulation_B.a_idx_1 = vB->data[dynamic_simulation_B.q_size_tmp]
          * dynamic_simulation_B.a_idx_0 +
          dynamic_simulation_B.X_m[dynamic_simulation_B.i_n];
        dynamic_simulation_B.a_idx_0 = aB->data[dynamic_simulation_B.q_size_tmp]
          * dynamic_simulation_B.a_idx_0 +
          dynamic_simulation_B.b_I_n[dynamic_simulation_B.i_n];
        dynamic_simulation_B.X_m[dynamic_simulation_B.i_n] =
          dynamic_simulation_B.a_idx_1;
        dynamic_simulation_B.b_I_n[dynamic_simulation_B.i_n] =
          dynamic_simulation_B.a_idx_0;
      }
    }

    for (dynamic_simulation_B.i_n = 0; dynamic_simulation_B.i_n < 6;
         dynamic_simulation_B.i_n++) {
      dynamic_simulation_B.q_data[dynamic_simulation_B.i_n] = 0.0;
      dynamic_simulation_B.a_idx_1 = 0.0;
      for (dynamic_simulation_B.b_k = 0; dynamic_simulation_B.b_k < 6;
           dynamic_simulation_B.b_k++) {
        dynamic_simulation_B.a_idx_1 += Xtree->
          data[dynamic_simulation_B.unnamed_idx_1].f1[6 *
          dynamic_simulation_B.i_n + dynamic_simulation_B.b_k] * fext[6 *
          dynamic_simulation_B.unnamed_idx_1 + dynamic_simulation_B.b_k];
        dynamic_simulation_B.q_data[dynamic_simulation_B.i_n] +=
          dynamic_simulation_B.R[6 * dynamic_simulation_B.b_k +
          dynamic_simulation_B.i_n] *
          dynamic_simulation_B.X_m[dynamic_simulation_B.b_k];
      }

      f->data[dynamic_simulation_B.i_n + 6 * dynamic_simulation_B.unnamed_idx_1]
        = (dynamic_simulation_B.b_I_n[dynamic_simulation_B.i_n] +
           dynamic_simulation_B.q_data[dynamic_simulation_B.i_n]) -
        dynamic_simulation_B.a_idx_1;
    }
  }

  dynamic_simulati_emxFree_real_T(&aB);
  dynamic_simulati_emxFree_real_T(&vB);
  dynamic_simulati_emxFree_real_T(&vJ);
  dynamic_sim_emxFree_f_cell_wrap(&Xtree);
  dynamic_simulation_B.q_size_tmp = static_cast<int32_T>(((-1.0 -
    dynamic_simulation_B.nb) + 1.0) / -1.0) - 1;
  dynamic_simulati_emxInit_real_T(&taui, 1);
  dynamic_simulati_emxInit_char_T(&a, 2);
  if (0 <= dynamic_simulation_B.q_size_tmp) {
    for (dynamic_simulation_B.i_n = 0; dynamic_simulation_B.i_n < 5;
         dynamic_simulation_B.i_n++) {
      dynamic_simulation_B.b_j[dynamic_simulation_B.i_n] =
        tmp[dynamic_simulation_B.i_n];
    }
  }

  for (dynamic_simulation_B.loop_ub_tmp = 0; dynamic_simulation_B.loop_ub_tmp <=
       dynamic_simulation_B.q_size_tmp; dynamic_simulation_B.loop_ub_tmp++) {
    dynamic_simulation_B.a_idx_0 = dynamic_simulation_B.nb + -static_cast<real_T>
      (dynamic_simulation_B.loop_ub_tmp);
    dynamic_simulation_B.p = static_cast<int32_T>(dynamic_simulation_B.a_idx_0);
    dynamic_simulation_B.inner = dynamic_simulation_B.p - 1;
    obj = robot->Bodies[dynamic_simulation_B.inner];
    dynamic_simulation_B.i_n = a->size[0] * a->size[1];
    a->size[0] = 1;
    a->size[1] = obj->JointInternal.Type->size[1];
    dynami_emxEnsureCapacity_char_T(a, dynamic_simulation_B.i_n);
    dynamic_simulation_B.b_k = obj->JointInternal.Type->size[0] *
      obj->JointInternal.Type->size[1] - 1;
    for (dynamic_simulation_B.i_n = 0; dynamic_simulation_B.i_n <=
         dynamic_simulation_B.b_k; dynamic_simulation_B.i_n++) {
      a->data[dynamic_simulation_B.i_n] = obj->JointInternal.Type->
        data[dynamic_simulation_B.i_n];
    }

    dynamic_simulation_B.b_bool = false;
    if (a->size[1] == 5) {
      dynamic_simulation_B.i_n = 1;
      do {
        exitg1 = 0;
        if (dynamic_simulation_B.i_n - 1 < 5) {
          dynamic_simulation_B.unnamed_idx_1 = dynamic_simulation_B.i_n - 1;
          if (a->data[dynamic_simulation_B.unnamed_idx_1] !=
              dynamic_simulation_B.b_j[dynamic_simulation_B.unnamed_idx_1]) {
            exitg1 = 1;
          } else {
            dynamic_simulation_B.i_n++;
          }
        } else {
          dynamic_simulation_B.b_bool = true;
          exitg1 = 1;
        }
      } while (exitg1 == 0);
    }

    if (!dynamic_simulation_B.b_bool) {
      obj = robot->Bodies[dynamic_simulation_B.inner];
      dynamic_simulation_B.i_n = S->size[0] * S->size[1];
      S->size[0] = 6;
      S->size[1] = obj->JointInternal.MotionSubspace->size[1];
      dynami_emxEnsureCapacity_real_T(S, dynamic_simulation_B.i_n);
      dynamic_simulation_B.b_k = obj->JointInternal.MotionSubspace->size[0] *
        obj->JointInternal.MotionSubspace->size[1] - 1;
      for (dynamic_simulation_B.i_n = 0; dynamic_simulation_B.i_n <=
           dynamic_simulation_B.b_k; dynamic_simulation_B.i_n++) {
        S->data[dynamic_simulation_B.i_n] = obj->
          JointInternal.MotionSubspace->data[dynamic_simulation_B.i_n];
      }

      dynamic_simulation_B.m = S->size[1] - 1;
      dynamic_simulation_B.i_n = taui->size[0];
      taui->size[0] = S->size[1];
      dynami_emxEnsureCapacity_real_T(taui, dynamic_simulation_B.i_n);
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
        VelocityDoFMap[dynamic_simulation_B.p - 1];
      dynamic_simulation_B.b_idx_1 = robot->
        VelocityDoFMap[dynamic_simulation_B.p + 7];
      if (dynamic_simulation_B.b_idx_0 > dynamic_simulation_B.b_idx_1) {
        dynamic_simulation_B.b_k = 0;
        dynamic_simulation_B.i_n = 0;
      } else {
        dynamic_simulation_B.b_k = static_cast<int32_T>
          (dynamic_simulation_B.b_idx_0) - 1;
        dynamic_simulation_B.i_n = static_cast<int32_T>
          (dynamic_simulation_B.b_idx_1);
      }

      dynamic_simulation_B.unnamed_idx_1 = dynamic_simulation_B.i_n -
        dynamic_simulation_B.b_k;
      for (dynamic_simulation_B.i_n = 0; dynamic_simulation_B.i_n <
           dynamic_simulation_B.unnamed_idx_1; dynamic_simulation_B.i_n++) {
        tau[dynamic_simulation_B.b_k + dynamic_simulation_B.i_n] = taui->
          data[dynamic_simulation_B.i_n];
      }
    }

    dynamic_simulation_B.a_idx_0 = robot->Bodies[dynamic_simulation_B.inner]
      ->ParentIndex;
    if (dynamic_simulation_B.a_idx_0 > 0.0) {
      dynamic_simulation_B.m = static_cast<int32_T>(dynamic_simulation_B.a_idx_0);
      for (dynamic_simulation_B.i_n = 0; dynamic_simulation_B.i_n < 6;
           dynamic_simulation_B.i_n++) {
        dynamic_simulation_B.a_idx_1 = 0.0;
        for (dynamic_simulation_B.b_k = 0; dynamic_simulation_B.b_k < 6;
             dynamic_simulation_B.b_k++) {
          dynamic_simulation_B.a_idx_1 += f->data[(dynamic_simulation_B.p - 1) *
            6 + dynamic_simulation_B.b_k] * X->data[dynamic_simulation_B.inner].
            f1[6 * dynamic_simulation_B.i_n + dynamic_simulation_B.b_k];
        }

        dynamic_simulation_B.a0[dynamic_simulation_B.i_n] = f->data
          [(dynamic_simulation_B.m - 1) * 6 + dynamic_simulation_B.i_n] +
          dynamic_simulation_B.a_idx_1;
      }

      for (dynamic_simulation_B.i_n = 0; dynamic_simulation_B.i_n < 6;
           dynamic_simulation_B.i_n++) {
        f->data[dynamic_simulation_B.i_n + 6 * (dynamic_simulation_B.m - 1)] =
          dynamic_simulation_B.a0[dynamic_simulation_B.i_n];
      }
    }
  }

  dynamic_simulati_emxFree_char_T(&a);
  dynamic_simulati_emxFree_real_T(&taui);
  dynamic_simulati_emxFree_real_T(&S);
  dynamic_simulati_emxFree_real_T(&f);
  dynamic_sim_emxFree_f_cell_wrap(&X);
}

static void matlabCodegenHandle_matlabCodeg(ros_slros_internal_block_GetP_T *obj)
{
  if (!obj->matlabCodegenIsDeleted) {
    obj->matlabCodegenIsDeleted = true;
  }
}

static void dynamic_simul_matlabCodegenHa_i(ros_slros_internal_block_Subs_T *obj)
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
  emxFreeStruct_c_rigidBodyJoint(&pStruct->JointInternal);
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

static o_robotics_manip_internal_Rig_T *dy_RigidBody_RigidBody_kq2tztaj
  (o_robotics_manip_internal_Rig_T *obj)
{
  o_robotics_manip_internal_Rig_T *b_obj;
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
      dynamic_simulation_B.b_d[b_kstr] = tmp_2[b_kstr];
    }

    b_bool = false;
    if (switch_expression->size[1] == 9) {
      b_kstr = 1;
      do {
        exitg1 = 0;
        if (b_kstr - 1 < 9) {
          loop_ub = b_kstr - 1;
          if (switch_expression->data[loop_ub] !=
              dynamic_simulation_B.b_d[loop_ub]) {
            exitg1 = 1;
          } else {
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

  dy_RigidBody_RigidBody_kq2tztaj(&obj->Base);
  return b_obj;
}

// Model step function
void dynamic_simulation_step(void)
{
  robotics_slmanip_internal_blo_T *obj;
  emxArray_real_T_dynamic_simul_T *L;
  emxArray_real_T_dynamic_simul_T *lambda;
  emxArray_real_T_dynamic_simul_T *H;
  emxArray_real_T_dynamic_simul_T *tmp;
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

  // SignalConversion generated from: '<S8>/ SFunction ' incorporates:
  //   MATLAB Function: '<S2>/Assign to JointState msg'
  //   MATLABSystem: '<S10>/Get Parameter'
  //   MATLABSystem: '<S10>/Get Parameter1'
  //   MATLABSystem: '<S10>/Get Parameter2'
  //   MATLABSystem: '<S10>/Get Parameter3'
  //   MATLABSystem: '<S10>/Get Parameter4'
  //   MATLABSystem: '<S10>/Get Parameter5'

  ParamGet_dynamic_simulation_224.get_parameter(32U,
    dynamic_simulation_B.charValue,
    &dynamic_simulation_B.TmpSignalConversionAtSFunct[0]);
  ParamGet_dynamic_simulation_228.get_parameter(32U,
    dynamic_simulation_B.charValue_l,
    &dynamic_simulation_B.TmpSignalConversionAtSFunct[1]);
  ParamGet_dynamic_simulation_229.get_parameter(32U,
    dynamic_simulation_B.charValue_j,
    &dynamic_simulation_B.TmpSignalConversionAtSFunct[2]);
  ParamGet_dynamic_simulation_230.get_parameter(32U,
    dynamic_simulation_B.charValue_d,
    &dynamic_simulation_B.TmpSignalConversionAtSFunct[3]);
  ParamGet_dynamic_simulation_231.get_parameter(32U,
    dynamic_simulation_B.charValue_g,
    &dynamic_simulation_B.TmpSignalConversionAtSFunct[4]);
  ParamGet_dynamic_simulation_232.get_parameter(32U,
    dynamic_simulation_B.charValue_ld,
    &dynamic_simulation_B.TmpSignalConversionAtSFunct[5]);
  if (rtmIsMajorTimeStep(dynamic_simulation_M)) {
    // Outputs for Atomic SubSystem: '<Root>/Subscribe'
    // MATLABSystem: '<S7>/SourceBlock' incorporates:
    //   Inport: '<S13>/In1'

    dynamic_simulat_SystemCore_step(&dynamic_simulation_B.b_varargout_1,
      dynamic_simulation_B.b_varargout_2_Data,
      &dynamic_simulation_B.b_varargout_2_Data_SL_Info_Curr,
      &dynamic_simulation_B.b_varargout_2_Data_SL_Info_Rece,
      &dynamic_simulation_B.b_varargout_2_Layout_DataOffset,
      dynamic_simulation_B.b_varargout_2_Layout_Dim,
      &dynamic_simulation_B.b_varargout_2_Layout_Dim_SL_Inf,
      &dynamic_simulation_B.b_varargout_2_Layout_Dim_SL_I_c);

    // Outputs for Enabled SubSystem: '<S7>/Enabled Subsystem' incorporates:
    //   EnablePort: '<S13>/Enable'

    if (dynamic_simulation_B.b_varargout_1) {
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

    // End of MATLABSystem: '<S7>/SourceBlock'
    // End of Outputs for SubSystem: '<S7>/Enabled Subsystem'
    // End of Outputs for SubSystem: '<Root>/Subscribe'
  }

  // MATLABSystem: '<S12>/Get Parameter12'
  ParamGet_dynamic_simulation_194.get_parameter(&dynamic_simulation_B.vNum);

  // MATLABSystem: '<S12>/Get Parameter1'
  ParamGet_dynamic_simulation_132.get_parameter(&dynamic_simulation_B.k);

  // MATLABSystem: '<S12>/Get Parameter4'
  ParamGet_dynamic_simulation_137.get_parameter(&dynamic_simulation_B.j);

  // MATLABSystem: '<S12>/Get Parameter5'
  ParamGet_dynamic_simulation_138.get_parameter(&dynamic_simulation_B.value);

  // MATLABSystem: '<S12>/Get Parameter6'
  ParamGet_dynamic_simulation_139.get_parameter(&dynamic_simulation_B.value_n);

  // MATLABSystem: '<S12>/Get Parameter7'
  ParamGet_dynamic_simulation_140.get_parameter(&dynamic_simulation_B.value_b);

  // Integrator: '<S6>/Position' incorporates:
  //   MATLABSystem: '<S12>/Get Parameter1'
  //   MATLABSystem: '<S12>/Get Parameter12'
  //   MATLABSystem: '<S12>/Get Parameter4'
  //   MATLABSystem: '<S12>/Get Parameter5'
  //   MATLABSystem: '<S12>/Get Parameter6'
  //   MATLABSystem: '<S12>/Get Parameter7'

  if (dynamic_simulation_DW.Position_IWORK != 0) {
    dynamic_simulation_X.Position_CSTATE[0] = dynamic_simulation_B.vNum;
    dynamic_simulation_X.Position_CSTATE[1] = dynamic_simulation_B.k;
    dynamic_simulation_X.Position_CSTATE[2] = dynamic_simulation_B.j;
    dynamic_simulation_X.Position_CSTATE[3] = dynamic_simulation_B.value;
    dynamic_simulation_X.Position_CSTATE[4] = dynamic_simulation_B.value_n;
    dynamic_simulation_X.Position_CSTATE[5] = dynamic_simulation_B.value_b;
  }

  // MATLABSystem: '<S12>/Get Parameter2'
  ParamGet_dynamic_simulation_135.get_parameter(&dynamic_simulation_B.vNum);

  // MATLABSystem: '<S12>/Get Parameter3'
  ParamGet_dynamic_simulation_136.get_parameter(&dynamic_simulation_B.k);

  // MATLABSystem: '<S12>/Get Parameter8'
  ParamGet_dynamic_simulation_141.get_parameter(&dynamic_simulation_B.j);

  // MATLABSystem: '<S12>/Get Parameter9'
  ParamGet_dynamic_simulation_142.get_parameter(&dynamic_simulation_B.value);

  // MATLABSystem: '<S12>/Get Parameter10'
  ParamGet_dynamic_simulation_133.get_parameter(&dynamic_simulation_B.value_n);

  // MATLABSystem: '<S12>/Get Parameter11'
  ParamGet_dynamic_simulation_134.get_parameter(&dynamic_simulation_B.value_b);

  // Integrator: '<S6>/Velocity' incorporates:
  //   MATLABSystem: '<S12>/Get Parameter10'
  //   MATLABSystem: '<S12>/Get Parameter11'
  //   MATLABSystem: '<S12>/Get Parameter2'
  //   MATLABSystem: '<S12>/Get Parameter3'
  //   MATLABSystem: '<S12>/Get Parameter8'
  //   MATLABSystem: '<S12>/Get Parameter9'

  if (dynamic_simulation_DW.Velocity_IWORK != 0) {
    dynamic_simulation_X.Velocity_CSTATE[0] = dynamic_simulation_B.vNum;
    dynamic_simulation_X.Velocity_CSTATE[1] = dynamic_simulation_B.k;
    dynamic_simulation_X.Velocity_CSTATE[2] = dynamic_simulation_B.j;
    dynamic_simulation_X.Velocity_CSTATE[3] = dynamic_simulation_B.value;
    dynamic_simulation_X.Velocity_CSTATE[4] = dynamic_simulation_B.value_n;
    dynamic_simulation_X.Velocity_CSTATE[5] = dynamic_simulation_B.value_b;
  }

  for (dynamic_simulation_B.i = 0; dynamic_simulation_B.i < 6;
       dynamic_simulation_B.i++) {
    dynamic_simulation_B.Velocity[dynamic_simulation_B.i] =
      dynamic_simulation_X.Velocity_CSTATE[dynamic_simulation_B.i];
  }

  // End of Integrator: '<S6>/Velocity'
  dynamic_simulati_emxInit_real_T(&L, 2);
  dynamic_simulati_emxInit_real_T(&lambda, 2);
  dynamic_simulati_emxInit_real_T(&tmp, 1);

  // MATLABSystem: '<S11>/MATLAB System' incorporates:
  //   Constant: '<S6>/Constant'
  //   Integrator: '<S6>/Position'

  obj = &dynamic_simulation_DW.obj;
  RigidBodyTreeDynamics_massMatri(&dynamic_simulation_DW.obj.TreeInternal,
    dynamic_simulation_X.Position_CSTATE, L, lambda);
  dynamic_simulation_B.vNum = obj->TreeInternal.VelocityNumber;
  dynamic_simulation_B.vNum_idx_0_tmp = static_cast<int32_T>
    (dynamic_simulation_B.vNum);
  dynamic_simulation_B.j_i = tmp->size[0];
  tmp->size[0] = dynamic_simulation_B.vNum_idx_0_tmp;
  dynami_emxEnsureCapacity_real_T(tmp, dynamic_simulation_B.j_i);
  for (dynamic_simulation_B.j_i = 0; dynamic_simulation_B.j_i <
       dynamic_simulation_B.vNum_idx_0_tmp; dynamic_simulation_B.j_i++) {
    tmp->data[dynamic_simulation_B.j_i] = 0.0;
  }

  RigidBodyTreeDynamics_inverseDy(&obj->TreeInternal,
    dynamic_simulation_X.Position_CSTATE, dynamic_simulation_B.Velocity,
    dynamic_simulation_P.Constant_Value_ij, dynamic_simulation_B.MATLABSystem);
  dynamic_simulati_emxFree_real_T(&tmp);

  // MATLABSystem: '<S11>/MATLAB System'
  for (dynamic_simulation_B.j_i = 0; dynamic_simulation_B.j_i < 6;
       dynamic_simulation_B.j_i++) {
    dynamic_simulation_B.MATLABSystem[dynamic_simulation_B.j_i] =
      dynamic_simulation_B.In1.Data[dynamic_simulation_B.j_i + 1] -
      dynamic_simulation_B.MATLABSystem[dynamic_simulation_B.j_i];
  }

  if ((L->size[0] == 0) || (L->size[1] == 0)) {
    dynamic_simulation_B.iend = 0;
  } else {
    dynamic_simulation_B.i = L->size[0];
    dynamic_simulation_B.iend = L->size[1];
    if (dynamic_simulation_B.i > dynamic_simulation_B.iend) {
      dynamic_simulation_B.iend = dynamic_simulation_B.i;
    }
  }

  dynamic_simulati_emxInit_real_T(&H, 2);

  // MATLABSystem: '<S11>/MATLAB System'
  dynamic_simulation_B.j_i = H->size[0] * H->size[1];
  H->size[0] = L->size[0];
  H->size[1] = L->size[1];
  dynami_emxEnsureCapacity_real_T(H, dynamic_simulation_B.j_i);
  dynamic_simulation_B.n = L->size[0] * L->size[1] - 1;
  for (dynamic_simulation_B.j_i = 0; dynamic_simulation_B.j_i <=
       dynamic_simulation_B.n; dynamic_simulation_B.j_i++) {
    H->data[dynamic_simulation_B.j_i] = L->data[dynamic_simulation_B.j_i];
  }

  dynamic_simulation_B.n = static_cast<int32_T>(((-1.0 - static_cast<real_T>
    (dynamic_simulation_B.iend)) + 1.0) / -1.0) - 1;
  for (dynamic_simulation_B.i = 0; dynamic_simulation_B.i <=
       dynamic_simulation_B.n; dynamic_simulation_B.i++) {
    dynamic_simulation_B.j = static_cast<real_T>(dynamic_simulation_B.iend) + -
      static_cast<real_T>(dynamic_simulation_B.i);
    dynamic_simulation_B.j_i = static_cast<int32_T>(dynamic_simulation_B.j);
    dynamic_simulation_B.MATLABSystem_tmp = dynamic_simulation_B.j_i - 1;
    H->data[(static_cast<int32_T>(dynamic_simulation_B.j) + H->size[0] * (
              static_cast<int32_T>(dynamic_simulation_B.j) - 1)) - 1] = sqrt
      (H->data[(dynamic_simulation_B.MATLABSystem_tmp * H->size[0] +
                dynamic_simulation_B.j_i) - 1]);
    dynamic_simulation_B.k = lambda->data[dynamic_simulation_B.MATLABSystem_tmp];
    while (dynamic_simulation_B.k > 0.0) {
      dynamic_simulation_B.i_o = static_cast<int32_T>(dynamic_simulation_B.k) -
        1;
      H->data[(static_cast<int32_T>(dynamic_simulation_B.j) + H->size[0] * (
                static_cast<int32_T>(dynamic_simulation_B.k) - 1)) - 1] =
        H->data[(dynamic_simulation_B.i_o * H->size[0] +
                 dynamic_simulation_B.j_i) - 1] / H->data[((static_cast<int32_T>
        (dynamic_simulation_B.j) - 1) * H->size[0] + static_cast<int32_T>
        (dynamic_simulation_B.j)) - 1];
      dynamic_simulation_B.k = lambda->data[dynamic_simulation_B.i_o];
    }

    dynamic_simulation_B.k = lambda->data[dynamic_simulation_B.MATLABSystem_tmp];
    while (dynamic_simulation_B.k > 0.0) {
      dynamic_simulation_B.j = dynamic_simulation_B.k;
      while (dynamic_simulation_B.j > 0.0) {
        dynamic_simulation_B.MATLABSystem_tmp = static_cast<int32_T>
          (dynamic_simulation_B.j) - 1;
        H->data[(static_cast<int32_T>(dynamic_simulation_B.k) + H->size[0] * (
                  static_cast<int32_T>(dynamic_simulation_B.j) - 1)) - 1] =
          H->data[(dynamic_simulation_B.MATLABSystem_tmp * H->size[0] +
                   static_cast<int32_T>(dynamic_simulation_B.k)) - 1] - H->data
          [((static_cast<int32_T>(dynamic_simulation_B.k) - 1) * H->size[0] +
            dynamic_simulation_B.j_i) - 1] * H->data[((static_cast<int32_T>
          (dynamic_simulation_B.j) - 1) * H->size[0] + dynamic_simulation_B.j_i)
          - 1];
        dynamic_simulation_B.j = lambda->
          data[dynamic_simulation_B.MATLABSystem_tmp];
      }

      dynamic_simulation_B.k = lambda->data[static_cast<int32_T>
        (dynamic_simulation_B.k) - 1];
    }
  }

  dynamic_simulation_B.j_i = L->size[0] * L->size[1];
  L->size[0] = H->size[0];
  L->size[1] = H->size[1];
  dynami_emxEnsureCapacity_real_T(L, dynamic_simulation_B.j_i);
  dynamic_simulation_B.n = H->size[0] * H->size[1] - 1;
  for (dynamic_simulation_B.j_i = 0; dynamic_simulation_B.j_i <=
       dynamic_simulation_B.n; dynamic_simulation_B.j_i++) {
    L->data[dynamic_simulation_B.j_i] = H->data[dynamic_simulation_B.j_i];
  }

  dynamic_simulation_B.n = H->size[1];
  if ((H->size[0] == 0) || (H->size[1] == 0) || (1 >= H->size[1])) {
  } else {
    dynamic_simulation_B.iend = 0;
    for (dynamic_simulation_B.j_i = 2; dynamic_simulation_B.j_i <=
         dynamic_simulation_B.n; dynamic_simulation_B.j_i++) {
      for (dynamic_simulation_B.i = 0; dynamic_simulation_B.i <=
           dynamic_simulation_B.iend; dynamic_simulation_B.i++) {
        L->data[dynamic_simulation_B.i + L->size[0] * (dynamic_simulation_B.j_i
          - 1)] = 0.0;
      }

      if (dynamic_simulation_B.iend + 1 < H->size[0]) {
        dynamic_simulation_B.iend++;
      }
    }
  }

  dynamic_simulati_emxFree_real_T(&H);

  // MATLABSystem: '<S11>/MATLAB System'
  dynamic_simulation_B.n = static_cast<int32_T>(((-1.0 -
    dynamic_simulation_B.vNum) + 1.0) / -1.0) - 1;
  for (dynamic_simulation_B.i = 0; dynamic_simulation_B.i <=
       dynamic_simulation_B.n; dynamic_simulation_B.i++) {
    dynamic_simulation_B.iend = static_cast<int32_T>(dynamic_simulation_B.vNum +
      -static_cast<real_T>(dynamic_simulation_B.i));
    dynamic_simulation_B.j_i = dynamic_simulation_B.iend - 1;
    dynamic_simulation_B.MATLABSystem[dynamic_simulation_B.j_i] /= L->data
      [(dynamic_simulation_B.j_i * L->size[0] + dynamic_simulation_B.iend) - 1];
    dynamic_simulation_B.j = lambda->data[dynamic_simulation_B.j_i];
    while (dynamic_simulation_B.j > 0.0) {
      dynamic_simulation_B.MATLABSystem_tmp = static_cast<int32_T>
        (dynamic_simulation_B.j) - 1;
      dynamic_simulation_B.MATLABSystem[dynamic_simulation_B.MATLABSystem_tmp] -=
        L->data[(dynamic_simulation_B.MATLABSystem_tmp * L->size[0] +
                 dynamic_simulation_B.iend) - 1] *
        dynamic_simulation_B.MATLABSystem[dynamic_simulation_B.j_i];
      dynamic_simulation_B.j = lambda->
        data[dynamic_simulation_B.MATLABSystem_tmp];
    }
  }

  dynamic_simulation_B.vNum_idx_0_tmp--;
  for (dynamic_simulation_B.i = 0; dynamic_simulation_B.i <=
       dynamic_simulation_B.vNum_idx_0_tmp; dynamic_simulation_B.i++) {
    dynamic_simulation_B.j = lambda->data[dynamic_simulation_B.i];
    while (dynamic_simulation_B.j > 0.0) {
      dynamic_simulation_B.j_i = static_cast<int32_T>(dynamic_simulation_B.j) -
        1;
      dynamic_simulation_B.MATLABSystem[dynamic_simulation_B.i] -= L->
        data[dynamic_simulation_B.j_i * L->size[0] + dynamic_simulation_B.i] *
        dynamic_simulation_B.MATLABSystem[dynamic_simulation_B.j_i];
      dynamic_simulation_B.j = lambda->data[dynamic_simulation_B.j_i];
    }

    dynamic_simulation_B.MATLABSystem[dynamic_simulation_B.i] /= L->data[L->
      size[0] * dynamic_simulation_B.i + dynamic_simulation_B.i];
  }

  dynamic_simulati_emxFree_real_T(&lambda);
  dynamic_simulati_emxFree_real_T(&L);
  for (dynamic_simulation_B.j_i = 0; dynamic_simulation_B.j_i < 32;
       dynamic_simulation_B.j_i++) {
    // SignalConversion generated from: '<S8>/ SFunction ' incorporates:
    //   MATLAB Function: '<S2>/Assign to JointState msg'
    //   MATLABSystem: '<S10>/Get Parameter'
    //   MATLABSystem: '<S10>/Get Parameter1'
    //   MATLABSystem: '<S10>/Get Parameter2'
    //   MATLABSystem: '<S10>/Get Parameter3'
    //   MATLABSystem: '<S10>/Get Parameter4'
    //   MATLABSystem: '<S10>/Get Parameter5'

    dynamic_simulation_B.TmpSignalConversionAtSFun_b[dynamic_simulation_B.j_i] =
      static_cast<uint8_T>
      (dynamic_simulation_B.charValue[dynamic_simulation_B.j_i]);
    dynamic_simulation_B.TmpSignalConversionAtSFun_b[dynamic_simulation_B.j_i +
      32] = static_cast<uint8_T>
      (dynamic_simulation_B.charValue_l[dynamic_simulation_B.j_i]);
    dynamic_simulation_B.TmpSignalConversionAtSFun_b[dynamic_simulation_B.j_i +
      64] = static_cast<uint8_T>
      (dynamic_simulation_B.charValue_j[dynamic_simulation_B.j_i]);
    dynamic_simulation_B.TmpSignalConversionAtSFun_b[dynamic_simulation_B.j_i +
      96] = static_cast<uint8_T>
      (dynamic_simulation_B.charValue_d[dynamic_simulation_B.j_i]);
    dynamic_simulation_B.TmpSignalConversionAtSFun_b[dynamic_simulation_B.j_i +
      128] = static_cast<uint8_T>
      (dynamic_simulation_B.charValue_g[dynamic_simulation_B.j_i]);
    dynamic_simulation_B.TmpSignalConversionAtSFun_b[dynamic_simulation_B.j_i +
      160] = static_cast<uint8_T>
      (dynamic_simulation_B.charValue_ld[dynamic_simulation_B.j_i]);
  }

  // BusAssignment: '<S2>/Bus Assignment' incorporates:
  //   Constant: '<S9>/Constant'
  //   MATLAB Function: '<S2>/Assign to JointState msg'

  dynamic_simulation_B.BusAssignment = dynamic_simulation_P.Constant_Value;

  // MATLAB Function: '<S2>/Assign to JointState msg' incorporates:
  //   BusAssignment: '<S2>/Bus Assignment'
  //   Constant: '<S2>/Constant'
  //   Integrator: '<S6>/Position'

  dynamic_simulation_B.BusAssignment.Name_SL_Info.CurrentLength = 6U;
  dynamic_simulation_B.BusAssignment.Position_SL_Info.CurrentLength = 6U;
  dynamic_simulation_B.BusAssignment.Velocity_SL_Info.CurrentLength = 6U;
  for (dynamic_simulation_B.i = 0; dynamic_simulation_B.i < 6;
       dynamic_simulation_B.i++) {
    dynamic_simulation_B.vNum = ((static_cast<real_T>(dynamic_simulation_B.i) +
      1.0) - 1.0) * static_cast<real_T>(dynamic_simulation_P.name_max_length);
    if (dynamic_simulation_B.vNum < 4.294967296E+9) {
      dynamic_simulation_B.b_varargout_2_Data_SL_Info_Rece =
        static_cast<uint32_T>(dynamic_simulation_B.vNum);
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
      dynamic_simulation_B.iend = 0;
      dynamic_simulation_B.vNum_idx_0_tmp = 0;
    } else {
      dynamic_simulation_B.iend = static_cast<int32_T>
        (dynamic_simulation_B.b_varargout_2_Data_SL_Info_Curr) - 1;
      dynamic_simulation_B.vNum_idx_0_tmp = static_cast<int32_T>
        (dynamic_simulation_B.b_varargout_2_Data_SL_Info_Rece);
    }

    dynamic_simulation_B.n = dynamic_simulation_B.vNum_idx_0_tmp -
      dynamic_simulation_B.iend;
    for (dynamic_simulation_B.j_i = 0; dynamic_simulation_B.j_i <
         dynamic_simulation_B.n; dynamic_simulation_B.j_i++) {
      dynamic_simulation_B.BusAssignment.Name[dynamic_simulation_B.i]
        .Data[dynamic_simulation_B.j_i] =
        dynamic_simulation_B.TmpSignalConversionAtSFun_b[dynamic_simulation_B.iend
        + dynamic_simulation_B.j_i];
    }

    dynamic_simulation_B.BusAssignment.Name[dynamic_simulation_B.i].
      Data_SL_Info.CurrentLength =
      dynamic_simulation_B.TmpSignalConversionAtSFunct[dynamic_simulation_B.i];
    dynamic_simulation_B.BusAssignment.Position[dynamic_simulation_B.i] =
      dynamic_simulation_X.Position_CSTATE[dynamic_simulation_B.i];
    dynamic_simulation_B.BusAssignment.Velocity[dynamic_simulation_B.i] =
      dynamic_simulation_B.Velocity[dynamic_simulation_B.i];
  }

  // Clock: '<Root>/Clock1'
  dynamic_simulation_B.k = dynamic_simulation_M->Timing.t[0];

  // MATLAB Function: '<Root>/MATLAB Function'
  if (dynamic_simulation_B.k < 0.0) {
    dynamic_simulation_B.vNum = ceil(dynamic_simulation_B.k);
  } else {
    dynamic_simulation_B.vNum = floor(dynamic_simulation_B.k);
  }

  dynamic_simulation_B.k = (dynamic_simulation_B.k - dynamic_simulation_B.vNum) *
    1.0E+9;
  if (dynamic_simulation_B.k < 0.0) {
    dynamic_simulation_B.k = ceil(dynamic_simulation_B.k);
  } else {
    dynamic_simulation_B.k = floor(dynamic_simulation_B.k);
  }

  // BusAssignment: '<S2>/Bus Assignment' incorporates:
  //   BusCreator generated from: '<S2>/Bus Assignment'
  //   MATLAB Function: '<Root>/MATLAB Function'

  dynamic_simulation_B.BusAssignment.Header.Stamp.Sec =
    dynamic_simulation_B.vNum;
  dynamic_simulation_B.BusAssignment.Header.Stamp.Nsec = dynamic_simulation_B.k;

  // Outputs for Atomic SubSystem: '<Root>/Publish'
  // MATLABSystem: '<S4>/SinkBlock'
  Pub_dynamic_simulation_157.publish(&dynamic_simulation_B.BusAssignment);

  // End of Outputs for SubSystem: '<Root>/Publish'

  // BusAssignment: '<Root>/Bus Assignment' incorporates:
  //   BusCreator generated from: '<Root>/Bus Assignment'
  //   MATLAB Function: '<Root>/MATLAB Function'

  dynamic_simulation_B.BusAssignment_o.Clock_.Sec = dynamic_simulation_B.vNum;
  dynamic_simulation_B.BusAssignment_o.Clock_.Nsec = dynamic_simulation_B.k;

  // Outputs for Atomic SubSystem: '<Root>/Publish1'
  // MATLABSystem: '<S5>/SinkBlock'
  Pub_dynamic_simulation_158.publish(&dynamic_simulation_B.BusAssignment_o);

  // End of Outputs for SubSystem: '<Root>/Publish1'
  if (rtmIsMajorTimeStep(dynamic_simulation_M)) {
    // Update for Integrator: '<S6>/Position'
    dynamic_simulation_DW.Position_IWORK = 0;

    // Update for Integrator: '<S6>/Velocity'
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
    // Derivatives for Integrator: '<S6>/Position'
    _rtXdot->Position_CSTATE[i] = dynamic_simulation_B.Velocity[i];

    // Derivatives for Integrator: '<S6>/Velocity'
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

    static const char_T tmp_1[13] = { '/', 'j', 'o', 'i', 'n', 't', '_', 's',
      't', 'a', 't', 'e', 's' };

    static const char_T tmp_2[6] = { '/', 'c', 'l', 'o', 'c', 'k' };

    static const char_T tmp_3[24] = { '/', 'c', 'o', 'n', 't', 'r', 'o', 'l',
      'l', 'e', 'r', '_', 'j', 'o', 'i', 'n', 't', '_', 'n', 'a', 'm', 'e', '_',
      '1' };

    static const char_T tmp_4[11] = { 'e', 'd', 'o', '_', 'j', 'o', 'i', 'n',
      't', '_', '1' };

    static const char_T tmp_5[24] = { '/', 'c', 'o', 'n', 't', 'r', 'o', 'l',
      'l', 'e', 'r', '_', 'j', 'o', 'i', 'n', 't', '_', 'n', 'a', 'm', 'e', '_',
      '2' };

    static const char_T tmp_6[11] = { 'e', 'd', 'o', '_', 'j', 'o', 'i', 'n',
      't', '_', '2' };

    static const char_T tmp_7[24] = { '/', 'c', 'o', 'n', 't', 'r', 'o', 'l',
      'l', 'e', 'r', '_', 'j', 'o', 'i', 'n', 't', '_', 'n', 'a', 'm', 'e', '_',
      '3' };

    static const char_T tmp_8[11] = { 'e', 'd', 'o', '_', 'j', 'o', 'i', 'n',
      't', '_', '3' };

    static const char_T tmp_9[24] = { '/', 'c', 'o', 'n', 't', 'r', 'o', 'l',
      'l', 'e', 'r', '_', 'j', 'o', 'i', 'n', 't', '_', 'n', 'a', 'm', 'e', '_',
      '4' };

    static const char_T tmp_a[11] = { 'e', 'd', 'o', '_', 'j', 'o', 'i', 'n',
      't', '_', '4' };

    static const char_T tmp_b[24] = { '/', 'c', 'o', 'n', 't', 'r', 'o', 'l',
      'l', 'e', 'r', '_', 'j', 'o', 'i', 'n', 't', '_', 'n', 'a', 'm', 'e', '_',
      '5' };

    static const char_T tmp_c[11] = { 'e', 'd', 'o', '_', 'j', 'o', 'i', 'n',
      't', '_', '5' };

    static const char_T tmp_d[24] = { '/', 'c', 'o', 'n', 't', 'r', 'o', 'l',
      'l', 'e', 'r', '_', 'j', 'o', 'i', 'n', 't', '_', 'n', 'a', 'm', 'e', '_',
      '6' };

    static const char_T tmp_e[11] = { 'e', 'd', 'o', '_', 'j', 'o', 'i', 'n',
      't', '_', '6' };

    static const char_T tmp_f[11] = { '/', 'q', '1', '_', 'i', 'n', 'i', 't',
      'i', 'a', 'l' };

    static const char_T tmp_g[11] = { '/', 'q', '2', '_', 'i', 'n', 'i', 't',
      'i', 'a', 'l' };

    static const char_T tmp_h[11] = { '/', 'q', '3', '_', 'i', 'n', 'i', 't',
      'i', 'a', 'l' };

    static const char_T tmp_i[11] = { '/', 'q', '4', '_', 'i', 'n', 'i', 't',
      'i', 'a', 'l' };

    static const char_T tmp_j[11] = { '/', 'q', '5', '_', 'i', 'n', 'i', 't',
      'i', 'a', 'l' };

    static const char_T tmp_k[11] = { '/', 'q', '6', '_', 'i', 'n', 'i', 't',
      'i', 'a', 'l' };

    static const char_T tmp_l[12] = { '/', 'q', 'v', '1', '_', 'i', 'n', 'i',
      't', 'i', 'a', 'l' };

    static const char_T tmp_m[12] = { '/', 'q', 'v', '2', '_', 'i', 'n', 'i',
      't', 'i', 'a', 'l' };

    static const char_T tmp_n[12] = { '/', 'q', 'v', '3', '_', 'i', 'n', 'i',
      't', 'i', 'a', 'l' };

    static const char_T tmp_o[12] = { '/', 'q', 'v', '4', '_', 'i', 'n', 'i',
      't', 'i', 'a', 'l' };

    static const char_T tmp_p[12] = { '/', 'q', 'v', '5', '_', 'i', 'n', 'i',
      't', 'i', 'a', 'l' };

    static const char_T tmp_q[12] = { '/', 'q', 'v', '6', '_', 'i', 'n', 'i',
      't', 'i', 'a', 'l' };

    // InitializeConditions for Integrator: '<S6>/Position' incorporates:
    //   Integrator: '<S6>/Velocity'

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

    // End of InitializeConditions for Integrator: '<S6>/Position'

    // InitializeConditions for Integrator: '<S6>/Velocity'
    dynamic_simulation_DW.Velocity_IWORK = 1;

    // SystemInitialize for Atomic SubSystem: '<Root>/Subscribe'
    // SystemInitialize for Enabled SubSystem: '<S7>/Enabled Subsystem'
    // SystemInitialize for Outport: '<S13>/Out1'
    dynamic_simulation_B.In1 = dynamic_simulation_P.Out1_Y0;

    // End of SystemInitialize for SubSystem: '<S7>/Enabled Subsystem'

    // Start for MATLABSystem: '<S7>/SourceBlock'
    dynamic_simulation_DW.obj_fk.matlabCodegenIsDeleted = false;
    dynamic_simulation_DW.obj_fk.isInitialized = 1;
    for (i = 0; i < 13; i++) {
      dynamic_simulation_B.cv1[i] = tmp_0[i];
    }

    dynamic_simulation_B.cv1[13] = '\x00';
    Sub_dynamic_simulation_160.createSubscriber(dynamic_simulation_B.cv1, 1);
    dynamic_simulation_DW.obj_fk.isSetupComplete = true;

    // End of Start for MATLABSystem: '<S7>/SourceBlock'
    // End of SystemInitialize for SubSystem: '<Root>/Subscribe'

    // SystemInitialize for Atomic SubSystem: '<Root>/Publish'
    // Start for MATLABSystem: '<S4>/SinkBlock'
    dynamic_simulation_DW.obj_cd.matlabCodegenIsDeleted = false;
    dynamic_simulation_DW.obj_cd.isInitialized = 1;
    for (i = 0; i < 13; i++) {
      dynamic_simulation_B.cv1[i] = tmp_1[i];
    }

    dynamic_simulation_B.cv1[13] = '\x00';
    Pub_dynamic_simulation_157.createPublisher(dynamic_simulation_B.cv1, 1);
    dynamic_simulation_DW.obj_cd.isSetupComplete = true;

    // End of Start for MATLABSystem: '<S4>/SinkBlock'
    // End of SystemInitialize for SubSystem: '<Root>/Publish'

    // SystemInitialize for Atomic SubSystem: '<Root>/Publish1'
    // Start for MATLABSystem: '<S5>/SinkBlock'
    dynamic_simulation_DW.obj_iv.matlabCodegenIsDeleted = false;
    dynamic_simulation_DW.obj_iv.isInitialized = 1;
    for (i = 0; i < 6; i++) {
      tmp[i] = tmp_2[i];
    }

    tmp[6] = '\x00';
    Pub_dynamic_simulation_158.createPublisher(tmp, 1);
    dynamic_simulation_DW.obj_iv.isSetupComplete = true;

    // End of Start for MATLABSystem: '<S5>/SinkBlock'
    // End of SystemInitialize for SubSystem: '<Root>/Publish1'

    // Start for MATLABSystem: '<S10>/Get Parameter'
    dynamic_simulation_DW.obj_m.matlabCodegenIsDeleted = false;
    dynamic_simulation_DW.obj_m.isInitialized = 1;
    for (i = 0; i < 24; i++) {
      dynamic_simulation_B.cv[i] = tmp_3[i];
    }

    dynamic_simulation_B.cv[24] = '\x00';
    ParamGet_dynamic_simulation_224.initialize(dynamic_simulation_B.cv);
    ParamGet_dynamic_simulation_224.initialize_error_codes(0, 1, 2, 3);
    for (i = 0; i < 11; i++) {
      dynamic_simulation_B.initialValue[i] = tmp_4[i];
    }

    ParamGet_dynamic_simulation_224.set_initial_value
      (dynamic_simulation_B.initialValue, 11U);
    dynamic_simulation_DW.obj_m.isSetupComplete = true;

    // End of Start for MATLABSystem: '<S10>/Get Parameter'

    // Start for MATLABSystem: '<S10>/Get Parameter1'
    dynamic_simulation_DW.obj_c.matlabCodegenIsDeleted = false;
    dynamic_simulation_DW.obj_c.isInitialized = 1;
    for (i = 0; i < 24; i++) {
      dynamic_simulation_B.cv[i] = tmp_5[i];
    }

    dynamic_simulation_B.cv[24] = '\x00';
    ParamGet_dynamic_simulation_228.initialize(dynamic_simulation_B.cv);
    ParamGet_dynamic_simulation_228.initialize_error_codes(0, 1, 2, 3);
    for (i = 0; i < 11; i++) {
      dynamic_simulation_B.initialValue[i] = tmp_6[i];
    }

    ParamGet_dynamic_simulation_228.set_initial_value
      (dynamic_simulation_B.initialValue, 11U);
    dynamic_simulation_DW.obj_c.isSetupComplete = true;

    // End of Start for MATLABSystem: '<S10>/Get Parameter1'

    // Start for MATLABSystem: '<S10>/Get Parameter2'
    dynamic_simulation_DW.obj_px.matlabCodegenIsDeleted = false;
    dynamic_simulation_DW.obj_px.isInitialized = 1;
    for (i = 0; i < 24; i++) {
      dynamic_simulation_B.cv[i] = tmp_7[i];
    }

    dynamic_simulation_B.cv[24] = '\x00';
    ParamGet_dynamic_simulation_229.initialize(dynamic_simulation_B.cv);
    ParamGet_dynamic_simulation_229.initialize_error_codes(0, 1, 2, 3);
    for (i = 0; i < 11; i++) {
      dynamic_simulation_B.initialValue[i] = tmp_8[i];
    }

    ParamGet_dynamic_simulation_229.set_initial_value
      (dynamic_simulation_B.initialValue, 11U);
    dynamic_simulation_DW.obj_px.isSetupComplete = true;

    // End of Start for MATLABSystem: '<S10>/Get Parameter2'

    // Start for MATLABSystem: '<S10>/Get Parameter3'
    dynamic_simulation_DW.obj_nt.matlabCodegenIsDeleted = false;
    dynamic_simulation_DW.obj_nt.isInitialized = 1;
    for (i = 0; i < 24; i++) {
      dynamic_simulation_B.cv[i] = tmp_9[i];
    }

    dynamic_simulation_B.cv[24] = '\x00';
    ParamGet_dynamic_simulation_230.initialize(dynamic_simulation_B.cv);
    ParamGet_dynamic_simulation_230.initialize_error_codes(0, 1, 2, 3);
    for (i = 0; i < 11; i++) {
      dynamic_simulation_B.initialValue[i] = tmp_a[i];
    }

    ParamGet_dynamic_simulation_230.set_initial_value
      (dynamic_simulation_B.initialValue, 11U);
    dynamic_simulation_DW.obj_nt.isSetupComplete = true;

    // End of Start for MATLABSystem: '<S10>/Get Parameter3'

    // Start for MATLABSystem: '<S10>/Get Parameter4'
    dynamic_simulation_DW.obj_o.matlabCodegenIsDeleted = false;
    dynamic_simulation_DW.obj_o.isInitialized = 1;
    for (i = 0; i < 24; i++) {
      dynamic_simulation_B.cv[i] = tmp_b[i];
    }

    dynamic_simulation_B.cv[24] = '\x00';
    ParamGet_dynamic_simulation_231.initialize(dynamic_simulation_B.cv);
    ParamGet_dynamic_simulation_231.initialize_error_codes(0, 1, 2, 3);
    for (i = 0; i < 11; i++) {
      dynamic_simulation_B.initialValue[i] = tmp_c[i];
    }

    ParamGet_dynamic_simulation_231.set_initial_value
      (dynamic_simulation_B.initialValue, 11U);
    dynamic_simulation_DW.obj_o.isSetupComplete = true;

    // End of Start for MATLABSystem: '<S10>/Get Parameter4'

    // Start for MATLABSystem: '<S10>/Get Parameter5'
    dynamic_simulation_DW.obj_m4.matlabCodegenIsDeleted = false;
    dynamic_simulation_DW.obj_m4.isInitialized = 1;
    for (i = 0; i < 24; i++) {
      dynamic_simulation_B.cv[i] = tmp_d[i];
    }

    dynamic_simulation_B.cv[24] = '\x00';
    ParamGet_dynamic_simulation_232.initialize(dynamic_simulation_B.cv);
    ParamGet_dynamic_simulation_232.initialize_error_codes(0, 1, 2, 3);
    for (i = 0; i < 11; i++) {
      dynamic_simulation_B.initialValue[i] = tmp_e[i];
    }

    ParamGet_dynamic_simulation_232.set_initial_value
      (dynamic_simulation_B.initialValue, 11U);
    dynamic_simulation_DW.obj_m4.isSetupComplete = true;

    // End of Start for MATLABSystem: '<S10>/Get Parameter5'

    // Start for MATLABSystem: '<S12>/Get Parameter12'
    dynamic_simulation_DW.obj_l.matlabCodegenIsDeleted = false;
    dynamic_simulation_DW.obj_l.isInitialized = 1;
    for (i = 0; i < 11; i++) {
      dynamic_simulation_B.cv3[i] = tmp_f[i];
    }

    dynamic_simulation_B.cv3[11] = '\x00';
    ParamGet_dynamic_simulation_194.initialize(dynamic_simulation_B.cv3);
    ParamGet_dynamic_simulation_194.initialize_error_codes(0, 1, 2, 3);
    ParamGet_dynamic_simulation_194.set_initial_value(0.0);
    dynamic_simulation_DW.obj_l.isSetupComplete = true;

    // End of Start for MATLABSystem: '<S12>/Get Parameter12'

    // Start for MATLABSystem: '<S12>/Get Parameter1'
    dynamic_simulation_DW.obj_lm.matlabCodegenIsDeleted = false;
    dynamic_simulation_DW.obj_lm.isInitialized = 1;
    for (i = 0; i < 11; i++) {
      dynamic_simulation_B.cv3[i] = tmp_g[i];
    }

    dynamic_simulation_B.cv3[11] = '\x00';
    ParamGet_dynamic_simulation_132.initialize(dynamic_simulation_B.cv3);
    ParamGet_dynamic_simulation_132.initialize_error_codes(0, 1, 2, 3);
    ParamGet_dynamic_simulation_132.set_initial_value(0.0);
    dynamic_simulation_DW.obj_lm.isSetupComplete = true;

    // End of Start for MATLABSystem: '<S12>/Get Parameter1'

    // Start for MATLABSystem: '<S12>/Get Parameter4'
    dynamic_simulation_DW.obj_a.matlabCodegenIsDeleted = false;
    dynamic_simulation_DW.obj_a.isInitialized = 1;
    for (i = 0; i < 11; i++) {
      dynamic_simulation_B.cv3[i] = tmp_h[i];
    }

    dynamic_simulation_B.cv3[11] = '\x00';
    ParamGet_dynamic_simulation_137.initialize(dynamic_simulation_B.cv3);
    ParamGet_dynamic_simulation_137.initialize_error_codes(0, 1, 2, 3);
    ParamGet_dynamic_simulation_137.set_initial_value(0.0);
    dynamic_simulation_DW.obj_a.isSetupComplete = true;

    // End of Start for MATLABSystem: '<S12>/Get Parameter4'

    // Start for MATLABSystem: '<S12>/Get Parameter5'
    dynamic_simulation_DW.obj_d.matlabCodegenIsDeleted = false;
    dynamic_simulation_DW.obj_d.isInitialized = 1;
    for (i = 0; i < 11; i++) {
      dynamic_simulation_B.cv3[i] = tmp_i[i];
    }

    dynamic_simulation_B.cv3[11] = '\x00';
    ParamGet_dynamic_simulation_138.initialize(dynamic_simulation_B.cv3);
    ParamGet_dynamic_simulation_138.initialize_error_codes(0, 1, 2, 3);
    ParamGet_dynamic_simulation_138.set_initial_value(0.0);
    dynamic_simulation_DW.obj_d.isSetupComplete = true;

    // End of Start for MATLABSystem: '<S12>/Get Parameter5'

    // Start for MATLABSystem: '<S12>/Get Parameter6'
    dynamic_simulation_DW.obj_h.matlabCodegenIsDeleted = false;
    dynamic_simulation_DW.obj_h.isInitialized = 1;
    for (i = 0; i < 11; i++) {
      dynamic_simulation_B.cv3[i] = tmp_j[i];
    }

    dynamic_simulation_B.cv3[11] = '\x00';
    ParamGet_dynamic_simulation_139.initialize(dynamic_simulation_B.cv3);
    ParamGet_dynamic_simulation_139.initialize_error_codes(0, 1, 2, 3);
    ParamGet_dynamic_simulation_139.set_initial_value(0.0);
    dynamic_simulation_DW.obj_h.isSetupComplete = true;

    // End of Start for MATLABSystem: '<S12>/Get Parameter6'

    // Start for MATLABSystem: '<S12>/Get Parameter7'
    dynamic_simulation_DW.obj_f.matlabCodegenIsDeleted = false;
    dynamic_simulation_DW.obj_f.isInitialized = 1;
    for (i = 0; i < 11; i++) {
      dynamic_simulation_B.cv3[i] = tmp_k[i];
    }

    dynamic_simulation_B.cv3[11] = '\x00';
    ParamGet_dynamic_simulation_140.initialize(dynamic_simulation_B.cv3);
    ParamGet_dynamic_simulation_140.initialize_error_codes(0, 1, 2, 3);
    ParamGet_dynamic_simulation_140.set_initial_value(0.0);
    dynamic_simulation_DW.obj_f.isSetupComplete = true;

    // End of Start for MATLABSystem: '<S12>/Get Parameter7'

    // Start for MATLABSystem: '<S12>/Get Parameter2'
    dynamic_simulation_DW.obj_e.matlabCodegenIsDeleted = false;
    dynamic_simulation_DW.obj_e.isInitialized = 1;
    for (i = 0; i < 12; i++) {
      dynamic_simulation_B.cv2[i] = tmp_l[i];
    }

    dynamic_simulation_B.cv2[12] = '\x00';
    ParamGet_dynamic_simulation_135.initialize(dynamic_simulation_B.cv2);
    ParamGet_dynamic_simulation_135.initialize_error_codes(0, 1, 2, 3);
    ParamGet_dynamic_simulation_135.set_initial_value(0.0);
    dynamic_simulation_DW.obj_e.isSetupComplete = true;

    // End of Start for MATLABSystem: '<S12>/Get Parameter2'

    // Start for MATLABSystem: '<S12>/Get Parameter3'
    dynamic_simulation_DW.obj_ez.matlabCodegenIsDeleted = false;
    dynamic_simulation_DW.obj_ez.isInitialized = 1;
    for (i = 0; i < 12; i++) {
      dynamic_simulation_B.cv2[i] = tmp_m[i];
    }

    dynamic_simulation_B.cv2[12] = '\x00';
    ParamGet_dynamic_simulation_136.initialize(dynamic_simulation_B.cv2);
    ParamGet_dynamic_simulation_136.initialize_error_codes(0, 1, 2, 3);
    ParamGet_dynamic_simulation_136.set_initial_value(0.0);
    dynamic_simulation_DW.obj_ez.isSetupComplete = true;

    // End of Start for MATLABSystem: '<S12>/Get Parameter3'

    // Start for MATLABSystem: '<S12>/Get Parameter8'
    dynamic_simulation_DW.obj_i.matlabCodegenIsDeleted = false;
    dynamic_simulation_DW.obj_i.isInitialized = 1;
    for (i = 0; i < 12; i++) {
      dynamic_simulation_B.cv2[i] = tmp_n[i];
    }

    dynamic_simulation_B.cv2[12] = '\x00';
    ParamGet_dynamic_simulation_141.initialize(dynamic_simulation_B.cv2);
    ParamGet_dynamic_simulation_141.initialize_error_codes(0, 1, 2, 3);
    ParamGet_dynamic_simulation_141.set_initial_value(0.0);
    dynamic_simulation_DW.obj_i.isSetupComplete = true;

    // End of Start for MATLABSystem: '<S12>/Get Parameter8'

    // Start for MATLABSystem: '<S12>/Get Parameter9'
    dynamic_simulation_DW.obj_ezd.matlabCodegenIsDeleted = false;
    dynamic_simulation_DW.obj_ezd.isInitialized = 1;
    for (i = 0; i < 12; i++) {
      dynamic_simulation_B.cv2[i] = tmp_o[i];
    }

    dynamic_simulation_B.cv2[12] = '\x00';
    ParamGet_dynamic_simulation_142.initialize(dynamic_simulation_B.cv2);
    ParamGet_dynamic_simulation_142.initialize_error_codes(0, 1, 2, 3);
    ParamGet_dynamic_simulation_142.set_initial_value(0.0);
    dynamic_simulation_DW.obj_ezd.isSetupComplete = true;

    // End of Start for MATLABSystem: '<S12>/Get Parameter9'

    // Start for MATLABSystem: '<S12>/Get Parameter10'
    dynamic_simulation_DW.obj_n.matlabCodegenIsDeleted = false;
    dynamic_simulation_DW.obj_n.isInitialized = 1;
    for (i = 0; i < 12; i++) {
      dynamic_simulation_B.cv2[i] = tmp_p[i];
    }

    dynamic_simulation_B.cv2[12] = '\x00';
    ParamGet_dynamic_simulation_133.initialize(dynamic_simulation_B.cv2);
    ParamGet_dynamic_simulation_133.initialize_error_codes(0, 1, 2, 3);
    ParamGet_dynamic_simulation_133.set_initial_value(0.0);
    dynamic_simulation_DW.obj_n.isSetupComplete = true;

    // End of Start for MATLABSystem: '<S12>/Get Parameter10'

    // Start for MATLABSystem: '<S12>/Get Parameter11'
    dynamic_simulation_DW.obj_p.matlabCodegenIsDeleted = false;
    dynamic_simulation_DW.obj_p.isInitialized = 1;
    for (i = 0; i < 12; i++) {
      dynamic_simulation_B.cv2[i] = tmp_q[i];
    }

    dynamic_simulation_B.cv2[12] = '\x00';
    ParamGet_dynamic_simulation_134.initialize(dynamic_simulation_B.cv2);
    ParamGet_dynamic_simulation_134.initialize_error_codes(0, 1, 2, 3);
    ParamGet_dynamic_simulation_134.set_initial_value(0.0);
    dynamic_simulation_DW.obj_p.isSetupComplete = true;

    // End of Start for MATLABSystem: '<S12>/Get Parameter11'
    emxInitStruct_robotics_slmanip_(&dynamic_simulation_DW.obj);
    emxInitStruct_n_robotics_manip_(&dynamic_simulation_DW.gobj_1);
    emxInitStruct_n_robotics_manip_(&dynamic_simulation_DW.gobj_16);
    emxInitStruct_n_robotics_manip_(&dynamic_simulation_DW.gobj_15);
    emxInitStruct_n_robotics_manip_(&dynamic_simulation_DW.gobj_14);
    emxInitStruct_n_robotics_manip_(&dynamic_simulation_DW.gobj_13);
    emxInitStruct_n_robotics_manip_(&dynamic_simulation_DW.gobj_12);
    emxInitStruct_n_robotics_manip_(&dynamic_simulation_DW.gobj_11);
    emxInitStruct_n_robotics_manip_(&dynamic_simulation_DW.gobj_10);
    emxInitStruct_n_robotics_manip_(&dynamic_simulation_DW.gobj_9);
    emxInitStruct_n_robotics_manip_(&dynamic_simulation_DW.gobj_8);
    emxInitStruct_n_robotics_manip_(&dynamic_simulation_DW.gobj_7);
    emxInitStruct_n_robotics_manip_(&dynamic_simulation_DW.gobj_6);
    emxInitStruct_n_robotics_manip_(&dynamic_simulation_DW.gobj_5);
    emxInitStruct_n_robotics_manip_(&dynamic_simulation_DW.gobj_4);
    emxInitStruct_n_robotics_manip_(&dynamic_simulation_DW.gobj_3);
    emxInitStruct_n_robotics_manip_(&dynamic_simulation_DW.gobj_2);

    // Start for MATLABSystem: '<S11>/MATLAB System'
    dynamic_simulation_DW.obj.isInitialized = 0;
    dynamic_simulation_DW.obj.isInitialized = 1;
    dyn_RigidBodyTree_RigidBodyTree(&dynamic_simulation_DW.obj.TreeInternal,
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
  // Terminate for MATLABSystem: '<S10>/Get Parameter'
  matlabCodegenHandle_matlabCodeg(&dynamic_simulation_DW.obj_m);

  // Terminate for MATLABSystem: '<S10>/Get Parameter1'
  matlabCodegenHandle_matlabCodeg(&dynamic_simulation_DW.obj_c);

  // Terminate for MATLABSystem: '<S10>/Get Parameter2'
  matlabCodegenHandle_matlabCodeg(&dynamic_simulation_DW.obj_px);

  // Terminate for MATLABSystem: '<S10>/Get Parameter3'
  matlabCodegenHandle_matlabCodeg(&dynamic_simulation_DW.obj_nt);

  // Terminate for MATLABSystem: '<S10>/Get Parameter4'
  matlabCodegenHandle_matlabCodeg(&dynamic_simulation_DW.obj_o);

  // Terminate for MATLABSystem: '<S10>/Get Parameter5'
  matlabCodegenHandle_matlabCodeg(&dynamic_simulation_DW.obj_m4);

  // Terminate for Atomic SubSystem: '<Root>/Subscribe'
  // Terminate for MATLABSystem: '<S7>/SourceBlock'
  dynamic_simul_matlabCodegenHa_i(&dynamic_simulation_DW.obj_fk);

  // End of Terminate for SubSystem: '<Root>/Subscribe'

  // Terminate for MATLABSystem: '<S12>/Get Parameter12'
  matlabCodegenHandle_matlabCodeg(&dynamic_simulation_DW.obj_l);

  // Terminate for MATLABSystem: '<S12>/Get Parameter1'
  matlabCodegenHandle_matlabCodeg(&dynamic_simulation_DW.obj_lm);

  // Terminate for MATLABSystem: '<S12>/Get Parameter4'
  matlabCodegenHandle_matlabCodeg(&dynamic_simulation_DW.obj_a);

  // Terminate for MATLABSystem: '<S12>/Get Parameter5'
  matlabCodegenHandle_matlabCodeg(&dynamic_simulation_DW.obj_d);

  // Terminate for MATLABSystem: '<S12>/Get Parameter6'
  matlabCodegenHandle_matlabCodeg(&dynamic_simulation_DW.obj_h);

  // Terminate for MATLABSystem: '<S12>/Get Parameter7'
  matlabCodegenHandle_matlabCodeg(&dynamic_simulation_DW.obj_f);

  // Terminate for MATLABSystem: '<S12>/Get Parameter2'
  matlabCodegenHandle_matlabCodeg(&dynamic_simulation_DW.obj_e);

  // Terminate for MATLABSystem: '<S12>/Get Parameter3'
  matlabCodegenHandle_matlabCodeg(&dynamic_simulation_DW.obj_ez);

  // Terminate for MATLABSystem: '<S12>/Get Parameter8'
  matlabCodegenHandle_matlabCodeg(&dynamic_simulation_DW.obj_i);

  // Terminate for MATLABSystem: '<S12>/Get Parameter9'
  matlabCodegenHandle_matlabCodeg(&dynamic_simulation_DW.obj_ezd);

  // Terminate for MATLABSystem: '<S12>/Get Parameter10'
  matlabCodegenHandle_matlabCodeg(&dynamic_simulation_DW.obj_n);

  // Terminate for MATLABSystem: '<S12>/Get Parameter11'
  matlabCodegenHandle_matlabCodeg(&dynamic_simulation_DW.obj_p);
  emxFreeStruct_robotics_slmanip_(&dynamic_simulation_DW.obj);
  emxFreeStruct_n_robotics_manip_(&dynamic_simulation_DW.gobj_1);
  emxFreeStruct_n_robotics_manip_(&dynamic_simulation_DW.gobj_16);
  emxFreeStruct_n_robotics_manip_(&dynamic_simulation_DW.gobj_15);
  emxFreeStruct_n_robotics_manip_(&dynamic_simulation_DW.gobj_14);
  emxFreeStruct_n_robotics_manip_(&dynamic_simulation_DW.gobj_13);
  emxFreeStruct_n_robotics_manip_(&dynamic_simulation_DW.gobj_12);
  emxFreeStruct_n_robotics_manip_(&dynamic_simulation_DW.gobj_11);
  emxFreeStruct_n_robotics_manip_(&dynamic_simulation_DW.gobj_10);
  emxFreeStruct_n_robotics_manip_(&dynamic_simulation_DW.gobj_9);
  emxFreeStruct_n_robotics_manip_(&dynamic_simulation_DW.gobj_8);
  emxFreeStruct_n_robotics_manip_(&dynamic_simulation_DW.gobj_7);
  emxFreeStruct_n_robotics_manip_(&dynamic_simulation_DW.gobj_6);
  emxFreeStruct_n_robotics_manip_(&dynamic_simulation_DW.gobj_5);
  emxFreeStruct_n_robotics_manip_(&dynamic_simulation_DW.gobj_4);
  emxFreeStruct_n_robotics_manip_(&dynamic_simulation_DW.gobj_3);
  emxFreeStruct_n_robotics_manip_(&dynamic_simulation_DW.gobj_2);

  // Terminate for Atomic SubSystem: '<Root>/Publish'
  // Terminate for MATLABSystem: '<S4>/SinkBlock'
  matlabCodegenHandle_matl_kq2tzt(&dynamic_simulation_DW.obj_cd);

  // End of Terminate for SubSystem: '<Root>/Publish'

  // Terminate for Atomic SubSystem: '<Root>/Publish1'
  // Terminate for MATLABSystem: '<S5>/SinkBlock'
  matlabCodegenHandle_matl_kq2tzt(&dynamic_simulation_DW.obj_iv);

  // End of Terminate for SubSystem: '<Root>/Publish1'
}

//
// File trailer for generated code.
//
// [EOF]
//
