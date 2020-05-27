//
// Academic License - for use in teaching, academic research, and meeting
// course requirements at degree granting institutions only.  Not for
// government, commercial, or other organizational use.
//
// File: controller.cpp
//
// Code generated for Simulink model 'controller'.
//
// Model version                  : 1.35
// Simulink Coder version         : 9.3 (R2020a) 18-Nov-2019
// C/C++ source code generated on : Wed May 27 21:29:22 2020
//
// Target selection: ert.tlc
// Embedded hardware selection: Generic->Unspecified (assume 32-bit Generic)
// Code generation objectives: Unspecified
// Validation result: Not run
//
#include "controller.h"
#include "controller_private.h"

// Block signals (default storage)
B_controller_T controller_B;

// Block states (default storage)
DW_controller_T controller_DW;

// Real-time model
RT_MODEL_controller_T controller_M_ = RT_MODEL_controller_T();
RT_MODEL_controller_T *const controller_M = &controller_M_;

// Forward declaration for local functions
static void controller_SystemCore_step_j(boolean_T *varargout_1, real_T
  varargout_2_Positions[128], uint32_T *varargout_2_Positions_SL_Info_C,
  uint32_T *varargout_2_Positions_SL_Info_R, real_T varargout_2_Velocities[128],
  uint32_T *varargout_2_Velocities_SL_Info_, uint32_T
  *varargout_2_Velocities_SL_Inf_0, real_T varargout_2_Accelerations[128],
  uint32_T *varargout_2_Accelerations_SL_In, uint32_T
  *varargout_2_Accelerations_SL__0, real_T varargout_2_Effort[128], uint32_T
  *varargout_2_Effort_SL_Info_Curr, uint32_T *varargout_2_Effort_SL_Info_Rece,
  real_T *varargout_2_TimeFromStart_Sec, real_T *varargout_2_TimeFromStart_Nsec);
static void controller_SystemCore_step(boolean_T *varargout_1,
  SL_Bus_controller_std_msgs_String varargout_2_Name[16], uint32_T
  *varargout_2_Name_SL_Info_Curren, uint32_T *varargout_2_Name_SL_Info_Receiv,
  real_T varargout_2_Position[128], uint32_T *varargout_2_Position_SL_Info_Cu,
  uint32_T *varargout_2_Position_SL_Info_Re, real_T varargout_2_Velocity[128],
  uint32_T *varargout_2_Velocity_SL_Info_Cu, uint32_T
  *varargout_2_Velocity_SL_Info_Re, real_T varargout_2_Effort[128], uint32_T
  *varargout_2_Effort_SL_Info_Curr, uint32_T *varargout_2_Effort_SL_Info_Rece,
  SL_Bus_controller_std_msgs_Header *varargout_2_Header);
static void matlabCodegenHandle_matlabCod_j(ros_slros_internal_block_Subs_T *obj);
static void matlabCodegenHandle_matlabC_jmq(ros_slros_internal_block_GetP_T *obj);
static void matlabCodegenHandle_matlabCodeg(ros_slros_internal_block_Publ_T *obj);
static void controller_SystemCore_step_j(boolean_T *varargout_1, real_T
  varargout_2_Positions[128], uint32_T *varargout_2_Positions_SL_Info_C,
  uint32_T *varargout_2_Positions_SL_Info_R, real_T varargout_2_Velocities[128],
  uint32_T *varargout_2_Velocities_SL_Info_, uint32_T
  *varargout_2_Velocities_SL_Inf_0, real_T varargout_2_Accelerations[128],
  uint32_T *varargout_2_Accelerations_SL_In, uint32_T
  *varargout_2_Accelerations_SL__0, real_T varargout_2_Effort[128], uint32_T
  *varargout_2_Effort_SL_Info_Curr, uint32_T *varargout_2_Effort_SL_Info_Rece,
  real_T *varargout_2_TimeFromStart_Sec, real_T *varargout_2_TimeFromStart_Nsec)
{
  *varargout_1 = Sub_controller_136.getLatestMessage
    (&controller_B.b_varargout_2_m);
  *varargout_2_Positions_SL_Info_C =
    controller_B.b_varargout_2_m.Positions_SL_Info.CurrentLength;
  *varargout_2_Positions_SL_Info_R =
    controller_B.b_varargout_2_m.Positions_SL_Info.ReceivedLength;
  *varargout_2_Velocities_SL_Info_ =
    controller_B.b_varargout_2_m.Velocities_SL_Info.CurrentLength;
  *varargout_2_Velocities_SL_Inf_0 =
    controller_B.b_varargout_2_m.Velocities_SL_Info.ReceivedLength;
  *varargout_2_Accelerations_SL_In =
    controller_B.b_varargout_2_m.Accelerations_SL_Info.CurrentLength;
  *varargout_2_Accelerations_SL__0 =
    controller_B.b_varargout_2_m.Accelerations_SL_Info.ReceivedLength;
  memcpy(&varargout_2_Positions[0], &controller_B.b_varargout_2_m.Positions[0],
         sizeof(real_T) << 7U);
  memcpy(&varargout_2_Velocities[0], &controller_B.b_varargout_2_m.Velocities[0],
         sizeof(real_T) << 7U);
  memcpy(&varargout_2_Accelerations[0],
         &controller_B.b_varargout_2_m.Accelerations[0], sizeof(real_T) << 7U);
  memcpy(&varargout_2_Effort[0], &controller_B.b_varargout_2_m.Effort[0], sizeof
         (real_T) << 7U);
  *varargout_2_Effort_SL_Info_Curr =
    controller_B.b_varargout_2_m.Effort_SL_Info.CurrentLength;
  *varargout_2_Effort_SL_Info_Rece =
    controller_B.b_varargout_2_m.Effort_SL_Info.ReceivedLength;
  *varargout_2_TimeFromStart_Sec =
    controller_B.b_varargout_2_m.TimeFromStart.Sec;
  *varargout_2_TimeFromStart_Nsec =
    controller_B.b_varargout_2_m.TimeFromStart.Nsec;
}

static void controller_SystemCore_step(boolean_T *varargout_1,
  SL_Bus_controller_std_msgs_String varargout_2_Name[16], uint32_T
  *varargout_2_Name_SL_Info_Curren, uint32_T *varargout_2_Name_SL_Info_Receiv,
  real_T varargout_2_Position[128], uint32_T *varargout_2_Position_SL_Info_Cu,
  uint32_T *varargout_2_Position_SL_Info_Re, real_T varargout_2_Velocity[128],
  uint32_T *varargout_2_Velocity_SL_Info_Cu, uint32_T
  *varargout_2_Velocity_SL_Info_Re, real_T varargout_2_Effort[128], uint32_T
  *varargout_2_Effort_SL_Info_Curr, uint32_T *varargout_2_Effort_SL_Info_Rece,
  SL_Bus_controller_std_msgs_Header *varargout_2_Header)
{
  *varargout_1 = Sub_controller_5.getLatestMessage(&controller_B.b_varargout_2);
  memcpy(&varargout_2_Name[0], &controller_B.b_varargout_2.Name[0], sizeof
         (SL_Bus_controller_std_msgs_String) << 4U);
  *varargout_2_Name_SL_Info_Curren =
    controller_B.b_varargout_2.Name_SL_Info.CurrentLength;
  *varargout_2_Name_SL_Info_Receiv =
    controller_B.b_varargout_2.Name_SL_Info.ReceivedLength;
  *varargout_2_Position_SL_Info_Cu =
    controller_B.b_varargout_2.Position_SL_Info.CurrentLength;
  *varargout_2_Position_SL_Info_Re =
    controller_B.b_varargout_2.Position_SL_Info.ReceivedLength;
  *varargout_2_Velocity_SL_Info_Cu =
    controller_B.b_varargout_2.Velocity_SL_Info.CurrentLength;
  *varargout_2_Velocity_SL_Info_Re =
    controller_B.b_varargout_2.Velocity_SL_Info.ReceivedLength;
  memcpy(&varargout_2_Position[0], &controller_B.b_varargout_2.Position[0],
         sizeof(real_T) << 7U);
  memcpy(&varargout_2_Velocity[0], &controller_B.b_varargout_2.Velocity[0],
         sizeof(real_T) << 7U);
  memcpy(&varargout_2_Effort[0], &controller_B.b_varargout_2.Effort[0], sizeof
         (real_T) << 7U);
  *varargout_2_Effort_SL_Info_Curr =
    controller_B.b_varargout_2.Effort_SL_Info.CurrentLength;
  *varargout_2_Effort_SL_Info_Rece =
    controller_B.b_varargout_2.Effort_SL_Info.ReceivedLength;
  *varargout_2_Header = controller_B.b_varargout_2.Header;
}

static void matlabCodegenHandle_matlabCod_j(ros_slros_internal_block_Subs_T *obj)
{
  if (!obj->matlabCodegenIsDeleted) {
    obj->matlabCodegenIsDeleted = true;
  }
}

static void matlabCodegenHandle_matlabC_jmq(ros_slros_internal_block_GetP_T *obj)
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

// Model step function
void controller_step(void)
{
  uint32_T b_varargout_2_Positions_SL_Info;
  uint32_T b_varargout_2_Positions_SL_In_0;
  uint32_T b_varargout_2_Velocities_SL_Inf;
  uint32_T b_varargout_2_Velocities_SL_I_0;
  uint32_T b_varargout_2_Accelerations_SL_;
  uint32_T b_varargout_2_Accelerations_S_0;
  uint32_T b_varargout_2_Effort_SL_Info_Cu;
  uint32_T b_varargout_2_Effort_SL_Info_Re;
  boolean_T b_varargout_1;
  int32_T i;
  real_T tmp;
  real_T tmp_0;
  real_T tmp_1;

  // Outputs for Atomic SubSystem: '<Root>/Subscribe1'
  // MATLABSystem: '<S5>/SourceBlock' incorporates:
  //   Inport: '<S9>/In1'

  controller_SystemCore_step_j(&b_varargout_1,
    controller_B.b_varargout_2_Positions, &b_varargout_2_Positions_SL_Info,
    &b_varargout_2_Positions_SL_In_0, controller_B.b_varargout_2_Velocities,
    &b_varargout_2_Velocities_SL_Inf, &b_varargout_2_Velocities_SL_I_0,
    controller_B.b_varargout_2_Accelerations, &b_varargout_2_Accelerations_SL_,
    &b_varargout_2_Accelerations_S_0, controller_B.b_varargout_2_Effort,
    &b_varargout_2_Effort_SL_Info_Cu, &b_varargout_2_Effort_SL_Info_Re,
    &controller_B.b_varargout_2_TimeFromStart_Sec,
    &controller_B.b_varargout_2_TimeFromStart_Nse);

  // Outputs for Enabled SubSystem: '<S5>/Enabled Subsystem' incorporates:
  //   EnablePort: '<S9>/Enable'

  if (b_varargout_1) {
    controller_B.In1_l.Positions_SL_Info.CurrentLength =
      b_varargout_2_Positions_SL_Info;
    controller_B.In1_l.Positions_SL_Info.ReceivedLength =
      b_varargout_2_Positions_SL_In_0;
    controller_B.In1_l.Velocities_SL_Info.CurrentLength =
      b_varargout_2_Velocities_SL_Inf;
    controller_B.In1_l.Velocities_SL_Info.ReceivedLength =
      b_varargout_2_Velocities_SL_I_0;
    controller_B.In1_l.Accelerations_SL_Info.CurrentLength =
      b_varargout_2_Accelerations_SL_;
    controller_B.In1_l.Accelerations_SL_Info.ReceivedLength =
      b_varargout_2_Accelerations_S_0;
    memcpy(&controller_B.In1_l.Positions[0],
           &controller_B.b_varargout_2_Positions[0], sizeof(real_T) << 7U);
    memcpy(&controller_B.In1_l.Velocities[0],
           &controller_B.b_varargout_2_Velocities[0], sizeof(real_T) << 7U);
    memcpy(&controller_B.In1_l.Accelerations[0],
           &controller_B.b_varargout_2_Accelerations[0], sizeof(real_T) << 7U);
    memcpy(&controller_B.In1_l.Effort[0], &controller_B.b_varargout_2_Effort[0],
           sizeof(real_T) << 7U);
    controller_B.In1_l.Effort_SL_Info.CurrentLength =
      b_varargout_2_Effort_SL_Info_Cu;
    controller_B.In1_l.Effort_SL_Info.ReceivedLength =
      b_varargout_2_Effort_SL_Info_Re;
    controller_B.In1_l.TimeFromStart.Sec =
      controller_B.b_varargout_2_TimeFromStart_Sec;
    controller_B.In1_l.TimeFromStart.Nsec =
      controller_B.b_varargout_2_TimeFromStart_Nse;
  }

  // End of MATLABSystem: '<S5>/SourceBlock'
  // End of Outputs for SubSystem: '<S5>/Enabled Subsystem'
  // End of Outputs for SubSystem: '<Root>/Subscribe1'

  // Outputs for Atomic SubSystem: '<Root>/Subscribe'
  // MATLABSystem: '<S4>/SourceBlock' incorporates:
  //   Inport: '<S8>/In1'

  controller_SystemCore_step(&b_varargout_1, controller_B.b_varargout_2_Name,
    &b_varargout_2_Positions_SL_Info, &b_varargout_2_Positions_SL_In_0,
    controller_B.b_varargout_2_Positions, &b_varargout_2_Velocities_SL_Inf,
    &b_varargout_2_Velocities_SL_I_0, controller_B.b_varargout_2_Velocities,
    &b_varargout_2_Accelerations_SL_, &b_varargout_2_Accelerations_S_0,
    controller_B.b_varargout_2_Effort, &b_varargout_2_Effort_SL_Info_Cu,
    &b_varargout_2_Effort_SL_Info_Re, &controller_B.b_varargout_2_Header);

  // Outputs for Enabled SubSystem: '<S4>/Enabled Subsystem' incorporates:
  //   EnablePort: '<S8>/Enable'

  if (b_varargout_1) {
    memcpy(&controller_B.In1.Name[0], &controller_B.b_varargout_2_Name[0],
           sizeof(SL_Bus_controller_std_msgs_String) << 4U);
    controller_B.In1.Name_SL_Info.CurrentLength =
      b_varargout_2_Positions_SL_Info;
    controller_B.In1.Name_SL_Info.ReceivedLength =
      b_varargout_2_Positions_SL_In_0;
    controller_B.In1.Position_SL_Info.CurrentLength =
      b_varargout_2_Velocities_SL_Inf;
    controller_B.In1.Position_SL_Info.ReceivedLength =
      b_varargout_2_Velocities_SL_I_0;
    controller_B.In1.Velocity_SL_Info.CurrentLength =
      b_varargout_2_Accelerations_SL_;
    controller_B.In1.Velocity_SL_Info.ReceivedLength =
      b_varargout_2_Accelerations_S_0;
    memcpy(&controller_B.In1.Position[0], &controller_B.b_varargout_2_Positions
           [0], sizeof(real_T) << 7U);
    memcpy(&controller_B.In1.Velocity[0],
           &controller_B.b_varargout_2_Velocities[0], sizeof(real_T) << 7U);
    memcpy(&controller_B.In1.Effort[0], &controller_B.b_varargout_2_Effort[0],
           sizeof(real_T) << 7U);
    controller_B.In1.Effort_SL_Info.CurrentLength =
      b_varargout_2_Effort_SL_Info_Cu;
    controller_B.In1.Effort_SL_Info.ReceivedLength =
      b_varargout_2_Effort_SL_Info_Re;
    controller_B.In1.Header = controller_B.b_varargout_2_Header;
  }

  // End of MATLABSystem: '<S4>/SourceBlock'
  // End of Outputs for SubSystem: '<S4>/Enabled Subsystem'
  // End of Outputs for SubSystem: '<Root>/Subscribe'

  // Sum: '<Root>/Add1'
  for (i = 0; i < 6; i++) {
    controller_B.Add[i] = controller_B.In1_l.Positions[i] -
      controller_B.In1.Position[i];
  }

  // End of Sum: '<Root>/Add1'

  // MATLABSystem: '<S6>/Get Parameter'
  ParamGet_controller_93.get_parameter
    (&controller_B.b_varargout_2_TimeFromStart_Sec);

  // MATLABSystem: '<S6>/Get Parameter1'
  ParamGet_controller_94.get_parameter
    (&controller_B.b_varargout_2_TimeFromStart_Nse);

  // MATLABSystem: '<S6>/Get Parameter2'
  ParamGet_controller_95.get_parameter(&controller_B.value);

  // MATLABSystem: '<S6>/Get Parameter3'
  ParamGet_controller_96.get_parameter(&controller_B.value_c);

  // MATLABSystem: '<S6>/Get Parameter4'
  ParamGet_controller_97.get_parameter(&controller_B.value_k);

  // MATLABSystem: '<S6>/Get Parameter5'
  ParamGet_controller_98.get_parameter(&controller_B.value_cx);

  // Product: '<Root>/MatrixMultiply' incorporates:
  //   MATLABSystem: '<S6>/Get Parameter'
  //   MATLABSystem: '<S6>/Get Parameter1'
  //   MATLABSystem: '<S6>/Get Parameter2'
  //   MATLABSystem: '<S6>/Get Parameter3'
  //   MATLABSystem: '<S6>/Get Parameter4'
  //   MATLABSystem: '<S6>/Get Parameter5'

  controller_B.d = controller_B.Add[0];
  controller_B.d1 = controller_B.Add[1];
  controller_B.d2 = controller_B.Add[2];
  tmp = controller_B.Add[3];
  tmp_0 = controller_B.Add[4];
  tmp_1 = controller_B.Add[5];
  controller_B.Add[0] = controller_B.d *
    controller_B.b_varargout_2_TimeFromStart_Sec;
  controller_B.Add[1] = controller_B.d1 *
    controller_B.b_varargout_2_TimeFromStart_Nse;
  controller_B.Add[2] = controller_B.d2 * controller_B.value;
  controller_B.Add[3] = tmp * controller_B.value_c;
  controller_B.Add[4] = tmp_0 * controller_B.value_k;
  controller_B.Add[5] = tmp_1 * controller_B.value_cx;

  // Sum: '<Root>/Add'
  for (i = 0; i < 6; i++) {
    controller_B.Add[i] -= controller_B.In1.Velocity[i];
  }

  // End of Sum: '<Root>/Add'

  // MATLABSystem: '<S7>/Get Parameter'
  ParamGet_controller_103.get_parameter
    (&controller_B.b_varargout_2_TimeFromStart_Sec);

  // MATLABSystem: '<S7>/Get Parameter1'
  ParamGet_controller_104.get_parameter
    (&controller_B.b_varargout_2_TimeFromStart_Nse);

  // MATLABSystem: '<S7>/Get Parameter2'
  ParamGet_controller_105.get_parameter(&controller_B.value);

  // MATLABSystem: '<S7>/Get Parameter3'
  ParamGet_controller_106.get_parameter(&controller_B.value_c);

  // MATLABSystem: '<S7>/Get Parameter4'
  ParamGet_controller_107.get_parameter(&controller_B.value_k);

  // MATLABSystem: '<S7>/Get Parameter5'
  ParamGet_controller_108.get_parameter(&controller_B.value_cx);

  // Product: '<Root>/MatrixMultiply1' incorporates:
  //   MATLABSystem: '<S7>/Get Parameter'
  //   MATLABSystem: '<S7>/Get Parameter1'
  //   MATLABSystem: '<S7>/Get Parameter2'
  //   MATLABSystem: '<S7>/Get Parameter3'
  //   MATLABSystem: '<S7>/Get Parameter4'
  //   MATLABSystem: '<S7>/Get Parameter5'

  controller_B.MatrixMultiply1[0] = controller_B.Add[0] *
    controller_B.b_varargout_2_TimeFromStart_Sec;
  controller_B.MatrixMultiply1[1] = controller_B.Add[1] *
    controller_B.b_varargout_2_TimeFromStart_Nse;
  controller_B.MatrixMultiply1[2] = controller_B.Add[2] * controller_B.value;
  controller_B.MatrixMultiply1[3] = controller_B.Add[3] * controller_B.value_c;
  controller_B.MatrixMultiply1[4] = controller_B.Add[4] * controller_B.value_k;
  controller_B.MatrixMultiply1[5] = controller_B.Add[5] * controller_B.value_cx;

  // MATLAB Function: '<Root>/MATLAB Function' incorporates:
  //   Constant: '<S1>/Constant'

  controller_B.msg = controller_P.Constant_Value_d;
  for (i = 0; i < 6; i++) {
    controller_B.msg.Data[i] = controller_B.MatrixMultiply1[i];
  }

  controller_B.msg.Data_SL_Info.CurrentLength = 6U;
  controller_B.msg.Layout.Dim_SL_Info.CurrentLength = 1U;
  controller_B.msg.Layout.Dim[0].Size = 6U;
  controller_B.msg.Layout.Dim[0].Stride = 6U;

  // End of MATLAB Function: '<Root>/MATLAB Function'

  // Outputs for Atomic SubSystem: '<Root>/Publish'
  // MATLABSystem: '<S3>/SinkBlock'
  Pub_controller_4.publish(&controller_B.msg);

  // End of Outputs for SubSystem: '<Root>/Publish'
}

// Model initialize function
void controller_initialize(void)
{
  {
    char_T tmp[18];
    char_T tmp_0[14];
    char_T tmp_1[15];
    int32_T i;
    static const char_T tmp_2[17] = { '/', 'j', 'o', 'i', 'n', 't', '_', 't',
      'r', 'a', 'j', 'e', 'c', 't', 'o', 'r', 'y' };

    static const char_T tmp_3[13] = { '/', 'j', 'o', 'i', 'n', 't', '_', 's',
      't', 'a', 't', 'e', 's' };

    static const char_T tmp_4[13] = { '/', 'j', 'o', 'i', 'n', 't', '_', 't',
      'o', 'r', 'q', 'u', 'e' };

    static const char_T tmp_5[14] = { '/', 'p', 'o', 's', '_', 'c', 'o', 'n',
      't', 'r', 'o', 'l', '_', '1' };

    static const char_T tmp_6[14] = { '/', 'p', 'o', 's', '_', 'c', 'o', 'n',
      't', 'r', 'o', 'l', '_', '2' };

    static const char_T tmp_7[14] = { '/', 'p', 'o', 's', '_', 'c', 'o', 'n',
      't', 'r', 'o', 'l', '_', '3' };

    static const char_T tmp_8[14] = { '/', 'p', 'o', 's', '_', 'c', 'o', 'n',
      't', 'r', 'o', 'l', '_', '4' };

    static const char_T tmp_9[14] = { '/', 'p', 'o', 's', '_', 'c', 'o', 'n',
      't', 'r', 'o', 'l', '_', '5' };

    static const char_T tmp_a[14] = { '/', 'p', 'o', 's', '_', 'c', 'o', 'n',
      't', 'r', 'o', 'l', '_', '6' };

    static const char_T tmp_b[14] = { '/', 'v', 'e', 'l', '_', 'c', 'o', 'n',
      't', 'r', 'o', 'l', '_', '1' };

    static const char_T tmp_c[14] = { '/', 'v', 'e', 'l', '_', 'c', 'o', 'n',
      't', 'r', 'o', 'l', '_', '2' };

    static const char_T tmp_d[14] = { '/', 'v', 'e', 'l', '_', 'c', 'o', 'n',
      't', 'r', 'o', 'l', '_', '3' };

    static const char_T tmp_e[14] = { '/', 'v', 'e', 'l', '_', 'c', 'o', 'n',
      't', 'r', 'o', 'l', '_', '4' };

    static const char_T tmp_f[14] = { '/', 'v', 'e', 'l', '_', 'c', 'o', 'n',
      't', 'r', 'o', 'l', '_', '5' };

    static const char_T tmp_g[14] = { '/', 'v', 'e', 'l', '_', 'c', 'o', 'n',
      't', 'r', 'o', 'l', '_', '6' };

    // SystemInitialize for Atomic SubSystem: '<Root>/Subscribe1'
    // SystemInitialize for Enabled SubSystem: '<S5>/Enabled Subsystem'
    // SystemInitialize for Outport: '<S9>/Out1'
    controller_B.In1_l = controller_P.Out1_Y0_d;

    // End of SystemInitialize for SubSystem: '<S5>/Enabled Subsystem'

    // Start for MATLABSystem: '<S5>/SourceBlock'
    controller_DW.obj_ez.matlabCodegenIsDeleted = false;
    controller_DW.obj_ez.isInitialized = 1;
    for (i = 0; i < 17; i++) {
      tmp[i] = tmp_2[i];
    }

    tmp[17] = '\x00';
    Sub_controller_136.createSubscriber(tmp, 1);
    controller_DW.obj_ez.isSetupComplete = true;

    // End of Start for MATLABSystem: '<S5>/SourceBlock'
    // End of SystemInitialize for SubSystem: '<Root>/Subscribe1'

    // SystemInitialize for Atomic SubSystem: '<Root>/Subscribe'
    // SystemInitialize for Enabled SubSystem: '<S4>/Enabled Subsystem'
    // SystemInitialize for Outport: '<S8>/Out1'
    controller_B.In1 = controller_P.Out1_Y0;

    // End of SystemInitialize for SubSystem: '<S4>/Enabled Subsystem'

    // Start for MATLABSystem: '<S4>/SourceBlock'
    controller_DW.obj_lb.matlabCodegenIsDeleted = false;
    controller_DW.obj_lb.isInitialized = 1;
    for (i = 0; i < 13; i++) {
      tmp_0[i] = tmp_3[i];
    }

    tmp_0[13] = '\x00';
    Sub_controller_5.createSubscriber(tmp_0, 1);
    controller_DW.obj_lb.isSetupComplete = true;

    // End of Start for MATLABSystem: '<S4>/SourceBlock'
    // End of SystemInitialize for SubSystem: '<Root>/Subscribe'

    // SystemInitialize for Atomic SubSystem: '<Root>/Publish'
    // Start for MATLABSystem: '<S3>/SinkBlock'
    controller_DW.obj_b.matlabCodegenIsDeleted = false;
    controller_DW.obj_b.isInitialized = 1;
    for (i = 0; i < 13; i++) {
      tmp_0[i] = tmp_4[i];
    }

    tmp_0[13] = '\x00';
    Pub_controller_4.createPublisher(tmp_0, 1);
    controller_DW.obj_b.isSetupComplete = true;

    // End of Start for MATLABSystem: '<S3>/SinkBlock'
    // End of SystemInitialize for SubSystem: '<Root>/Publish'

    // Start for MATLABSystem: '<S6>/Get Parameter'
    controller_DW.obj_j.matlabCodegenIsDeleted = false;
    controller_DW.obj_j.isInitialized = 1;
    for (i = 0; i < 14; i++) {
      tmp_1[i] = tmp_5[i];
    }

    tmp_1[14] = '\x00';
    ParamGet_controller_93.initialize(tmp_1);
    ParamGet_controller_93.initialize_error_codes(0, 1, 2, 3);
    ParamGet_controller_93.set_initial_value(0.2);
    controller_DW.obj_j.isSetupComplete = true;

    // End of Start for MATLABSystem: '<S6>/Get Parameter'

    // Start for MATLABSystem: '<S6>/Get Parameter1'
    controller_DW.obj_l.matlabCodegenIsDeleted = false;
    controller_DW.obj_l.isInitialized = 1;
    for (i = 0; i < 14; i++) {
      tmp_1[i] = tmp_6[i];
    }

    tmp_1[14] = '\x00';
    ParamGet_controller_94.initialize(tmp_1);
    ParamGet_controller_94.initialize_error_codes(0, 1, 2, 3);
    ParamGet_controller_94.set_initial_value(0.2);
    controller_DW.obj_l.isSetupComplete = true;

    // End of Start for MATLABSystem: '<S6>/Get Parameter1'

    // Start for MATLABSystem: '<S6>/Get Parameter2'
    controller_DW.obj_nc.matlabCodegenIsDeleted = false;
    controller_DW.obj_nc.isInitialized = 1;
    for (i = 0; i < 14; i++) {
      tmp_1[i] = tmp_7[i];
    }

    tmp_1[14] = '\x00';
    ParamGet_controller_95.initialize(tmp_1);
    ParamGet_controller_95.initialize_error_codes(0, 1, 2, 3);
    ParamGet_controller_95.set_initial_value(0.2);
    controller_DW.obj_nc.isSetupComplete = true;

    // End of Start for MATLABSystem: '<S6>/Get Parameter2'

    // Start for MATLABSystem: '<S6>/Get Parameter3'
    controller_DW.obj_e.matlabCodegenIsDeleted = false;
    controller_DW.obj_e.isInitialized = 1;
    for (i = 0; i < 14; i++) {
      tmp_1[i] = tmp_8[i];
    }

    tmp_1[14] = '\x00';
    ParamGet_controller_96.initialize(tmp_1);
    ParamGet_controller_96.initialize_error_codes(0, 1, 2, 3);
    ParamGet_controller_96.set_initial_value(0.2);
    controller_DW.obj_e.isSetupComplete = true;

    // End of Start for MATLABSystem: '<S6>/Get Parameter3'

    // Start for MATLABSystem: '<S6>/Get Parameter4'
    controller_DW.obj_j1.matlabCodegenIsDeleted = false;
    controller_DW.obj_j1.isInitialized = 1;
    for (i = 0; i < 14; i++) {
      tmp_1[i] = tmp_9[i];
    }

    tmp_1[14] = '\x00';
    ParamGet_controller_97.initialize(tmp_1);
    ParamGet_controller_97.initialize_error_codes(0, 1, 2, 3);
    ParamGet_controller_97.set_initial_value(0.2);
    controller_DW.obj_j1.isSetupComplete = true;

    // End of Start for MATLABSystem: '<S6>/Get Parameter4'

    // Start for MATLABSystem: '<S6>/Get Parameter5'
    controller_DW.obj_f.matlabCodegenIsDeleted = false;
    controller_DW.obj_f.isInitialized = 1;
    for (i = 0; i < 14; i++) {
      tmp_1[i] = tmp_a[i];
    }

    tmp_1[14] = '\x00';
    ParamGet_controller_98.initialize(tmp_1);
    ParamGet_controller_98.initialize_error_codes(0, 1, 2, 3);
    ParamGet_controller_98.set_initial_value(0.2);
    controller_DW.obj_f.isSetupComplete = true;

    // End of Start for MATLABSystem: '<S6>/Get Parameter5'

    // Start for MATLABSystem: '<S7>/Get Parameter'
    controller_DW.obj.matlabCodegenIsDeleted = false;
    controller_DW.obj.isInitialized = 1;
    for (i = 0; i < 14; i++) {
      tmp_1[i] = tmp_b[i];
    }

    tmp_1[14] = '\x00';
    ParamGet_controller_103.initialize(tmp_1);
    ParamGet_controller_103.initialize_error_codes(0, 1, 2, 3);
    ParamGet_controller_103.set_initial_value(0.1);
    controller_DW.obj.isSetupComplete = true;

    // End of Start for MATLABSystem: '<S7>/Get Parameter'

    // Start for MATLABSystem: '<S7>/Get Parameter1'
    controller_DW.obj_k.matlabCodegenIsDeleted = false;
    controller_DW.obj_k.isInitialized = 1;
    for (i = 0; i < 14; i++) {
      tmp_1[i] = tmp_c[i];
    }

    tmp_1[14] = '\x00';
    ParamGet_controller_104.initialize(tmp_1);
    ParamGet_controller_104.initialize_error_codes(0, 1, 2, 3);
    ParamGet_controller_104.set_initial_value(0.1);
    controller_DW.obj_k.isSetupComplete = true;

    // End of Start for MATLABSystem: '<S7>/Get Parameter1'

    // Start for MATLABSystem: '<S7>/Get Parameter2'
    controller_DW.obj_a.matlabCodegenIsDeleted = false;
    controller_DW.obj_a.isInitialized = 1;
    for (i = 0; i < 14; i++) {
      tmp_1[i] = tmp_d[i];
    }

    tmp_1[14] = '\x00';
    ParamGet_controller_105.initialize(tmp_1);
    ParamGet_controller_105.initialize_error_codes(0, 1, 2, 3);
    ParamGet_controller_105.set_initial_value(0.1);
    controller_DW.obj_a.isSetupComplete = true;

    // End of Start for MATLABSystem: '<S7>/Get Parameter2'

    // Start for MATLABSystem: '<S7>/Get Parameter3'
    controller_DW.obj_n.matlabCodegenIsDeleted = false;
    controller_DW.obj_n.isInitialized = 1;
    for (i = 0; i < 14; i++) {
      tmp_1[i] = tmp_e[i];
    }

    tmp_1[14] = '\x00';
    ParamGet_controller_106.initialize(tmp_1);
    ParamGet_controller_106.initialize_error_codes(0, 1, 2, 3);
    ParamGet_controller_106.set_initial_value(0.01);
    controller_DW.obj_n.isSetupComplete = true;

    // End of Start for MATLABSystem: '<S7>/Get Parameter3'

    // Start for MATLABSystem: '<S7>/Get Parameter4'
    controller_DW.obj_nr.matlabCodegenIsDeleted = false;
    controller_DW.obj_nr.isInitialized = 1;
    for (i = 0; i < 14; i++) {
      tmp_1[i] = tmp_f[i];
    }

    tmp_1[14] = '\x00';
    ParamGet_controller_107.initialize(tmp_1);
    ParamGet_controller_107.initialize_error_codes(0, 1, 2, 3);
    ParamGet_controller_107.set_initial_value(0.1);
    controller_DW.obj_nr.isSetupComplete = true;

    // End of Start for MATLABSystem: '<S7>/Get Parameter4'

    // Start for MATLABSystem: '<S7>/Get Parameter5'
    controller_DW.obj_as.matlabCodegenIsDeleted = false;
    controller_DW.obj_as.isInitialized = 1;
    for (i = 0; i < 14; i++) {
      tmp_1[i] = tmp_g[i];
    }

    tmp_1[14] = '\x00';
    ParamGet_controller_108.initialize(tmp_1);
    ParamGet_controller_108.initialize_error_codes(0, 1, 2, 3);
    ParamGet_controller_108.set_initial_value(0.0001);
    controller_DW.obj_as.isSetupComplete = true;

    // End of Start for MATLABSystem: '<S7>/Get Parameter5'
  }
}

// Model terminate function
void controller_terminate(void)
{
  // Terminate for Atomic SubSystem: '<Root>/Subscribe1'
  // Terminate for MATLABSystem: '<S5>/SourceBlock'
  matlabCodegenHandle_matlabCod_j(&controller_DW.obj_ez);

  // End of Terminate for SubSystem: '<Root>/Subscribe1'

  // Terminate for Atomic SubSystem: '<Root>/Subscribe'
  // Terminate for MATLABSystem: '<S4>/SourceBlock'
  matlabCodegenHandle_matlabCod_j(&controller_DW.obj_lb);

  // End of Terminate for SubSystem: '<Root>/Subscribe'

  // Terminate for MATLABSystem: '<S6>/Get Parameter'
  matlabCodegenHandle_matlabC_jmq(&controller_DW.obj_j);

  // Terminate for MATLABSystem: '<S6>/Get Parameter1'
  matlabCodegenHandle_matlabC_jmq(&controller_DW.obj_l);

  // Terminate for MATLABSystem: '<S6>/Get Parameter2'
  matlabCodegenHandle_matlabC_jmq(&controller_DW.obj_nc);

  // Terminate for MATLABSystem: '<S6>/Get Parameter3'
  matlabCodegenHandle_matlabC_jmq(&controller_DW.obj_e);

  // Terminate for MATLABSystem: '<S6>/Get Parameter4'
  matlabCodegenHandle_matlabC_jmq(&controller_DW.obj_j1);

  // Terminate for MATLABSystem: '<S6>/Get Parameter5'
  matlabCodegenHandle_matlabC_jmq(&controller_DW.obj_f);

  // Terminate for MATLABSystem: '<S7>/Get Parameter'
  matlabCodegenHandle_matlabC_jmq(&controller_DW.obj);

  // Terminate for MATLABSystem: '<S7>/Get Parameter1'
  matlabCodegenHandle_matlabC_jmq(&controller_DW.obj_k);

  // Terminate for MATLABSystem: '<S7>/Get Parameter2'
  matlabCodegenHandle_matlabC_jmq(&controller_DW.obj_a);

  // Terminate for MATLABSystem: '<S7>/Get Parameter3'
  matlabCodegenHandle_matlabC_jmq(&controller_DW.obj_n);

  // Terminate for MATLABSystem: '<S7>/Get Parameter4'
  matlabCodegenHandle_matlabC_jmq(&controller_DW.obj_nr);

  // Terminate for MATLABSystem: '<S7>/Get Parameter5'
  matlabCodegenHandle_matlabC_jmq(&controller_DW.obj_as);

  // Terminate for Atomic SubSystem: '<Root>/Publish'
  // Terminate for MATLABSystem: '<S3>/SinkBlock'
  matlabCodegenHandle_matlabCodeg(&controller_DW.obj_b);

  // End of Terminate for SubSystem: '<Root>/Publish'
}

//
// File trailer for generated code.
//
// [EOF]
//
