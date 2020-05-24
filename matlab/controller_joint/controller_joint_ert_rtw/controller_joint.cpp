//
// Academic License - for use in teaching, academic research, and meeting
// course requirements at degree granting institutions only.  Not for
// government, commercial, or other organizational use.
//
// File: controller_joint.cpp
//
// Code generated for Simulink model 'controller_joint'.
//
// Model version                  : 1.25
// Simulink Coder version         : 9.3 (R2020a) 18-Nov-2019
// C/C++ source code generated on : Sun May 24 21:25:14 2020
//
// Target selection: ert.tlc
// Embedded hardware selection: Generic->Unspecified (assume 32-bit Generic)
// Code generation objectives: Unspecified
// Validation result: Not run
//
#include "controller_joint.h"
#include "controller_joint_private.h"

// Block signals (default storage)
B_controller_joint_T controller_joint_B;

// Block states (default storage)
DW_controller_joint_T controller_joint_DW;

// Real-time model
RT_MODEL_controller_joint_T controller_joint_M_ = RT_MODEL_controller_joint_T();
RT_MODEL_controller_joint_T *const controller_joint_M = &controller_joint_M_;

// Forward declaration for local functions
static void controller_join_SystemCore_step(boolean_T *varargout_1,
  SL_Bus_controller_joint_std_msgs_String varargout_2_Name[16], uint32_T
  *varargout_2_Name_SL_Info_Curren, uint32_T *varargout_2_Name_SL_Info_Receiv,
  real_T varargout_2_Position[128], uint32_T *varargout_2_Position_SL_Info_Cu,
  uint32_T *varargout_2_Position_SL_Info_Re, real_T varargout_2_Velocity[128],
  uint32_T *varargout_2_Velocity_SL_Info_Cu, uint32_T
  *varargout_2_Velocity_SL_Info_Re, real_T varargout_2_Effort[128], uint32_T
  *varargout_2_Effort_SL_Info_Curr, uint32_T *varargout_2_Effort_SL_Info_Rece,
  SL_Bus_controller_joint_std_msgs_Header *varargout_2_Header);
static void matlabCodegenHandle_matlabCo_nl(ros_slros_internal_block_GetP_T *obj);
static void matlabCodegenHandle_matlabCod_n(ros_slros_internal_block_Subs_T *obj);
static void matlabCodegenHandle_matlabCodeg(ros_slros_internal_block_Publ_T *obj);
static void controller_join_SystemCore_step(boolean_T *varargout_1,
  SL_Bus_controller_joint_std_msgs_String varargout_2_Name[16], uint32_T
  *varargout_2_Name_SL_Info_Curren, uint32_T *varargout_2_Name_SL_Info_Receiv,
  real_T varargout_2_Position[128], uint32_T *varargout_2_Position_SL_Info_Cu,
  uint32_T *varargout_2_Position_SL_Info_Re, real_T varargout_2_Velocity[128],
  uint32_T *varargout_2_Velocity_SL_Info_Cu, uint32_T
  *varargout_2_Velocity_SL_Info_Re, real_T varargout_2_Effort[128], uint32_T
  *varargout_2_Effort_SL_Info_Curr, uint32_T *varargout_2_Effort_SL_Info_Rece,
  SL_Bus_controller_joint_std_msgs_Header *varargout_2_Header)
{
  *varargout_1 = Sub_controller_joint_5.getLatestMessage
    (&controller_joint_B.b_varargout_2);
  memcpy(&varargout_2_Name[0], &controller_joint_B.b_varargout_2.Name[0], sizeof
         (SL_Bus_controller_joint_std_msgs_String) << 4U);
  *varargout_2_Name_SL_Info_Curren =
    controller_joint_B.b_varargout_2.Name_SL_Info.CurrentLength;
  *varargout_2_Name_SL_Info_Receiv =
    controller_joint_B.b_varargout_2.Name_SL_Info.ReceivedLength;
  *varargout_2_Position_SL_Info_Cu =
    controller_joint_B.b_varargout_2.Position_SL_Info.CurrentLength;
  *varargout_2_Position_SL_Info_Re =
    controller_joint_B.b_varargout_2.Position_SL_Info.ReceivedLength;
  *varargout_2_Velocity_SL_Info_Cu =
    controller_joint_B.b_varargout_2.Velocity_SL_Info.CurrentLength;
  *varargout_2_Velocity_SL_Info_Re =
    controller_joint_B.b_varargout_2.Velocity_SL_Info.ReceivedLength;
  memcpy(&varargout_2_Position[0], &controller_joint_B.b_varargout_2.Position[0],
         sizeof(real_T) << 7U);
  memcpy(&varargout_2_Velocity[0], &controller_joint_B.b_varargout_2.Velocity[0],
         sizeof(real_T) << 7U);
  memcpy(&varargout_2_Effort[0], &controller_joint_B.b_varargout_2.Effort[0],
         sizeof(real_T) << 7U);
  *varargout_2_Effort_SL_Info_Curr =
    controller_joint_B.b_varargout_2.Effort_SL_Info.CurrentLength;
  *varargout_2_Effort_SL_Info_Rece =
    controller_joint_B.b_varargout_2.Effort_SL_Info.ReceivedLength;
  *varargout_2_Header = controller_joint_B.b_varargout_2.Header;
}

static void matlabCodegenHandle_matlabCo_nl(ros_slros_internal_block_GetP_T *obj)
{
  if (!obj->matlabCodegenIsDeleted) {
    obj->matlabCodegenIsDeleted = true;
  }
}

static void matlabCodegenHandle_matlabCod_n(ros_slros_internal_block_Subs_T *obj)
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
void controller_joint_step(void)
{
  int32_T i;
  real_T value;
  real_T value_0;
  real_T value_1;
  uint32_T b_varargout_2_Name_SL_Info_Curr;
  uint32_T b_varargout_2_Name_SL_Info_Rece;
  uint32_T b_varargout_2_Position_SL_Info_;
  uint32_T b_varargout_2_Position_SL_Inf_0;
  uint32_T b_varargout_2_Velocity_SL_Info_;
  uint32_T b_varargout_2_Velocity_SL_Inf_0;
  uint32_T b_varargout_2_Effort_SL_Info_Cu;
  uint32_T b_varargout_2_Effort_SL_Info_Re;
  boolean_T b_varargout_1;

  // MATLABSystem: '<S7>/Get Parameter'
  ParamGet_controller_joint_115.get_parameter(&controller_joint_B.value);

  // MATLABSystem: '<S7>/Get Parameter1'
  ParamGet_controller_joint_116.get_parameter(&controller_joint_B.value_m);

  // MATLABSystem: '<S7>/Get Parameter2'
  ParamGet_controller_joint_117.get_parameter(&controller_joint_B.value_c);

  // MATLABSystem: '<S7>/Get Parameter3'
  ParamGet_controller_joint_118.get_parameter(&controller_joint_B.value_k);

  // MATLABSystem: '<S7>/Get Parameter4'
  ParamGet_controller_joint_119.get_parameter(&controller_joint_B.value_cx);

  // MATLABSystem: '<S7>/Get Parameter5'
  ParamGet_controller_joint_120.get_parameter(&controller_joint_B.value_b);

  // Outputs for Atomic SubSystem: '<Root>/Subscribe'
  // MATLABSystem: '<S4>/SourceBlock' incorporates:
  //   Inport: '<S8>/In1'

  controller_join_SystemCore_step(&b_varargout_1,
    controller_joint_B.b_varargout_2_Name, &b_varargout_2_Name_SL_Info_Curr,
    &b_varargout_2_Name_SL_Info_Rece, controller_joint_B.b_varargout_2_Position,
    &b_varargout_2_Position_SL_Info_, &b_varargout_2_Position_SL_Inf_0,
    controller_joint_B.b_varargout_2_Velocity, &b_varargout_2_Velocity_SL_Info_,
    &b_varargout_2_Velocity_SL_Inf_0, controller_joint_B.b_varargout_2_Effort,
    &b_varargout_2_Effort_SL_Info_Cu, &b_varargout_2_Effort_SL_Info_Re,
    &controller_joint_B.b_varargout_2_Header);

  // Outputs for Enabled SubSystem: '<S4>/Enabled Subsystem' incorporates:
  //   EnablePort: '<S8>/Enable'

  if (b_varargout_1) {
    memcpy(&controller_joint_B.In1.Name[0],
           &controller_joint_B.b_varargout_2_Name[0], sizeof
           (SL_Bus_controller_joint_std_msgs_String) << 4U);
    controller_joint_B.In1.Name_SL_Info.CurrentLength =
      b_varargout_2_Name_SL_Info_Curr;
    controller_joint_B.In1.Name_SL_Info.ReceivedLength =
      b_varargout_2_Name_SL_Info_Rece;
    controller_joint_B.In1.Position_SL_Info.CurrentLength =
      b_varargout_2_Position_SL_Info_;
    controller_joint_B.In1.Position_SL_Info.ReceivedLength =
      b_varargout_2_Position_SL_Inf_0;
    controller_joint_B.In1.Velocity_SL_Info.CurrentLength =
      b_varargout_2_Velocity_SL_Info_;
    controller_joint_B.In1.Velocity_SL_Info.ReceivedLength =
      b_varargout_2_Velocity_SL_Inf_0;
    memcpy(&controller_joint_B.In1.Position[0],
           &controller_joint_B.b_varargout_2_Position[0], sizeof(real_T) << 7U);
    memcpy(&controller_joint_B.In1.Velocity[0],
           &controller_joint_B.b_varargout_2_Velocity[0], sizeof(real_T) << 7U);
    memcpy(&controller_joint_B.In1.Effort[0],
           &controller_joint_B.b_varargout_2_Effort[0], sizeof(real_T) << 7U);
    controller_joint_B.In1.Effort_SL_Info.CurrentLength =
      b_varargout_2_Effort_SL_Info_Cu;
    controller_joint_B.In1.Effort_SL_Info.ReceivedLength =
      b_varargout_2_Effort_SL_Info_Re;
    controller_joint_B.In1.Header = controller_joint_B.b_varargout_2_Header;
  }

  // End of MATLABSystem: '<S4>/SourceBlock'
  // End of Outputs for SubSystem: '<S4>/Enabled Subsystem'
  // End of Outputs for SubSystem: '<Root>/Subscribe'

  // MATLABSystem: '<S5>/Get Parameter'
  ParamGet_controller_joint_93.get_parameter(&controller_joint_B.value_p);

  // MATLABSystem: '<S5>/Get Parameter1'
  ParamGet_controller_joint_94.get_parameter(&controller_joint_B.value_cv);

  // MATLABSystem: '<S5>/Get Parameter2'
  ParamGet_controller_joint_95.get_parameter(&controller_joint_B.value_f);

  // MATLABSystem: '<S5>/Get Parameter3'
  ParamGet_controller_joint_96.get_parameter(&value);

  // MATLABSystem: '<S5>/Get Parameter4'
  ParamGet_controller_joint_97.get_parameter(&value_0);

  // MATLABSystem: '<S5>/Get Parameter5'
  ParamGet_controller_joint_98.get_parameter(&value_1);

  // Product: '<Root>/MatrixMultiply' incorporates:
  //   MATLABSystem: '<S5>/Get Parameter'
  //   MATLABSystem: '<S5>/Get Parameter1'
  //   MATLABSystem: '<S5>/Get Parameter2'
  //   MATLABSystem: '<S5>/Get Parameter3'
  //   MATLABSystem: '<S5>/Get Parameter4'
  //   MATLABSystem: '<S5>/Get Parameter5'
  //   MATLABSystem: '<S7>/Get Parameter'
  //   MATLABSystem: '<S7>/Get Parameter1'
  //   MATLABSystem: '<S7>/Get Parameter2'
  //   MATLABSystem: '<S7>/Get Parameter3'
  //   MATLABSystem: '<S7>/Get Parameter4'
  //   MATLABSystem: '<S7>/Get Parameter5'
  //   Sum: '<Root>/Add1'

  controller_joint_B.Add[0] = (controller_joint_B.value -
    controller_joint_B.In1.Position[0]) * controller_joint_B.value_p;
  controller_joint_B.Add[1] = (controller_joint_B.value_m -
    controller_joint_B.In1.Position[1]) * controller_joint_B.value_cv;
  controller_joint_B.Add[2] = (controller_joint_B.value_c -
    controller_joint_B.In1.Position[2]) * controller_joint_B.value_f;
  controller_joint_B.Add[3] = (controller_joint_B.value_k -
    controller_joint_B.In1.Position[3]) * value;
  controller_joint_B.Add[4] = (controller_joint_B.value_cx -
    controller_joint_B.In1.Position[4]) * value_0;
  controller_joint_B.Add[5] = (controller_joint_B.value_b -
    controller_joint_B.In1.Position[5]) * value_1;

  // Sum: '<Root>/Add'
  for (i = 0; i < 6; i++) {
    controller_joint_B.Add[i] -= controller_joint_B.In1.Velocity[i];
  }

  // End of Sum: '<Root>/Add'

  // MATLABSystem: '<S6>/Get Parameter'
  ParamGet_controller_joint_103.get_parameter(&controller_joint_B.value);

  // MATLABSystem: '<S6>/Get Parameter1'
  ParamGet_controller_joint_104.get_parameter(&controller_joint_B.value_m);

  // MATLABSystem: '<S6>/Get Parameter2'
  ParamGet_controller_joint_105.get_parameter(&controller_joint_B.value_c);

  // MATLABSystem: '<S6>/Get Parameter3'
  ParamGet_controller_joint_106.get_parameter(&controller_joint_B.value_k);

  // MATLABSystem: '<S6>/Get Parameter4'
  ParamGet_controller_joint_107.get_parameter(&controller_joint_B.value_cx);

  // MATLABSystem: '<S6>/Get Parameter5'
  ParamGet_controller_joint_108.get_parameter(&controller_joint_B.value_b);

  // Product: '<Root>/MatrixMultiply1' incorporates:
  //   MATLABSystem: '<S6>/Get Parameter'
  //   MATLABSystem: '<S6>/Get Parameter1'
  //   MATLABSystem: '<S6>/Get Parameter2'
  //   MATLABSystem: '<S6>/Get Parameter3'
  //   MATLABSystem: '<S6>/Get Parameter4'
  //   MATLABSystem: '<S6>/Get Parameter5'

  controller_joint_B.MatrixMultiply1[0] = controller_joint_B.Add[0] *
    controller_joint_B.value;
  controller_joint_B.MatrixMultiply1[1] = controller_joint_B.Add[1] *
    controller_joint_B.value_m;
  controller_joint_B.MatrixMultiply1[2] = controller_joint_B.Add[2] *
    controller_joint_B.value_c;
  controller_joint_B.MatrixMultiply1[3] = controller_joint_B.Add[3] *
    controller_joint_B.value_k;
  controller_joint_B.MatrixMultiply1[4] = controller_joint_B.Add[4] *
    controller_joint_B.value_cx;
  controller_joint_B.MatrixMultiply1[5] = controller_joint_B.Add[5] *
    controller_joint_B.value_b;

  // MATLAB Function: '<Root>/MATLAB Function' incorporates:
  //   Constant: '<S1>/Constant'

  controller_joint_B.msg = controller_joint_P.Constant_Value_d;
  for (i = 0; i < 6; i++) {
    controller_joint_B.msg.Data[i] = controller_joint_B.MatrixMultiply1[i];
  }

  controller_joint_B.msg.Data_SL_Info.CurrentLength = 6U;
  controller_joint_B.msg.Layout.Dim_SL_Info.CurrentLength = 1U;
  controller_joint_B.msg.Layout.Dim[0].Size = 6U;
  controller_joint_B.msg.Layout.Dim[0].Stride = 6U;

  // End of MATLAB Function: '<Root>/MATLAB Function'

  // Outputs for Atomic SubSystem: '<Root>/Publish'
  // MATLABSystem: '<S3>/SinkBlock'
  Pub_controller_joint_4.publish(&controller_joint_B.msg);

  // End of Outputs for SubSystem: '<Root>/Publish'
}

// Model initialize function
void controller_joint_initialize(void)
{
  {
    char_T tmp[14];
    char_T tmp_0[10];
    char_T tmp_1[15];
    int32_T i;
    static const char_T tmp_2[13] = { '/', 'j', 'o', 'i', 'n', 't', '_', 's',
      't', 'a', 't', 'e', 's' };

    static const char_T tmp_3[13] = { '/', 'j', 'o', 'i', 'n', 't', '_', 't',
      'o', 'r', 'q', 'u', 'e' };

    static const char_T tmp_4[9] = { '/', 'q', '1', '_', 'f', 'i', 'n', 'a', 'l'
    };

    static const char_T tmp_5[9] = { '/', 'q', '2', '_', 'f', 'i', 'n', 'a', 'l'
    };

    static const char_T tmp_6[9] = { '/', 'q', '3', '_', 'f', 'i', 'n', 'a', 'l'
    };

    static const char_T tmp_7[9] = { '/', 'q', '4', '_', 'f', 'i', 'n', 'a', 'l'
    };

    static const char_T tmp_8[9] = { '/', 'q', '5', '_', 'f', 'i', 'n', 'a', 'l'
    };

    static const char_T tmp_9[9] = { '/', 'q', '6', '_', 'f', 'i', 'n', 'a', 'l'
    };

    static const char_T tmp_a[14] = { '/', 'p', 'o', 's', '_', 'c', 'o', 'n',
      't', 'r', 'o', 'l', '_', '1' };

    static const char_T tmp_b[14] = { '/', 'p', 'o', 's', '_', 'c', 'o', 'n',
      't', 'r', 'o', 'l', '_', '2' };

    static const char_T tmp_c[14] = { '/', 'p', 'o', 's', '_', 'c', 'o', 'n',
      't', 'r', 'o', 'l', '_', '3' };

    static const char_T tmp_d[14] = { '/', 'p', 'o', 's', '_', 'c', 'o', 'n',
      't', 'r', 'o', 'l', '_', '4' };

    static const char_T tmp_e[14] = { '/', 'p', 'o', 's', '_', 'c', 'o', 'n',
      't', 'r', 'o', 'l', '_', '5' };

    static const char_T tmp_f[14] = { '/', 'p', 'o', 's', '_', 'c', 'o', 'n',
      't', 'r', 'o', 'l', '_', '6' };

    static const char_T tmp_g[14] = { '/', 'v', 'e', 'l', '_', 'c', 'o', 'n',
      't', 'r', 'o', 'l', '_', '1' };

    static const char_T tmp_h[14] = { '/', 'v', 'e', 'l', '_', 'c', 'o', 'n',
      't', 'r', 'o', 'l', '_', '2' };

    static const char_T tmp_i[14] = { '/', 'v', 'e', 'l', '_', 'c', 'o', 'n',
      't', 'r', 'o', 'l', '_', '3' };

    static const char_T tmp_j[14] = { '/', 'v', 'e', 'l', '_', 'c', 'o', 'n',
      't', 'r', 'o', 'l', '_', '4' };

    static const char_T tmp_k[14] = { '/', 'v', 'e', 'l', '_', 'c', 'o', 'n',
      't', 'r', 'o', 'l', '_', '5' };

    static const char_T tmp_l[14] = { '/', 'v', 'e', 'l', '_', 'c', 'o', 'n',
      't', 'r', 'o', 'l', '_', '6' };

    // SystemInitialize for Atomic SubSystem: '<Root>/Subscribe'
    // SystemInitialize for Enabled SubSystem: '<S4>/Enabled Subsystem'
    // SystemInitialize for Outport: '<S8>/Out1'
    controller_joint_B.In1 = controller_joint_P.Out1_Y0;

    // End of SystemInitialize for SubSystem: '<S4>/Enabled Subsystem'

    // Start for MATLABSystem: '<S4>/SourceBlock'
    controller_joint_DW.obj_lb.matlabCodegenIsDeleted = false;
    controller_joint_DW.obj_lb.isInitialized = 1;
    for (i = 0; i < 13; i++) {
      tmp[i] = tmp_2[i];
    }

    tmp[13] = '\x00';
    Sub_controller_joint_5.createSubscriber(tmp, 1);
    controller_joint_DW.obj_lb.isSetupComplete = true;

    // End of Start for MATLABSystem: '<S4>/SourceBlock'
    // End of SystemInitialize for SubSystem: '<Root>/Subscribe'

    // SystemInitialize for Atomic SubSystem: '<Root>/Publish'
    // Start for MATLABSystem: '<S3>/SinkBlock'
    controller_joint_DW.obj_bq.matlabCodegenIsDeleted = false;
    controller_joint_DW.obj_bq.isInitialized = 1;
    for (i = 0; i < 13; i++) {
      tmp[i] = tmp_3[i];
    }

    tmp[13] = '\x00';
    Pub_controller_joint_4.createPublisher(tmp, 1);
    controller_joint_DW.obj_bq.isSetupComplete = true;

    // End of Start for MATLABSystem: '<S3>/SinkBlock'
    // End of SystemInitialize for SubSystem: '<Root>/Publish'

    // Start for MATLABSystem: '<S7>/Get Parameter'
    controller_joint_DW.obj.matlabCodegenIsDeleted = false;
    controller_joint_DW.obj.isInitialized = 1;
    for (i = 0; i < 9; i++) {
      tmp_0[i] = tmp_4[i];
    }

    tmp_0[9] = '\x00';
    ParamGet_controller_joint_115.initialize(tmp_0);
    ParamGet_controller_joint_115.initialize_error_codes(0, 1, 2, 3);
    ParamGet_controller_joint_115.set_initial_value(1.57);
    controller_joint_DW.obj.isSetupComplete = true;

    // End of Start for MATLABSystem: '<S7>/Get Parameter'

    // Start for MATLABSystem: '<S7>/Get Parameter1'
    controller_joint_DW.obj_p.matlabCodegenIsDeleted = false;
    controller_joint_DW.obj_p.isInitialized = 1;
    for (i = 0; i < 9; i++) {
      tmp_0[i] = tmp_5[i];
    }

    tmp_0[9] = '\x00';
    ParamGet_controller_joint_116.initialize(tmp_0);
    ParamGet_controller_joint_116.initialize_error_codes(0, 1, 2, 3);
    ParamGet_controller_joint_116.set_initial_value(1.57);
    controller_joint_DW.obj_p.isSetupComplete = true;

    // End of Start for MATLABSystem: '<S7>/Get Parameter1'

    // Start for MATLABSystem: '<S7>/Get Parameter2'
    controller_joint_DW.obj_h.matlabCodegenIsDeleted = false;
    controller_joint_DW.obj_h.isInitialized = 1;
    for (i = 0; i < 9; i++) {
      tmp_0[i] = tmp_6[i];
    }

    tmp_0[9] = '\x00';
    ParamGet_controller_joint_117.initialize(tmp_0);
    ParamGet_controller_joint_117.initialize_error_codes(0, 1, 2, 3);
    ParamGet_controller_joint_117.set_initial_value(1.57);
    controller_joint_DW.obj_h.isSetupComplete = true;

    // End of Start for MATLABSystem: '<S7>/Get Parameter2'

    // Start for MATLABSystem: '<S7>/Get Parameter3'
    controller_joint_DW.obj_b.matlabCodegenIsDeleted = false;
    controller_joint_DW.obj_b.isInitialized = 1;
    for (i = 0; i < 9; i++) {
      tmp_0[i] = tmp_7[i];
    }

    tmp_0[9] = '\x00';
    ParamGet_controller_joint_118.initialize(tmp_0);
    ParamGet_controller_joint_118.initialize_error_codes(0, 1, 2, 3);
    ParamGet_controller_joint_118.set_initial_value(1.57);
    controller_joint_DW.obj_b.isSetupComplete = true;

    // End of Start for MATLABSystem: '<S7>/Get Parameter3'

    // Start for MATLABSystem: '<S7>/Get Parameter4'
    controller_joint_DW.obj_o.matlabCodegenIsDeleted = false;
    controller_joint_DW.obj_o.isInitialized = 1;
    for (i = 0; i < 9; i++) {
      tmp_0[i] = tmp_8[i];
    }

    tmp_0[9] = '\x00';
    ParamGet_controller_joint_119.initialize(tmp_0);
    ParamGet_controller_joint_119.initialize_error_codes(0, 1, 2, 3);
    ParamGet_controller_joint_119.set_initial_value(1.57);
    controller_joint_DW.obj_o.isSetupComplete = true;

    // End of Start for MATLABSystem: '<S7>/Get Parameter4'

    // Start for MATLABSystem: '<S7>/Get Parameter5'
    controller_joint_DW.obj_i.matlabCodegenIsDeleted = false;
    controller_joint_DW.obj_i.isInitialized = 1;
    for (i = 0; i < 9; i++) {
      tmp_0[i] = tmp_9[i];
    }

    tmp_0[9] = '\x00';
    ParamGet_controller_joint_120.initialize(tmp_0);
    ParamGet_controller_joint_120.initialize_error_codes(0, 1, 2, 3);
    ParamGet_controller_joint_120.set_initial_value(1.57);
    controller_joint_DW.obj_i.isSetupComplete = true;

    // End of Start for MATLABSystem: '<S7>/Get Parameter5'

    // Start for MATLABSystem: '<S5>/Get Parameter'
    controller_joint_DW.obj_j.matlabCodegenIsDeleted = false;
    controller_joint_DW.obj_j.isInitialized = 1;
    for (i = 0; i < 14; i++) {
      tmp_1[i] = tmp_a[i];
    }

    tmp_1[14] = '\x00';
    ParamGet_controller_joint_93.initialize(tmp_1);
    ParamGet_controller_joint_93.initialize_error_codes(0, 1, 2, 3);
    ParamGet_controller_joint_93.set_initial_value(0.1);
    controller_joint_DW.obj_j.isSetupComplete = true;

    // End of Start for MATLABSystem: '<S5>/Get Parameter'

    // Start for MATLABSystem: '<S5>/Get Parameter1'
    controller_joint_DW.obj_l.matlabCodegenIsDeleted = false;
    controller_joint_DW.obj_l.isInitialized = 1;
    for (i = 0; i < 14; i++) {
      tmp_1[i] = tmp_b[i];
    }

    tmp_1[14] = '\x00';
    ParamGet_controller_joint_94.initialize(tmp_1);
    ParamGet_controller_joint_94.initialize_error_codes(0, 1, 2, 3);
    ParamGet_controller_joint_94.set_initial_value(0.1);
    controller_joint_DW.obj_l.isSetupComplete = true;

    // End of Start for MATLABSystem: '<S5>/Get Parameter1'

    // Start for MATLABSystem: '<S5>/Get Parameter2'
    controller_joint_DW.obj_nc.matlabCodegenIsDeleted = false;
    controller_joint_DW.obj_nc.isInitialized = 1;
    for (i = 0; i < 14; i++) {
      tmp_1[i] = tmp_c[i];
    }

    tmp_1[14] = '\x00';
    ParamGet_controller_joint_95.initialize(tmp_1);
    ParamGet_controller_joint_95.initialize_error_codes(0, 1, 2, 3);
    ParamGet_controller_joint_95.set_initial_value(0.1);
    controller_joint_DW.obj_nc.isSetupComplete = true;

    // End of Start for MATLABSystem: '<S5>/Get Parameter2'

    // Start for MATLABSystem: '<S5>/Get Parameter3'
    controller_joint_DW.obj_e.matlabCodegenIsDeleted = false;
    controller_joint_DW.obj_e.isInitialized = 1;
    for (i = 0; i < 14; i++) {
      tmp_1[i] = tmp_d[i];
    }

    tmp_1[14] = '\x00';
    ParamGet_controller_joint_96.initialize(tmp_1);
    ParamGet_controller_joint_96.initialize_error_codes(0, 1, 2, 3);
    ParamGet_controller_joint_96.set_initial_value(0.1);
    controller_joint_DW.obj_e.isSetupComplete = true;

    // End of Start for MATLABSystem: '<S5>/Get Parameter3'

    // Start for MATLABSystem: '<S5>/Get Parameter4'
    controller_joint_DW.obj_j1.matlabCodegenIsDeleted = false;
    controller_joint_DW.obj_j1.isInitialized = 1;
    for (i = 0; i < 14; i++) {
      tmp_1[i] = tmp_e[i];
    }

    tmp_1[14] = '\x00';
    ParamGet_controller_joint_97.initialize(tmp_1);
    ParamGet_controller_joint_97.initialize_error_codes(0, 1, 2, 3);
    ParamGet_controller_joint_97.set_initial_value(0.1);
    controller_joint_DW.obj_j1.isSetupComplete = true;

    // End of Start for MATLABSystem: '<S5>/Get Parameter4'

    // Start for MATLABSystem: '<S5>/Get Parameter5'
    controller_joint_DW.obj_f.matlabCodegenIsDeleted = false;
    controller_joint_DW.obj_f.isInitialized = 1;
    for (i = 0; i < 14; i++) {
      tmp_1[i] = tmp_f[i];
    }

    tmp_1[14] = '\x00';
    ParamGet_controller_joint_98.initialize(tmp_1);
    ParamGet_controller_joint_98.initialize_error_codes(0, 1, 2, 3);
    ParamGet_controller_joint_98.set_initial_value(0.1);
    controller_joint_DW.obj_f.isSetupComplete = true;

    // End of Start for MATLABSystem: '<S5>/Get Parameter5'

    // Start for MATLABSystem: '<S6>/Get Parameter'
    controller_joint_DW.obj_g.matlabCodegenIsDeleted = false;
    controller_joint_DW.obj_g.isInitialized = 1;
    for (i = 0; i < 14; i++) {
      tmp_1[i] = tmp_g[i];
    }

    tmp_1[14] = '\x00';
    ParamGet_controller_joint_103.initialize(tmp_1);
    ParamGet_controller_joint_103.initialize_error_codes(0, 1, 2, 3);
    ParamGet_controller_joint_103.set_initial_value(0.1);
    controller_joint_DW.obj_g.isSetupComplete = true;

    // End of Start for MATLABSystem: '<S6>/Get Parameter'

    // Start for MATLABSystem: '<S6>/Get Parameter1'
    controller_joint_DW.obj_k.matlabCodegenIsDeleted = false;
    controller_joint_DW.obj_k.isInitialized = 1;
    for (i = 0; i < 14; i++) {
      tmp_1[i] = tmp_h[i];
    }

    tmp_1[14] = '\x00';
    ParamGet_controller_joint_104.initialize(tmp_1);
    ParamGet_controller_joint_104.initialize_error_codes(0, 1, 2, 3);
    ParamGet_controller_joint_104.set_initial_value(0.1);
    controller_joint_DW.obj_k.isSetupComplete = true;

    // End of Start for MATLABSystem: '<S6>/Get Parameter1'

    // Start for MATLABSystem: '<S6>/Get Parameter2'
    controller_joint_DW.obj_a.matlabCodegenIsDeleted = false;
    controller_joint_DW.obj_a.isInitialized = 1;
    for (i = 0; i < 14; i++) {
      tmp_1[i] = tmp_i[i];
    }

    tmp_1[14] = '\x00';
    ParamGet_controller_joint_105.initialize(tmp_1);
    ParamGet_controller_joint_105.initialize_error_codes(0, 1, 2, 3);
    ParamGet_controller_joint_105.set_initial_value(0.1);
    controller_joint_DW.obj_a.isSetupComplete = true;

    // End of Start for MATLABSystem: '<S6>/Get Parameter2'

    // Start for MATLABSystem: '<S6>/Get Parameter3'
    controller_joint_DW.obj_n.matlabCodegenIsDeleted = false;
    controller_joint_DW.obj_n.isInitialized = 1;
    for (i = 0; i < 14; i++) {
      tmp_1[i] = tmp_j[i];
    }

    tmp_1[14] = '\x00';
    ParamGet_controller_joint_106.initialize(tmp_1);
    ParamGet_controller_joint_106.initialize_error_codes(0, 1, 2, 3);
    ParamGet_controller_joint_106.set_initial_value(0.01);
    controller_joint_DW.obj_n.isSetupComplete = true;

    // End of Start for MATLABSystem: '<S6>/Get Parameter3'

    // Start for MATLABSystem: '<S6>/Get Parameter4'
    controller_joint_DW.obj_nr.matlabCodegenIsDeleted = false;
    controller_joint_DW.obj_nr.isInitialized = 1;
    for (i = 0; i < 14; i++) {
      tmp_1[i] = tmp_k[i];
    }

    tmp_1[14] = '\x00';
    ParamGet_controller_joint_107.initialize(tmp_1);
    ParamGet_controller_joint_107.initialize_error_codes(0, 1, 2, 3);
    ParamGet_controller_joint_107.set_initial_value(0.1);
    controller_joint_DW.obj_nr.isSetupComplete = true;

    // End of Start for MATLABSystem: '<S6>/Get Parameter4'

    // Start for MATLABSystem: '<S6>/Get Parameter5'
    controller_joint_DW.obj_as.matlabCodegenIsDeleted = false;
    controller_joint_DW.obj_as.isInitialized = 1;
    for (i = 0; i < 14; i++) {
      tmp_1[i] = tmp_l[i];
    }

    tmp_1[14] = '\x00';
    ParamGet_controller_joint_108.initialize(tmp_1);
    ParamGet_controller_joint_108.initialize_error_codes(0, 1, 2, 3);
    ParamGet_controller_joint_108.set_initial_value(0.0001);
    controller_joint_DW.obj_as.isSetupComplete = true;

    // End of Start for MATLABSystem: '<S6>/Get Parameter5'
  }
}

// Model terminate function
void controller_joint_terminate(void)
{
  // Terminate for MATLABSystem: '<S7>/Get Parameter'
  matlabCodegenHandle_matlabCo_nl(&controller_joint_DW.obj);

  // Terminate for MATLABSystem: '<S7>/Get Parameter1'
  matlabCodegenHandle_matlabCo_nl(&controller_joint_DW.obj_p);

  // Terminate for MATLABSystem: '<S7>/Get Parameter2'
  matlabCodegenHandle_matlabCo_nl(&controller_joint_DW.obj_h);

  // Terminate for MATLABSystem: '<S7>/Get Parameter3'
  matlabCodegenHandle_matlabCo_nl(&controller_joint_DW.obj_b);

  // Terminate for MATLABSystem: '<S7>/Get Parameter4'
  matlabCodegenHandle_matlabCo_nl(&controller_joint_DW.obj_o);

  // Terminate for MATLABSystem: '<S7>/Get Parameter5'
  matlabCodegenHandle_matlabCo_nl(&controller_joint_DW.obj_i);

  // Terminate for Atomic SubSystem: '<Root>/Subscribe'
  // Terminate for MATLABSystem: '<S4>/SourceBlock'
  matlabCodegenHandle_matlabCod_n(&controller_joint_DW.obj_lb);

  // End of Terminate for SubSystem: '<Root>/Subscribe'

  // Terminate for MATLABSystem: '<S5>/Get Parameter'
  matlabCodegenHandle_matlabCo_nl(&controller_joint_DW.obj_j);

  // Terminate for MATLABSystem: '<S5>/Get Parameter1'
  matlabCodegenHandle_matlabCo_nl(&controller_joint_DW.obj_l);

  // Terminate for MATLABSystem: '<S5>/Get Parameter2'
  matlabCodegenHandle_matlabCo_nl(&controller_joint_DW.obj_nc);

  // Terminate for MATLABSystem: '<S5>/Get Parameter3'
  matlabCodegenHandle_matlabCo_nl(&controller_joint_DW.obj_e);

  // Terminate for MATLABSystem: '<S5>/Get Parameter4'
  matlabCodegenHandle_matlabCo_nl(&controller_joint_DW.obj_j1);

  // Terminate for MATLABSystem: '<S5>/Get Parameter5'
  matlabCodegenHandle_matlabCo_nl(&controller_joint_DW.obj_f);

  // Terminate for MATLABSystem: '<S6>/Get Parameter'
  matlabCodegenHandle_matlabCo_nl(&controller_joint_DW.obj_g);

  // Terminate for MATLABSystem: '<S6>/Get Parameter1'
  matlabCodegenHandle_matlabCo_nl(&controller_joint_DW.obj_k);

  // Terminate for MATLABSystem: '<S6>/Get Parameter2'
  matlabCodegenHandle_matlabCo_nl(&controller_joint_DW.obj_a);

  // Terminate for MATLABSystem: '<S6>/Get Parameter3'
  matlabCodegenHandle_matlabCo_nl(&controller_joint_DW.obj_n);

  // Terminate for MATLABSystem: '<S6>/Get Parameter4'
  matlabCodegenHandle_matlabCo_nl(&controller_joint_DW.obj_nr);

  // Terminate for MATLABSystem: '<S6>/Get Parameter5'
  matlabCodegenHandle_matlabCo_nl(&controller_joint_DW.obj_as);

  // Terminate for Atomic SubSystem: '<Root>/Publish'
  // Terminate for MATLABSystem: '<S3>/SinkBlock'
  matlabCodegenHandle_matlabCodeg(&controller_joint_DW.obj_bq);

  // End of Terminate for SubSystem: '<Root>/Publish'
}

//
// File trailer for generated code.
//
// [EOF]
//
