//
// Academic License - for use in teaching, academic research, and meeting
// course requirements at degree granting institutions only.  Not for
// government, commercial, or other organizational use.
//
// File: controller_cartesian.cpp
//
// Code generated for Simulink model 'controller_cartesian'.
//
// Model version                  : 1.2
// Simulink Coder version         : 9.3 (R2020a) 18-Nov-2019
// C/C++ source code generated on : Tue May 26 00:11:50 2020
//
// Target selection: ert.tlc
// Embedded hardware selection: Generic->Unspecified (assume 32-bit Generic)
// Code generation objectives: Unspecified
// Validation result: Not run
//
#include "controller_cartesian.h"
#include "controller_cartesian_private.h"

// Block signals (default storage)
B_controller_cartesian_T controller_cartesian_B;

// Block states (default storage)
DW_controller_cartesian_T controller_cartesian_DW;

// Real-time model
RT_MODEL_controller_cartesian_T controller_cartesian_M_ =
  RT_MODEL_controller_cartesian_T();
RT_MODEL_controller_cartesian_T *const controller_cartesian_M =
  &controller_cartesian_M_;

// Forward declaration for local functions
static void controller_ca_SystemCore_step_d(boolean_T *varargout_1, real32_T
  varargout_2_Data[128], uint32_T *varargout_2_Data_SL_Info_Curren, uint32_T
  *varargout_2_Data_SL_Info_Receiv, uint32_T *varargout_2_Layout_DataOffset,
  SL_Bus_controller_cartesian_std_msgs_MultiArrayDimension
  varargout_2_Layout_Dim[16], uint32_T *varargout_2_Layout_Dim_SL_Info_,
  uint32_T *varargout_2_Layout_Dim_SL_Inf_0);
static void controller_cart_SystemCore_step(boolean_T *varargout_1,
  SL_Bus_controller_cartesian_std_msgs_String varargout_2_Name[16], uint32_T
  *varargout_2_Name_SL_Info_Curren, uint32_T *varargout_2_Name_SL_Info_Receiv,
  real_T varargout_2_Position[128], uint32_T *varargout_2_Position_SL_Info_Cu,
  uint32_T *varargout_2_Position_SL_Info_Re, real_T varargout_2_Velocity[128],
  uint32_T *varargout_2_Velocity_SL_Info_Cu, uint32_T
  *varargout_2_Velocity_SL_Info_Re, real_T varargout_2_Effort[128], uint32_T
  *varargout_2_Effort_SL_Info_Curr, uint32_T *varargout_2_Effort_SL_Info_Rece,
  SL_Bus_controller_cartesian_std_msgs_Header *varargout_2_Header);
static void matlabCodegenHandle_matlab_dqzq(ros_slros_internal_block_Subs_T *obj);
static void matlabCodegenHandle_matlabCodeg(ros_slros_internal_block_GetP_T *obj);
static void matlabCodegenHandle_matlabC_dqz(ros_slros_internal_block_Publ_T *obj);
static void controller_ca_SystemCore_step_d(boolean_T *varargout_1, real32_T
  varargout_2_Data[128], uint32_T *varargout_2_Data_SL_Info_Curren, uint32_T
  *varargout_2_Data_SL_Info_Receiv, uint32_T *varargout_2_Layout_DataOffset,
  SL_Bus_controller_cartesian_std_msgs_MultiArrayDimension
  varargout_2_Layout_Dim[16], uint32_T *varargout_2_Layout_Dim_SL_Info_,
  uint32_T *varargout_2_Layout_Dim_SL_Inf_0)
{
  *varargout_1 = Sub_controller_cartesian_20.getLatestMessage
    (&controller_cartesian_B.b_varargout_2_m);
  memcpy(&varargout_2_Data[0], &controller_cartesian_B.b_varargout_2_m.Data[0],
         sizeof(real32_T) << 7U);
  *varargout_2_Data_SL_Info_Curren =
    controller_cartesian_B.b_varargout_2_m.Data_SL_Info.CurrentLength;
  *varargout_2_Data_SL_Info_Receiv =
    controller_cartesian_B.b_varargout_2_m.Data_SL_Info.ReceivedLength;
  *varargout_2_Layout_DataOffset =
    controller_cartesian_B.b_varargout_2_m.Layout.DataOffset;
  memcpy(&varargout_2_Layout_Dim[0],
         &controller_cartesian_B.b_varargout_2_m.Layout.Dim[0], sizeof
         (SL_Bus_controller_cartesian_std_msgs_MultiArrayDimension) << 4U);
  *varargout_2_Layout_Dim_SL_Info_ =
    controller_cartesian_B.b_varargout_2_m.Layout.Dim_SL_Info.CurrentLength;
  *varargout_2_Layout_Dim_SL_Inf_0 =
    controller_cartesian_B.b_varargout_2_m.Layout.Dim_SL_Info.ReceivedLength;
}

static void controller_cart_SystemCore_step(boolean_T *varargout_1,
  SL_Bus_controller_cartesian_std_msgs_String varargout_2_Name[16], uint32_T
  *varargout_2_Name_SL_Info_Curren, uint32_T *varargout_2_Name_SL_Info_Receiv,
  real_T varargout_2_Position[128], uint32_T *varargout_2_Position_SL_Info_Cu,
  uint32_T *varargout_2_Position_SL_Info_Re, real_T varargout_2_Velocity[128],
  uint32_T *varargout_2_Velocity_SL_Info_Cu, uint32_T
  *varargout_2_Velocity_SL_Info_Re, real_T varargout_2_Effort[128], uint32_T
  *varargout_2_Effort_SL_Info_Curr, uint32_T *varargout_2_Effort_SL_Info_Rece,
  SL_Bus_controller_cartesian_std_msgs_Header *varargout_2_Header)
{
  *varargout_1 = Sub_controller_cartesian_18.getLatestMessage
    (&controller_cartesian_B.b_varargout_2);
  memcpy(&varargout_2_Name[0], &controller_cartesian_B.b_varargout_2.Name[0],
         sizeof(SL_Bus_controller_cartesian_std_msgs_String) << 4U);
  *varargout_2_Name_SL_Info_Curren =
    controller_cartesian_B.b_varargout_2.Name_SL_Info.CurrentLength;
  *varargout_2_Name_SL_Info_Receiv =
    controller_cartesian_B.b_varargout_2.Name_SL_Info.ReceivedLength;
  *varargout_2_Position_SL_Info_Cu =
    controller_cartesian_B.b_varargout_2.Position_SL_Info.CurrentLength;
  *varargout_2_Position_SL_Info_Re =
    controller_cartesian_B.b_varargout_2.Position_SL_Info.ReceivedLength;
  *varargout_2_Velocity_SL_Info_Cu =
    controller_cartesian_B.b_varargout_2.Velocity_SL_Info.CurrentLength;
  *varargout_2_Velocity_SL_Info_Re =
    controller_cartesian_B.b_varargout_2.Velocity_SL_Info.ReceivedLength;
  memcpy(&varargout_2_Position[0],
         &controller_cartesian_B.b_varargout_2.Position[0], sizeof(real_T) << 7U);
  memcpy(&varargout_2_Velocity[0],
         &controller_cartesian_B.b_varargout_2.Velocity[0], sizeof(real_T) << 7U);
  memcpy(&varargout_2_Effort[0], &controller_cartesian_B.b_varargout_2.Effort[0],
         sizeof(real_T) << 7U);
  *varargout_2_Effort_SL_Info_Curr =
    controller_cartesian_B.b_varargout_2.Effort_SL_Info.CurrentLength;
  *varargout_2_Effort_SL_Info_Rece =
    controller_cartesian_B.b_varargout_2.Effort_SL_Info.ReceivedLength;
  *varargout_2_Header = controller_cartesian_B.b_varargout_2.Header;
}

static void matlabCodegenHandle_matlab_dqzq(ros_slros_internal_block_Subs_T *obj)
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

static void matlabCodegenHandle_matlabC_dqz(ros_slros_internal_block_Publ_T *obj)
{
  if (!obj->matlabCodegenIsDeleted) {
    obj->matlabCodegenIsDeleted = true;
  }
}

// Model step function
void controller_cartesian_step(void)
{
  int32_T i;
  int32_T j;
  int32_T k;
  uint32_T b_varargout_2_Data_SL_Info_Curr;
  uint32_T b_varargout_2_Data_SL_Info_Rece;
  uint32_T b_varargout_2_Layout_DataOffset;
  uint32_T b_varargout_2_Layout_Dim_SL_Inf;
  uint32_T b_varargout_2_Layout_Dim_SL_I_0;
  boolean_T b_varargout_1;
  uint32_T b_varargout_2_Velocity_SL_Info_;
  uint32_T b_varargout_2_Effort_SL_Info_Cu;
  uint32_T b_varargout_2_Effort_SL_Info_Re;
  real_T tmp;
  int32_T dist_tmp;
  int32_T tmp_0;
  boolean_T exitg1;

  // Outputs for Atomic SubSystem: '<Root>/Subscribe2'
  // MATLABSystem: '<S9>/SourceBlock' incorporates:
  //   Inport: '<S14>/In1'

  controller_ca_SystemCore_step_d(&b_varargout_1,
    controller_cartesian_B.b_varargout_2_Data, &b_varargout_2_Data_SL_Info_Curr,
    &b_varargout_2_Data_SL_Info_Rece, &b_varargout_2_Layout_DataOffset,
    controller_cartesian_B.b_varargout_2_Layout_Dim,
    &b_varargout_2_Layout_Dim_SL_Inf, &b_varargout_2_Layout_Dim_SL_I_0);

  // Outputs for Enabled SubSystem: '<S9>/Enabled Subsystem' incorporates:
  //   EnablePort: '<S14>/Enable'

  if (b_varargout_1) {
    memcpy(&controller_cartesian_B.In1_o.Data[0],
           &controller_cartesian_B.b_varargout_2_Data[0], sizeof(real32_T) << 7U);
    controller_cartesian_B.In1_o.Data_SL_Info.CurrentLength =
      b_varargout_2_Data_SL_Info_Curr;
    controller_cartesian_B.In1_o.Data_SL_Info.ReceivedLength =
      b_varargout_2_Data_SL_Info_Rece;
    controller_cartesian_B.In1_o.Layout.DataOffset =
      b_varargout_2_Layout_DataOffset;
    memcpy(&controller_cartesian_B.In1_o.Layout.Dim[0],
           &controller_cartesian_B.b_varargout_2_Layout_Dim[0], sizeof
           (SL_Bus_controller_cartesian_std_msgs_MultiArrayDimension) << 4U);
    controller_cartesian_B.In1_o.Layout.Dim_SL_Info.CurrentLength =
      b_varargout_2_Layout_Dim_SL_Inf;
    controller_cartesian_B.In1_o.Layout.Dim_SL_Info.ReceivedLength =
      b_varargout_2_Layout_Dim_SL_I_0;
  }

  // End of MATLABSystem: '<S9>/SourceBlock'
  // End of Outputs for SubSystem: '<S9>/Enabled Subsystem'
  // End of Outputs for SubSystem: '<Root>/Subscribe2'

  // MATLAB Function: '<Root>/MATLAB Function1'
  for (i = 0; i < 6; i++) {
    for (j = 0; j < 20; j++) {
      controller_cartesian_B.matrix[i + 6 * j] =
        controller_cartesian_B.In1_o.Data[i * 20 + j];
    }
  }

  // End of MATLAB Function: '<Root>/MATLAB Function1'

  // MATLABSystem: '<Root>/Get Parameter'
  ParamGet_controller_cartesian_6.get_parameter(&controller_cartesian_B.t_up);

  // Outputs for Atomic SubSystem: '<Root>/Subscribe1'
  // MATLABSystem: '<S8>/SourceBlock' incorporates:
  //   Inport: '<S13>/In1'

  b_varargout_1 = Sub_controller_cartesian_19.getLatestMessage
    (&controller_cartesian_B.b_varargout_2_c);

  // Outputs for Enabled SubSystem: '<S8>/Enabled Subsystem' incorporates:
  //   EnablePort: '<S13>/Enable'

  if (b_varargout_1) {
    controller_cartesian_B.In1_n = controller_cartesian_B.b_varargout_2_c;
  }

  // End of MATLABSystem: '<S8>/SourceBlock'
  // End of Outputs for SubSystem: '<S8>/Enabled Subsystem'
  // End of Outputs for SubSystem: '<Root>/Subscribe1'

  // MATLAB Function: '<Root>/MATLAB Function2' incorporates:
  //   MATLABSystem: '<Root>/Get Parameter'

  controller_cartesian_B.delayed_time =
    (controller_cartesian_B.In1_n.Clock_.Nsec / 1.0E+9 +
     controller_cartesian_B.In1_n.Clock_.Sec) - controller_cartesian_B.t_up;
  if (controller_cartesian_B.delayed_time < 0.0) {
    controller_cartesian_B.delayed_time = 0.0;
  }

  // End of MATLAB Function: '<Root>/MATLAB Function2'

  // MATLABSystem: '<Root>/Get Parameter1'
  ParamGet_controller_cartesian_7.get_parameter(&controller_cartesian_B.t_up);

  // MATLABSystem: '<Root>/Get Parameter2'
  ParamGet_controller_cartesian_8.get_parameter(&controller_cartesian_B.value);

  // MATLAB Function: '<Root>/MATLAB Function3' incorporates:
  //   MATLABSystem: '<Root>/Get Parameter1'
  //   MATLABSystem: '<Root>/Get Parameter2'

  for (i = 0; i < 6; i++) {
    controller_cartesian_B.q[i] = controller_cartesian_B.matrix[i];
  }

  for (i = 0; i < 20; i++) {
    controller_cartesian_B.max_vel[i] = controller_cartesian_B.t_up;
    controller_cartesian_B.acc[i] = controller_cartesian_B.value;
  }

  memset(&controller_cartesian_B.t[0], 0, 342U * sizeof(real_T));
  for (i = 0; i < 6; i++) {
    controller_cartesian_B.t_up = controller_cartesian_B.max_vel[i] /
      controller_cartesian_B.acc[i];
    for (j = 0; j < 19; j++) {
      dist_tmp = 6 * j + i;
      controller_cartesian_B.dist = controller_cartesian_B.matrix[(j + 1) * 6 +
        i] - controller_cartesian_B.matrix[dist_tmp];
      if (controller_cartesian_B.dist < 0.0) {
        controller_cartesian_B.signes[dist_tmp] = -1.0;
      } else if (controller_cartesian_B.dist > 0.0) {
        controller_cartesian_B.signes[dist_tmp] = 1.0;
      } else if (controller_cartesian_B.dist == 0.0) {
        controller_cartesian_B.signes[dist_tmp] = 0.0;
      } else {
        controller_cartesian_B.signes[dist_tmp] = (rtNaN);
      }

      controller_cartesian_B.dist = fabs(controller_cartesian_B.dist);
      controller_cartesian_B.t1 = controller_cartesian_B.t_up *
        controller_cartesian_B.max_vel[i];
      if (controller_cartesian_B.dist < controller_cartesian_B.t1) {
        controller_cartesian_B.dist = sqrt(controller_cartesian_B.dist /
          controller_cartesian_B.acc[i]);
        controller_cartesian_B.t[dist_tmp + 114] = 0.0;
        controller_cartesian_B.t[dist_tmp] = controller_cartesian_B.dist;
        controller_cartesian_B.t[(i + 6 * j) + 228] =
          controller_cartesian_B.dist;
        controller_cartesian_B.act_max_vel[dist_tmp] =
          controller_cartesian_B.dist * controller_cartesian_B.acc[i];
      } else {
        controller_cartesian_B.t[dist_tmp] = controller_cartesian_B.t_up;
        controller_cartesian_B.t[dist_tmp + 228] = controller_cartesian_B.t_up;
        controller_cartesian_B.t[(i + 6 * j) + 114] =
          (controller_cartesian_B.dist - controller_cartesian_B.t1) /
          controller_cartesian_B.max_vel[i];
        controller_cartesian_B.act_max_vel[dist_tmp] =
          controller_cartesian_B.max_vel[i];
      }
    }
  }

  for (j = 0; j < 6; j++) {
    controller_cartesian_B.t1 = controller_cartesian_B.delayed_time;
    dist_tmp = 0;
    while ((dist_tmp < 19) && (!(controller_cartesian_B.t1 <= 0.0))) {
      k = 0;
      exitg1 = false;
      while ((!exitg1) && (k < 3)) {
        i = 6 * dist_tmp + j;
        tmp_0 = i + 114 * k;
        controller_cartesian_B.dist = controller_cartesian_B.t1 -
          controller_cartesian_B.t[tmp_0];
        if (controller_cartesian_B.dist <= 0.0) {
          switch (k + 1) {
           case 1:
            controller_cartesian_B.q[j] += 0.5 * controller_cartesian_B.acc[j] *
              (controller_cartesian_B.t1 * controller_cartesian_B.t1) *
              controller_cartesian_B.signes[i];
            controller_cartesian_B.t1 = controller_cartesian_B.dist;
            break;

           case 2:
            controller_cartesian_B.q[j] += controller_cartesian_B.signes[i] *
              (controller_cartesian_B.max_vel[j] * controller_cartesian_B.t1);
            controller_cartesian_B.t1 = controller_cartesian_B.dist;
            break;

           default:
            controller_cartesian_B.q[j] +=
              (controller_cartesian_B.act_max_vel[dist_tmp] *
               controller_cartesian_B.t1 - 0.5 * controller_cartesian_B.acc[j] *
               (controller_cartesian_B.t1 * controller_cartesian_B.t1)) *
              controller_cartesian_B.signes[i];
            controller_cartesian_B.t1 = controller_cartesian_B.dist;
            break;
          }

          exitg1 = true;
        } else {
          controller_cartesian_B.t1 = controller_cartesian_B.dist;
          switch (k + 1) {
           case 1:
            controller_cartesian_B.q[j] += controller_cartesian_B.t[tmp_0] *
              controller_cartesian_B.t[tmp_0] * (0.5 *
              controller_cartesian_B.acc[j]) * controller_cartesian_B.signes[i];
            break;

           case 2:
            controller_cartesian_B.q[j] += controller_cartesian_B.t[tmp_0] *
              controller_cartesian_B.max_vel[j] *
              controller_cartesian_B.signes[i];
            break;

           default:
            controller_cartesian_B.q[j] += (controller_cartesian_B.t[tmp_0] *
              controller_cartesian_B.act_max_vel[j] -
              controller_cartesian_B.t[tmp_0] * controller_cartesian_B.t[tmp_0] *
              (0.5 * controller_cartesian_B.acc[j])) *
              controller_cartesian_B.signes[i];
            break;
          }

          k++;
        }
      }

      dist_tmp++;
    }
  }

  // End of MATLAB Function: '<Root>/MATLAB Function3'

  // Outputs for Atomic SubSystem: '<Root>/Subscribe'
  // MATLABSystem: '<S7>/SourceBlock' incorporates:
  //   Inport: '<S12>/In1'

  controller_cart_SystemCore_step(&b_varargout_1,
    controller_cartesian_B.b_varargout_2_Name, &b_varargout_2_Data_SL_Info_Curr,
    &b_varargout_2_Data_SL_Info_Rece,
    controller_cartesian_B.b_varargout_2_Position,
    &b_varargout_2_Layout_DataOffset, &b_varargout_2_Layout_Dim_SL_Inf,
    controller_cartesian_B.b_varargout_2_Velocity,
    &b_varargout_2_Layout_Dim_SL_I_0, &b_varargout_2_Velocity_SL_Info_,
    controller_cartesian_B.b_varargout_2_Effort,
    &b_varargout_2_Effort_SL_Info_Cu, &b_varargout_2_Effort_SL_Info_Re,
    &controller_cartesian_B.b_varargout_2_Header);

  // Outputs for Enabled SubSystem: '<S7>/Enabled Subsystem' incorporates:
  //   EnablePort: '<S12>/Enable'

  if (b_varargout_1) {
    memcpy(&controller_cartesian_B.In1.Name[0],
           &controller_cartesian_B.b_varargout_2_Name[0], sizeof
           (SL_Bus_controller_cartesian_std_msgs_String) << 4U);
    controller_cartesian_B.In1.Name_SL_Info.CurrentLength =
      b_varargout_2_Data_SL_Info_Curr;
    controller_cartesian_B.In1.Name_SL_Info.ReceivedLength =
      b_varargout_2_Data_SL_Info_Rece;
    controller_cartesian_B.In1.Position_SL_Info.CurrentLength =
      b_varargout_2_Layout_DataOffset;
    controller_cartesian_B.In1.Position_SL_Info.ReceivedLength =
      b_varargout_2_Layout_Dim_SL_Inf;
    controller_cartesian_B.In1.Velocity_SL_Info.CurrentLength =
      b_varargout_2_Layout_Dim_SL_I_0;
    controller_cartesian_B.In1.Velocity_SL_Info.ReceivedLength =
      b_varargout_2_Velocity_SL_Info_;
    memcpy(&controller_cartesian_B.In1.Position[0],
           &controller_cartesian_B.b_varargout_2_Position[0], sizeof(real_T) <<
           7U);
    memcpy(&controller_cartesian_B.In1.Velocity[0],
           &controller_cartesian_B.b_varargout_2_Velocity[0], sizeof(real_T) <<
           7U);
    memcpy(&controller_cartesian_B.In1.Effort[0],
           &controller_cartesian_B.b_varargout_2_Effort[0], sizeof(real_T) << 7U);
    controller_cartesian_B.In1.Effort_SL_Info.CurrentLength =
      b_varargout_2_Effort_SL_Info_Cu;
    controller_cartesian_B.In1.Effort_SL_Info.ReceivedLength =
      b_varargout_2_Effort_SL_Info_Re;
    controller_cartesian_B.In1.Header =
      controller_cartesian_B.b_varargout_2_Header;
  }

  // End of MATLABSystem: '<S7>/SourceBlock'
  // End of Outputs for SubSystem: '<S7>/Enabled Subsystem'
  // End of Outputs for SubSystem: '<Root>/Subscribe'

  // Sum: '<Root>/Add1'
  for (i = 0; i < 6; i++) {
    controller_cartesian_B.q[i] -= controller_cartesian_B.In1.Position[i];
  }

  // End of Sum: '<Root>/Add1'

  // MATLABSystem: '<S10>/Get Parameter'
  ParamGet_controller_cartesian_22.get_parameter(&controller_cartesian_B.t_up);

  // MATLABSystem: '<S10>/Get Parameter1'
  ParamGet_controller_cartesian_23.get_parameter(&controller_cartesian_B.value);

  // MATLABSystem: '<S10>/Get Parameter2'
  ParamGet_controller_cartesian_24.get_parameter
    (&controller_cartesian_B.delayed_time);

  // MATLABSystem: '<S10>/Get Parameter3'
  ParamGet_controller_cartesian_25.get_parameter(&controller_cartesian_B.value_k);

  // MATLABSystem: '<S10>/Get Parameter4'
  ParamGet_controller_cartesian_26.get_parameter(&controller_cartesian_B.value_c);

  // MATLABSystem: '<S10>/Get Parameter5'
  ParamGet_controller_cartesian_27.get_parameter(&controller_cartesian_B.value_b);

  // Product: '<Root>/MatrixMultiply' incorporates:
  //   MATLABSystem: '<S10>/Get Parameter'
  //   MATLABSystem: '<S10>/Get Parameter1'
  //   MATLABSystem: '<S10>/Get Parameter2'
  //   MATLABSystem: '<S10>/Get Parameter3'
  //   MATLABSystem: '<S10>/Get Parameter4'
  //   MATLABSystem: '<S10>/Get Parameter5'

  controller_cartesian_B.dist = controller_cartesian_B.q[0];
  controller_cartesian_B.t1 = controller_cartesian_B.q[1];
  controller_cartesian_B.d = controller_cartesian_B.q[2];
  controller_cartesian_B.d1 = controller_cartesian_B.q[3];
  controller_cartesian_B.d2 = controller_cartesian_B.q[4];
  tmp = controller_cartesian_B.q[5];
  controller_cartesian_B.q[0] = controller_cartesian_B.dist *
    controller_cartesian_B.t_up;
  controller_cartesian_B.q[1] = controller_cartesian_B.t1 *
    controller_cartesian_B.value;
  controller_cartesian_B.q[2] = controller_cartesian_B.d *
    controller_cartesian_B.delayed_time;
  controller_cartesian_B.q[3] = controller_cartesian_B.d1 *
    controller_cartesian_B.value_k;
  controller_cartesian_B.q[4] = controller_cartesian_B.d2 *
    controller_cartesian_B.value_c;
  controller_cartesian_B.q[5] = tmp * controller_cartesian_B.value_b;

  // Sum: '<Root>/Add'
  for (i = 0; i < 6; i++) {
    controller_cartesian_B.q[i] -= controller_cartesian_B.In1.Velocity[i];
  }

  // End of Sum: '<Root>/Add'

  // MATLABSystem: '<S11>/Get Parameter'
  ParamGet_controller_cartesian_31.get_parameter(&controller_cartesian_B.t_up);

  // MATLABSystem: '<S11>/Get Parameter1'
  ParamGet_controller_cartesian_32.get_parameter(&controller_cartesian_B.value);

  // MATLABSystem: '<S11>/Get Parameter2'
  ParamGet_controller_cartesian_33.get_parameter
    (&controller_cartesian_B.delayed_time);

  // MATLABSystem: '<S11>/Get Parameter3'
  ParamGet_controller_cartesian_34.get_parameter(&controller_cartesian_B.value_k);

  // MATLABSystem: '<S11>/Get Parameter4'
  ParamGet_controller_cartesian_35.get_parameter(&controller_cartesian_B.value_c);

  // MATLABSystem: '<S11>/Get Parameter5'
  ParamGet_controller_cartesian_36.get_parameter(&controller_cartesian_B.value_b);

  // Product: '<Root>/MatrixMultiply1' incorporates:
  //   MATLABSystem: '<S11>/Get Parameter'
  //   MATLABSystem: '<S11>/Get Parameter1'
  //   MATLABSystem: '<S11>/Get Parameter2'
  //   MATLABSystem: '<S11>/Get Parameter3'
  //   MATLABSystem: '<S11>/Get Parameter4'
  //   MATLABSystem: '<S11>/Get Parameter5'

  controller_cartesian_B.MatrixMultiply1[0] = controller_cartesian_B.q[0] *
    controller_cartesian_B.t_up;
  controller_cartesian_B.MatrixMultiply1[1] = controller_cartesian_B.q[1] *
    controller_cartesian_B.value;
  controller_cartesian_B.MatrixMultiply1[2] = controller_cartesian_B.q[2] *
    controller_cartesian_B.delayed_time;
  controller_cartesian_B.MatrixMultiply1[3] = controller_cartesian_B.q[3] *
    controller_cartesian_B.value_k;
  controller_cartesian_B.MatrixMultiply1[4] = controller_cartesian_B.q[4] *
    controller_cartesian_B.value_c;
  controller_cartesian_B.MatrixMultiply1[5] = controller_cartesian_B.q[5] *
    controller_cartesian_B.value_b;

  // MATLAB Function: '<Root>/MATLAB Function' incorporates:
  //   Constant: '<S1>/Constant'

  controller_cartesian_B.msg = controller_cartesian_P.Constant_Value_j;
  for (i = 0; i < 6; i++) {
    controller_cartesian_B.msg.Data[i] =
      controller_cartesian_B.MatrixMultiply1[i];
  }

  controller_cartesian_B.msg.Data_SL_Info.CurrentLength = 6U;
  controller_cartesian_B.msg.Layout.Dim_SL_Info.CurrentLength = 1U;
  controller_cartesian_B.msg.Layout.Dim[0].Size = 6U;
  controller_cartesian_B.msg.Layout.Dim[0].Stride = 6U;

  // End of MATLAB Function: '<Root>/MATLAB Function'

  // Outputs for Atomic SubSystem: '<Root>/Publish'
  // MATLABSystem: '<S6>/SinkBlock'
  Pub_controller_cartesian_15.publish(&controller_cartesian_B.msg);

  // End of Outputs for SubSystem: '<Root>/Publish'
}

// Model initialize function
void controller_cartesian_initialize(void)
{
  // Registration code

  // initialize non-finites
  rt_InitInfAndNaN(sizeof(real_T));

  {
    char_T tmp[7];
    char_T tmp_0[14];
    char_T tmp_1[13];
    char_T tmp_2[15];
    int32_T i;
    static const char_T tmp_3[16] = { '/', 'j', 'o', 'i', 'n', 't', '_', 'w',
      'a', 'y', 'p', 'o', 'i', 'n', 't', 's' };

    static const char_T tmp_4[6] = { '/', 'c', 'l', 'o', 'c', 'k' };

    static const char_T tmp_5[13] = { '/', 'j', 'o', 'i', 'n', 't', '_', 's',
      't', 'a', 't', 'e', 's' };

    static const char_T tmp_6[13] = { '/', 'j', 'o', 'i', 'n', 't', '_', 't',
      'o', 'r', 'q', 'u', 'e' };

    static const char_T tmp_7[12] = { '/', 's', 't', 'a', 'r', 't', '_', 'd',
      'e', 'l', 'a', 'y' };

    static const char_T tmp_8[12] = { '/', 'm', 'a', 'x', '_', 'a', 'n', 'g',
      '_', 'v', 'e', 'l' };

    static const char_T tmp_9[12] = { '/', 'm', 'a', 'x', '_', 'a', 'n', 'g',
      '_', 'a', 'c', 'c' };

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

    // SystemInitialize for Atomic SubSystem: '<Root>/Subscribe2'
    // SystemInitialize for Enabled SubSystem: '<S9>/Enabled Subsystem'
    // SystemInitialize for Outport: '<S14>/Out1'
    controller_cartesian_B.In1_o = controller_cartesian_P.Out1_Y0_l;

    // End of SystemInitialize for SubSystem: '<S9>/Enabled Subsystem'

    // Start for MATLABSystem: '<S9>/SourceBlock'
    controller_cartesian_DW.obj_e3.matlabCodegenIsDeleted = false;
    controller_cartesian_DW.obj_e3.isInitialized = 1;
    for (i = 0; i < 16; i++) {
      controller_cartesian_B.cv[i] = tmp_3[i];
    }

    controller_cartesian_B.cv[16] = '\x00';
    Sub_controller_cartesian_20.createSubscriber(controller_cartesian_B.cv, 1);
    controller_cartesian_DW.obj_e3.isSetupComplete = true;

    // End of Start for MATLABSystem: '<S9>/SourceBlock'
    // End of SystemInitialize for SubSystem: '<Root>/Subscribe2'

    // SystemInitialize for Atomic SubSystem: '<Root>/Subscribe1'
    // SystemInitialize for Enabled SubSystem: '<S8>/Enabled Subsystem'
    // SystemInitialize for Outport: '<S13>/Out1'
    controller_cartesian_B.In1_n = controller_cartesian_P.Out1_Y0_p;

    // End of SystemInitialize for SubSystem: '<S8>/Enabled Subsystem'

    // Start for MATLABSystem: '<S8>/SourceBlock'
    controller_cartesian_DW.obj_mm.matlabCodegenIsDeleted = false;
    controller_cartesian_DW.obj_mm.isInitialized = 1;
    for (i = 0; i < 6; i++) {
      tmp[i] = tmp_4[i];
    }

    tmp[6] = '\x00';
    Sub_controller_cartesian_19.createSubscriber(tmp, 1);
    controller_cartesian_DW.obj_mm.isSetupComplete = true;

    // End of Start for MATLABSystem: '<S8>/SourceBlock'
    // End of SystemInitialize for SubSystem: '<Root>/Subscribe1'

    // SystemInitialize for Atomic SubSystem: '<Root>/Subscribe'
    // SystemInitialize for Enabled SubSystem: '<S7>/Enabled Subsystem'
    // SystemInitialize for Outport: '<S12>/Out1'
    controller_cartesian_B.In1 = controller_cartesian_P.Out1_Y0;

    // End of SystemInitialize for SubSystem: '<S7>/Enabled Subsystem'

    // Start for MATLABSystem: '<S7>/SourceBlock'
    controller_cartesian_DW.obj_el.matlabCodegenIsDeleted = false;
    controller_cartesian_DW.obj_el.isInitialized = 1;
    for (i = 0; i < 13; i++) {
      tmp_0[i] = tmp_5[i];
    }

    tmp_0[13] = '\x00';
    Sub_controller_cartesian_18.createSubscriber(tmp_0, 1);
    controller_cartesian_DW.obj_el.isSetupComplete = true;

    // End of Start for MATLABSystem: '<S7>/SourceBlock'
    // End of SystemInitialize for SubSystem: '<Root>/Subscribe'

    // SystemInitialize for Atomic SubSystem: '<Root>/Publish'
    // Start for MATLABSystem: '<S6>/SinkBlock'
    controller_cartesian_DW.obj_ga.matlabCodegenIsDeleted = false;
    controller_cartesian_DW.obj_ga.isInitialized = 1;
    for (i = 0; i < 13; i++) {
      tmp_0[i] = tmp_6[i];
    }

    tmp_0[13] = '\x00';
    Pub_controller_cartesian_15.createPublisher(tmp_0, 1);
    controller_cartesian_DW.obj_ga.isSetupComplete = true;

    // End of Start for MATLABSystem: '<S6>/SinkBlock'
    // End of SystemInitialize for SubSystem: '<Root>/Publish'

    // Start for MATLABSystem: '<Root>/Get Parameter'
    controller_cartesian_DW.obj_bo.matlabCodegenIsDeleted = false;
    controller_cartesian_DW.obj_bo.isInitialized = 1;
    for (i = 0; i < 12; i++) {
      tmp_1[i] = tmp_7[i];
    }

    tmp_1[12] = '\x00';
    ParamGet_controller_cartesian_6.initialize(tmp_1);
    ParamGet_controller_cartesian_6.initialize_error_codes(0, 1, 2, 3);
    ParamGet_controller_cartesian_6.set_initial_value(10.0);
    controller_cartesian_DW.obj_bo.isSetupComplete = true;

    // End of Start for MATLABSystem: '<Root>/Get Parameter'

    // Start for MATLABSystem: '<Root>/Get Parameter1'
    controller_cartesian_DW.obj_lh.matlabCodegenIsDeleted = false;
    controller_cartesian_DW.obj_lh.isInitialized = 1;
    for (i = 0; i < 12; i++) {
      tmp_1[i] = tmp_8[i];
    }

    tmp_1[12] = '\x00';
    ParamGet_controller_cartesian_7.initialize(tmp_1);
    ParamGet_controller_cartesian_7.initialize_error_codes(0, 1, 2, 3);
    ParamGet_controller_cartesian_7.set_initial_value(1.0);
    controller_cartesian_DW.obj_lh.isSetupComplete = true;

    // End of Start for MATLABSystem: '<Root>/Get Parameter1'

    // Start for MATLABSystem: '<Root>/Get Parameter2'
    controller_cartesian_DW.obj_i.matlabCodegenIsDeleted = false;
    controller_cartesian_DW.obj_i.isInitialized = 1;
    for (i = 0; i < 12; i++) {
      tmp_1[i] = tmp_9[i];
    }

    tmp_1[12] = '\x00';
    ParamGet_controller_cartesian_8.initialize(tmp_1);
    ParamGet_controller_cartesian_8.initialize_error_codes(0, 1, 2, 3);
    ParamGet_controller_cartesian_8.set_initial_value(0.5);
    controller_cartesian_DW.obj_i.isSetupComplete = true;

    // End of Start for MATLABSystem: '<Root>/Get Parameter2'

    // Start for MATLABSystem: '<S10>/Get Parameter'
    controller_cartesian_DW.obj_h0.matlabCodegenIsDeleted = false;
    controller_cartesian_DW.obj_h0.isInitialized = 1;
    for (i = 0; i < 14; i++) {
      tmp_2[i] = tmp_a[i];
    }

    tmp_2[14] = '\x00';
    ParamGet_controller_cartesian_22.initialize(tmp_2);
    ParamGet_controller_cartesian_22.initialize_error_codes(0, 1, 2, 3);
    ParamGet_controller_cartesian_22.set_initial_value(0.2);
    controller_cartesian_DW.obj_h0.isSetupComplete = true;

    // End of Start for MATLABSystem: '<S10>/Get Parameter'

    // Start for MATLABSystem: '<S10>/Get Parameter1'
    controller_cartesian_DW.obj_bu.matlabCodegenIsDeleted = false;
    controller_cartesian_DW.obj_bu.isInitialized = 1;
    for (i = 0; i < 14; i++) {
      tmp_2[i] = tmp_b[i];
    }

    tmp_2[14] = '\x00';
    ParamGet_controller_cartesian_23.initialize(tmp_2);
    ParamGet_controller_cartesian_23.initialize_error_codes(0, 1, 2, 3);
    ParamGet_controller_cartesian_23.set_initial_value(0.2);
    controller_cartesian_DW.obj_bu.isSetupComplete = true;

    // End of Start for MATLABSystem: '<S10>/Get Parameter1'

    // Start for MATLABSystem: '<S10>/Get Parameter2'
    controller_cartesian_DW.obj_o.matlabCodegenIsDeleted = false;
    controller_cartesian_DW.obj_o.isInitialized = 1;
    for (i = 0; i < 14; i++) {
      tmp_2[i] = tmp_c[i];
    }

    tmp_2[14] = '\x00';
    ParamGet_controller_cartesian_24.initialize(tmp_2);
    ParamGet_controller_cartesian_24.initialize_error_codes(0, 1, 2, 3);
    ParamGet_controller_cartesian_24.set_initial_value(0.2);
    controller_cartesian_DW.obj_o.isSetupComplete = true;

    // End of Start for MATLABSystem: '<S10>/Get Parameter2'

    // Start for MATLABSystem: '<S10>/Get Parameter3'
    controller_cartesian_DW.obj_hb.matlabCodegenIsDeleted = false;
    controller_cartesian_DW.obj_hb.isInitialized = 1;
    for (i = 0; i < 14; i++) {
      tmp_2[i] = tmp_d[i];
    }

    tmp_2[14] = '\x00';
    ParamGet_controller_cartesian_25.initialize(tmp_2);
    ParamGet_controller_cartesian_25.initialize_error_codes(0, 1, 2, 3);
    ParamGet_controller_cartesian_25.set_initial_value(0.2);
    controller_cartesian_DW.obj_hb.isSetupComplete = true;

    // End of Start for MATLABSystem: '<S10>/Get Parameter3'

    // Start for MATLABSystem: '<S10>/Get Parameter4'
    controller_cartesian_DW.obj_g.matlabCodegenIsDeleted = false;
    controller_cartesian_DW.obj_g.isInitialized = 1;
    for (i = 0; i < 14; i++) {
      tmp_2[i] = tmp_e[i];
    }

    tmp_2[14] = '\x00';
    ParamGet_controller_cartesian_26.initialize(tmp_2);
    ParamGet_controller_cartesian_26.initialize_error_codes(0, 1, 2, 3);
    ParamGet_controller_cartesian_26.set_initial_value(0.2);
    controller_cartesian_DW.obj_g.isSetupComplete = true;

    // End of Start for MATLABSystem: '<S10>/Get Parameter4'

    // Start for MATLABSystem: '<S10>/Get Parameter5'
    controller_cartesian_DW.obj_l.matlabCodegenIsDeleted = false;
    controller_cartesian_DW.obj_l.isInitialized = 1;
    for (i = 0; i < 14; i++) {
      tmp_2[i] = tmp_f[i];
    }

    tmp_2[14] = '\x00';
    ParamGet_controller_cartesian_27.initialize(tmp_2);
    ParamGet_controller_cartesian_27.initialize_error_codes(0, 1, 2, 3);
    ParamGet_controller_cartesian_27.set_initial_value(0.2);
    controller_cartesian_DW.obj_l.isSetupComplete = true;

    // End of Start for MATLABSystem: '<S10>/Get Parameter5'

    // Start for MATLABSystem: '<S11>/Get Parameter'
    controller_cartesian_DW.obj.matlabCodegenIsDeleted = false;
    controller_cartesian_DW.obj.isInitialized = 1;
    for (i = 0; i < 14; i++) {
      tmp_2[i] = tmp_g[i];
    }

    tmp_2[14] = '\x00';
    ParamGet_controller_cartesian_31.initialize(tmp_2);
    ParamGet_controller_cartesian_31.initialize_error_codes(0, 1, 2, 3);
    ParamGet_controller_cartesian_31.set_initial_value(0.1);
    controller_cartesian_DW.obj.isSetupComplete = true;

    // End of Start for MATLABSystem: '<S11>/Get Parameter'

    // Start for MATLABSystem: '<S11>/Get Parameter1'
    controller_cartesian_DW.obj_p.matlabCodegenIsDeleted = false;
    controller_cartesian_DW.obj_p.isInitialized = 1;
    for (i = 0; i < 14; i++) {
      tmp_2[i] = tmp_h[i];
    }

    tmp_2[14] = '\x00';
    ParamGet_controller_cartesian_32.initialize(tmp_2);
    ParamGet_controller_cartesian_32.initialize_error_codes(0, 1, 2, 3);
    ParamGet_controller_cartesian_32.set_initial_value(0.1);
    controller_cartesian_DW.obj_p.isSetupComplete = true;

    // End of Start for MATLABSystem: '<S11>/Get Parameter1'

    // Start for MATLABSystem: '<S11>/Get Parameter2'
    controller_cartesian_DW.obj_h.matlabCodegenIsDeleted = false;
    controller_cartesian_DW.obj_h.isInitialized = 1;
    for (i = 0; i < 14; i++) {
      tmp_2[i] = tmp_i[i];
    }

    tmp_2[14] = '\x00';
    ParamGet_controller_cartesian_33.initialize(tmp_2);
    ParamGet_controller_cartesian_33.initialize_error_codes(0, 1, 2, 3);
    ParamGet_controller_cartesian_33.set_initial_value(0.1);
    controller_cartesian_DW.obj_h.isSetupComplete = true;

    // End of Start for MATLABSystem: '<S11>/Get Parameter2'

    // Start for MATLABSystem: '<S11>/Get Parameter3'
    controller_cartesian_DW.obj_b.matlabCodegenIsDeleted = false;
    controller_cartesian_DW.obj_b.isInitialized = 1;
    for (i = 0; i < 14; i++) {
      tmp_2[i] = tmp_j[i];
    }

    tmp_2[14] = '\x00';
    ParamGet_controller_cartesian_34.initialize(tmp_2);
    ParamGet_controller_cartesian_34.initialize_error_codes(0, 1, 2, 3);
    ParamGet_controller_cartesian_34.set_initial_value(0.01);
    controller_cartesian_DW.obj_b.isSetupComplete = true;

    // End of Start for MATLABSystem: '<S11>/Get Parameter3'

    // Start for MATLABSystem: '<S11>/Get Parameter4'
    controller_cartesian_DW.obj_m.matlabCodegenIsDeleted = false;
    controller_cartesian_DW.obj_m.isInitialized = 1;
    for (i = 0; i < 14; i++) {
      tmp_2[i] = tmp_k[i];
    }

    tmp_2[14] = '\x00';
    ParamGet_controller_cartesian_35.initialize(tmp_2);
    ParamGet_controller_cartesian_35.initialize_error_codes(0, 1, 2, 3);
    ParamGet_controller_cartesian_35.set_initial_value(0.1);
    controller_cartesian_DW.obj_m.isSetupComplete = true;

    // End of Start for MATLABSystem: '<S11>/Get Parameter4'

    // Start for MATLABSystem: '<S11>/Get Parameter5'
    controller_cartesian_DW.obj_e.matlabCodegenIsDeleted = false;
    controller_cartesian_DW.obj_e.isInitialized = 1;
    for (i = 0; i < 14; i++) {
      tmp_2[i] = tmp_l[i];
    }

    tmp_2[14] = '\x00';
    ParamGet_controller_cartesian_36.initialize(tmp_2);
    ParamGet_controller_cartesian_36.initialize_error_codes(0, 1, 2, 3);
    ParamGet_controller_cartesian_36.set_initial_value(0.0001);
    controller_cartesian_DW.obj_e.isSetupComplete = true;

    // End of Start for MATLABSystem: '<S11>/Get Parameter5'
  }
}

// Model terminate function
void controller_cartesian_terminate(void)
{
  // Terminate for Atomic SubSystem: '<Root>/Subscribe2'
  // Terminate for MATLABSystem: '<S9>/SourceBlock'
  matlabCodegenHandle_matlab_dqzq(&controller_cartesian_DW.obj_e3);

  // End of Terminate for SubSystem: '<Root>/Subscribe2'

  // Terminate for MATLABSystem: '<Root>/Get Parameter'
  matlabCodegenHandle_matlabCodeg(&controller_cartesian_DW.obj_bo);

  // Terminate for Atomic SubSystem: '<Root>/Subscribe1'
  // Terminate for MATLABSystem: '<S8>/SourceBlock'
  matlabCodegenHandle_matlab_dqzq(&controller_cartesian_DW.obj_mm);

  // End of Terminate for SubSystem: '<Root>/Subscribe1'

  // Terminate for MATLABSystem: '<Root>/Get Parameter1'
  matlabCodegenHandle_matlabCodeg(&controller_cartesian_DW.obj_lh);

  // Terminate for MATLABSystem: '<Root>/Get Parameter2'
  matlabCodegenHandle_matlabCodeg(&controller_cartesian_DW.obj_i);

  // Terminate for Atomic SubSystem: '<Root>/Subscribe'
  // Terminate for MATLABSystem: '<S7>/SourceBlock'
  matlabCodegenHandle_matlab_dqzq(&controller_cartesian_DW.obj_el);

  // End of Terminate for SubSystem: '<Root>/Subscribe'

  // Terminate for MATLABSystem: '<S10>/Get Parameter'
  matlabCodegenHandle_matlabCodeg(&controller_cartesian_DW.obj_h0);

  // Terminate for MATLABSystem: '<S10>/Get Parameter1'
  matlabCodegenHandle_matlabCodeg(&controller_cartesian_DW.obj_bu);

  // Terminate for MATLABSystem: '<S10>/Get Parameter2'
  matlabCodegenHandle_matlabCodeg(&controller_cartesian_DW.obj_o);

  // Terminate for MATLABSystem: '<S10>/Get Parameter3'
  matlabCodegenHandle_matlabCodeg(&controller_cartesian_DW.obj_hb);

  // Terminate for MATLABSystem: '<S10>/Get Parameter4'
  matlabCodegenHandle_matlabCodeg(&controller_cartesian_DW.obj_g);

  // Terminate for MATLABSystem: '<S10>/Get Parameter5'
  matlabCodegenHandle_matlabCodeg(&controller_cartesian_DW.obj_l);

  // Terminate for MATLABSystem: '<S11>/Get Parameter'
  matlabCodegenHandle_matlabCodeg(&controller_cartesian_DW.obj);

  // Terminate for MATLABSystem: '<S11>/Get Parameter1'
  matlabCodegenHandle_matlabCodeg(&controller_cartesian_DW.obj_p);

  // Terminate for MATLABSystem: '<S11>/Get Parameter2'
  matlabCodegenHandle_matlabCodeg(&controller_cartesian_DW.obj_h);

  // Terminate for MATLABSystem: '<S11>/Get Parameter3'
  matlabCodegenHandle_matlabCodeg(&controller_cartesian_DW.obj_b);

  // Terminate for MATLABSystem: '<S11>/Get Parameter4'
  matlabCodegenHandle_matlabCodeg(&controller_cartesian_DW.obj_m);

  // Terminate for MATLABSystem: '<S11>/Get Parameter5'
  matlabCodegenHandle_matlabCodeg(&controller_cartesian_DW.obj_e);

  // Terminate for Atomic SubSystem: '<Root>/Publish'
  // Terminate for MATLABSystem: '<S6>/SinkBlock'
  matlabCodegenHandle_matlabC_dqz(&controller_cartesian_DW.obj_ga);

  // End of Terminate for SubSystem: '<Root>/Publish'
}

//
// File trailer for generated code.
//
// [EOF]
//
