//
// Academic License - for use in teaching, academic research, and meeting
// course requirements at degree granting institutions only.  Not for
// government, commercial, or other organizational use.
//
// File: kinematic_simulation.cpp
//
// Code generated for Simulink model 'kinematic_simulation'.
//
// Model version                  : 1.136
// Simulink Coder version         : 9.3 (R2020a) 18-Nov-2019
// C/C++ source code generated on : Sat May 23 12:08:46 2020
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

// Previous zero-crossings (trigger) states
PrevZCX_kinematic_simulation_T kinematic_simulation_PrevZCX;

// Real-time model
RT_MODEL_kinematic_simulation_T kinematic_simulation_M_ =
  RT_MODEL_kinematic_simulation_T();
RT_MODEL_kinematic_simulation_T *const kinematic_simulation_M =
  &kinematic_simulation_M_;

// Forward declaration for local functions
static void kinematic_simul_SystemCore_step(boolean_T *varargout_1, real_T
  varargout_2_Positions[128], uint32_T *varargout_2_Positions_SL_Info_C,
  uint32_T *varargout_2_Positions_SL_Info_R, real_T varargout_2_Velocities[128],
  uint32_T *varargout_2_Velocities_SL_Info_, uint32_T
  *varargout_2_Velocities_SL_Inf_0, real_T varargout_2_Accelerations[128],
  uint32_T *varargout_2_Accelerations_SL_In, uint32_T
  *varargout_2_Accelerations_SL__0, real_T varargout_2_Effort[128], uint32_T
  *varargout_2_Effort_SL_Info_Curr, uint32_T *varargout_2_Effort_SL_Info_Rece,
  real_T *varargout_2_TimeFromStart_Sec, real_T *varargout_2_TimeFromStart_Nsec);
static void matlabCodegenHandle_matlabCodeg(ros_slros_internal_block_GetP_T *obj);
static void matlabCodegenHan_alw5typh44u2sd(ros_slros_internal_block_Subs_T *obj);
static void matlabCodegenHandle_matl_alw5ty(ros_slros_internal_block_Publ_T *obj);

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

static void matlabCodegenHandle_matlabCodeg(ros_slros_internal_block_GetP_T *obj)
{
  if (!obj->matlabCodegenIsDeleted) {
    obj->matlabCodegenIsDeleted = true;
  }
}

static void matlabCodegenHan_alw5typh44u2sd(ros_slros_internal_block_Subs_T *obj)
{
  if (!obj->matlabCodegenIsDeleted) {
    obj->matlabCodegenIsDeleted = true;
  }
}

static void matlabCodegenHandle_matl_alw5ty(ros_slros_internal_block_Publ_T *obj)
{
  if (!obj->matlabCodegenIsDeleted) {
    obj->matlabCodegenIsDeleted = true;
  }
}

// Model step function
void kinematic_simulation_step(void)
{
  int_T iS;
  ZCEventType zcEvent;
  int32_T c;
  int32_T d;
  uint32_T b_varargout_2_Positions_SL_Info;
  uint32_T b_varargout_2_Positions_SL_In_0;
  uint32_T b_varargout_2_Velocities_SL_Inf;
  uint32_T b_varargout_2_Velocities_SL_I_0;
  uint32_T b_varargout_2_Accelerations_SL_;
  uint32_T b_varargout_2_Accelerations_S_0;
  uint32_T b_varargout_2_Effort_SL_Info_Cu;
  uint32_T b_varargout_2_Effort_SL_Info_Re;
  boolean_T b_varargout_1;
  uint8_T DataTypeConversion[6];
  int32_T loop_ub;
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

  // SignalConversion generated from: '<S8>/ SFunction ' incorporates:
  //   MATLAB Function: '<S2>/Assign to JointState msg'
  //   MATLABSystem: '<S10>/Get Parameter1'
  //   MATLABSystem: '<S10>/Get Parameter2'
  //   MATLABSystem: '<S10>/Get Parameter3'
  //   MATLABSystem: '<S10>/Get Parameter4'
  //   MATLABSystem: '<S10>/Get Parameter5'
  //   MATLABSystem: '<S10>/Get Parameter6'

  ParamGet_kinematic_simulation_212.get_parameter(32U,
    kinematic_simulation_B.charValue,
    &kinematic_simulation_B.TmpSignalConversionAtSFunct[0]);
  ParamGet_kinematic_simulation_184.get_parameter(32U,
    kinematic_simulation_B.charValue_m,
    &kinematic_simulation_B.TmpSignalConversionAtSFunct[1]);
  ParamGet_kinematic_simulation_185.get_parameter(32U,
    kinematic_simulation_B.charValue_c,
    &kinematic_simulation_B.TmpSignalConversionAtSFunct[2]);
  ParamGet_kinematic_simulation_186.get_parameter(32U,
    kinematic_simulation_B.charValue_k,
    &kinematic_simulation_B.TmpSignalConversionAtSFunct[3]);
  ParamGet_kinematic_simulation_187.get_parameter(32U,
    kinematic_simulation_B.charValue_cx,
    &kinematic_simulation_B.TmpSignalConversionAtSFunct[4]);
  ParamGet_kinematic_simulation_188.get_parameter(32U,
    kinematic_simulation_B.charValue_b,
    &kinematic_simulation_B.TmpSignalConversionAtSFunct[5]);

  // DataTypeConversion: '<S6>/Data Type Conversion' incorporates:
  //   MATLABSystem: '<S11>/Get Parameter'
  //   MATLABSystem: '<S11>/Get Parameter1'
  //   MATLABSystem: '<S11>/Get Parameter2'
  //   MATLABSystem: '<S11>/Get Parameter3'
  //   MATLABSystem: '<S11>/Get Parameter4'
  //   MATLABSystem: '<S11>/Get Parameter5'
  //   SignalConversion generated from: '<S6>/Integrator'

  DataTypeConversion[0] = ParamGet_kinematic_simulation_158.get_parameter
    (&kinematic_simulation_B.TmpSignalConversionAtIntegrator[0]);
  DataTypeConversion[1] = ParamGet_kinematic_simulation_160.get_parameter
    (&kinematic_simulation_B.TmpSignalConversionAtIntegrator[1]);
  DataTypeConversion[2] = ParamGet_kinematic_simulation_163.get_parameter
    (&kinematic_simulation_B.TmpSignalConversionAtIntegrator[2]);
  DataTypeConversion[3] = ParamGet_kinematic_simulation_162.get_parameter
    (&kinematic_simulation_B.TmpSignalConversionAtIntegrator[3]);
  DataTypeConversion[4] = ParamGet_kinematic_simulation_161.get_parameter
    (&kinematic_simulation_B.TmpSignalConversionAtIntegrator[4]);
  DataTypeConversion[5] = ParamGet_kinematic_simulation_159.get_parameter
    (&kinematic_simulation_B.TmpSignalConversionAtIntegrator[5]);

  // Integrator: '<S6>/Integrator'
  if (rtmIsMajorTimeStep(kinematic_simulation_M)) {
    for (iS = 0; iS < 6; iS++) {
      zcEvent = rt_ZCFcn(FALLING_ZERO_CROSSING,
                         &kinematic_simulation_PrevZCX.Integrator_Reset_ZCE[iS],
                         (static_cast<real_T>(DataTypeConversion[iS])));
      if ((zcEvent != NO_ZCEVENT) || (kinematic_simulation_DW.Integrator_IWORK
           != 0)) {
        kinematic_simulation_X.Integrator_CSTATE[iS] =
          kinematic_simulation_B.TmpSignalConversionAtIntegrator[iS];
      }
    }
  }

  if (rtmIsMajorTimeStep(kinematic_simulation_M)) {
    // Outputs for Atomic SubSystem: '<Root>/Subscribe'
    // MATLABSystem: '<S7>/SourceBlock' incorporates:
    //   Inport: '<S12>/In1'

    kinematic_simul_SystemCore_step(&b_varargout_1,
      kinematic_simulation_B.b_varargout_2_Positions,
      &b_varargout_2_Positions_SL_Info, &b_varargout_2_Positions_SL_In_0,
      kinematic_simulation_B.b_varargout_2_Velocities,
      &b_varargout_2_Velocities_SL_Inf, &b_varargout_2_Velocities_SL_I_0,
      kinematic_simulation_B.b_varargout_2_Accelerations,
      &b_varargout_2_Accelerations_SL_, &b_varargout_2_Accelerations_S_0,
      kinematic_simulation_B.b_varargout_2_Effort,
      &b_varargout_2_Effort_SL_Info_Cu, &b_varargout_2_Effort_SL_Info_Re,
      &kinematic_simulation_B.b_varargout_2_TimeFromStart_Sec,
      &kinematic_simulation_B.Clock);

    // Outputs for Enabled SubSystem: '<S7>/Enabled Subsystem' incorporates:
    //   EnablePort: '<S12>/Enable'

    if (b_varargout_1) {
      kinematic_simulation_B.In1.Positions_SL_Info.CurrentLength =
        b_varargout_2_Positions_SL_Info;
      kinematic_simulation_B.In1.Positions_SL_Info.ReceivedLength =
        b_varargout_2_Positions_SL_In_0;
      kinematic_simulation_B.In1.Velocities_SL_Info.CurrentLength =
        b_varargout_2_Velocities_SL_Inf;
      kinematic_simulation_B.In1.Velocities_SL_Info.ReceivedLength =
        b_varargout_2_Velocities_SL_I_0;
      kinematic_simulation_B.In1.Accelerations_SL_Info.CurrentLength =
        b_varargout_2_Accelerations_SL_;
      kinematic_simulation_B.In1.Accelerations_SL_Info.ReceivedLength =
        b_varargout_2_Accelerations_S_0;
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
        b_varargout_2_Effort_SL_Info_Cu;
      kinematic_simulation_B.In1.Effort_SL_Info.ReceivedLength =
        b_varargout_2_Effort_SL_Info_Re;
      kinematic_simulation_B.In1.TimeFromStart.Sec =
        kinematic_simulation_B.b_varargout_2_TimeFromStart_Sec;
      kinematic_simulation_B.In1.TimeFromStart.Nsec =
        kinematic_simulation_B.Clock;
    }

    // End of MATLABSystem: '<S7>/SourceBlock'
    // End of Outputs for SubSystem: '<S7>/Enabled Subsystem'
    // End of Outputs for SubSystem: '<Root>/Subscribe'
  }

  for (c = 0; c < 32; c++) {
    // SignalConversion generated from: '<S8>/ SFunction ' incorporates:
    //   MATLAB Function: '<S2>/Assign to JointState msg'
    //   MATLABSystem: '<S10>/Get Parameter1'
    //   MATLABSystem: '<S10>/Get Parameter2'
    //   MATLABSystem: '<S10>/Get Parameter3'
    //   MATLABSystem: '<S10>/Get Parameter4'
    //   MATLABSystem: '<S10>/Get Parameter5'
    //   MATLABSystem: '<S10>/Get Parameter6'

    kinematic_simulation_B.TmpSignalConversionAtSFun_c[c] = static_cast<uint8_T>
      (kinematic_simulation_B.charValue[c]);
    kinematic_simulation_B.TmpSignalConversionAtSFun_c[c + 32] =
      static_cast<uint8_T>(kinematic_simulation_B.charValue_m[c]);
    kinematic_simulation_B.TmpSignalConversionAtSFun_c[c + 64] =
      static_cast<uint8_T>(kinematic_simulation_B.charValue_c[c]);
    kinematic_simulation_B.TmpSignalConversionAtSFun_c[c + 96] =
      static_cast<uint8_T>(kinematic_simulation_B.charValue_k[c]);
    kinematic_simulation_B.TmpSignalConversionAtSFun_c[c + 128] =
      static_cast<uint8_T>(kinematic_simulation_B.charValue_cx[c]);
    kinematic_simulation_B.TmpSignalConversionAtSFun_c[c + 160] =
      static_cast<uint8_T>(kinematic_simulation_B.charValue_b[c]);
  }

  // BusAssignment: '<S2>/Bus Assignment' incorporates:
  //   Constant: '<S9>/Constant'
  //   MATLAB Function: '<S2>/Assign to JointState msg'

  kinematic_simulation_B.BusAssignment = kinematic_simulation_P.Constant_Value;

  // MATLAB Function: '<S2>/Assign to JointState msg' incorporates:
  //   BusAssignment: '<S2>/Bus Assignment'
  //   Constant: '<S2>/Constant'
  //   Integrator: '<S6>/Integrator'

  kinematic_simulation_B.BusAssignment.Name_SL_Info.CurrentLength = 6U;
  kinematic_simulation_B.BusAssignment.Position_SL_Info.CurrentLength = 6U;
  kinematic_simulation_B.BusAssignment.Velocity_SL_Info.CurrentLength = 6U;
  for (iS = 0; iS < 6; iS++) {
    kinematic_simulation_B.b_varargout_2_TimeFromStart_Sec =
      ((static_cast<real_T>(iS) + 1.0) - 1.0) * static_cast<real_T>
      (kinematic_simulation_P.name_max_length);
    if (kinematic_simulation_B.b_varargout_2_TimeFromStart_Sec < 4.294967296E+9)
    {
      b_varargout_2_Positions_SL_In_0 = static_cast<uint32_T>
        (kinematic_simulation_B.b_varargout_2_TimeFromStart_Sec);
    } else {
      b_varargout_2_Positions_SL_In_0 = MAX_uint32_T;
    }

    b_varargout_2_Positions_SL_Info = b_varargout_2_Positions_SL_In_0 +
      /*MW:OvSatOk*/ 1U;
    if (b_varargout_2_Positions_SL_Info < b_varargout_2_Positions_SL_In_0) {
      b_varargout_2_Positions_SL_Info = MAX_uint32_T;
    }

    b_varargout_2_Positions_SL_In_0 = b_varargout_2_Positions_SL_Info +
      /*MW:OvSatOk*/ kinematic_simulation_B.TmpSignalConversionAtSFunct[iS];
    if (b_varargout_2_Positions_SL_In_0 < b_varargout_2_Positions_SL_Info) {
      b_varargout_2_Positions_SL_In_0 = MAX_uint32_T;
    }

    b_varargout_2_Positions_SL_In_0--;
    if (b_varargout_2_Positions_SL_Info > b_varargout_2_Positions_SL_In_0) {
      d = 0;
      c = 0;
    } else {
      d = static_cast<int32_T>(b_varargout_2_Positions_SL_Info) - 1;
      c = static_cast<int32_T>(b_varargout_2_Positions_SL_In_0);
    }

    loop_ub = c - d;
    for (c = 0; c < loop_ub; c++) {
      kinematic_simulation_B.BusAssignment.Name[iS].Data[c] =
        kinematic_simulation_B.TmpSignalConversionAtSFun_c[d + c];
    }

    kinematic_simulation_B.BusAssignment.Name[iS].Data_SL_Info.CurrentLength =
      kinematic_simulation_B.TmpSignalConversionAtSFunct[iS];
    kinematic_simulation_B.BusAssignment.Position[iS] =
      kinematic_simulation_X.Integrator_CSTATE[iS];
    kinematic_simulation_B.BusAssignment.Velocity[iS] =
      kinematic_simulation_B.In1.Velocities[iS];
  }

  // Clock: '<Root>/Clock'
  kinematic_simulation_B.Clock = kinematic_simulation_M->Timing.t[0];

  // MATLAB Function: '<Root>/MATLAB Function'
  if (kinematic_simulation_B.Clock < 0.0) {
    kinematic_simulation_B.b_varargout_2_TimeFromStart_Sec = ceil
      (kinematic_simulation_B.Clock);
  } else {
    kinematic_simulation_B.b_varargout_2_TimeFromStart_Sec = floor
      (kinematic_simulation_B.Clock);
  }

  kinematic_simulation_B.Clock = (kinematic_simulation_B.Clock -
    kinematic_simulation_B.b_varargout_2_TimeFromStart_Sec) * 1.0E+9;
  if (kinematic_simulation_B.Clock < 0.0) {
    kinematic_simulation_B.Clock = ceil(kinematic_simulation_B.Clock);
  } else {
    kinematic_simulation_B.Clock = floor(kinematic_simulation_B.Clock);
  }

  // BusAssignment: '<S2>/Bus Assignment' incorporates:
  //   BusCreator generated from: '<S2>/Bus Assignment'
  //   MATLAB Function: '<Root>/MATLAB Function'

  kinematic_simulation_B.BusAssignment.Header.Stamp.Sec =
    kinematic_simulation_B.b_varargout_2_TimeFromStart_Sec;
  kinematic_simulation_B.BusAssignment.Header.Stamp.Nsec =
    kinematic_simulation_B.Clock;

  // Outputs for Atomic SubSystem: '<Root>/Publish'
  // MATLABSystem: '<S4>/SinkBlock'
  Pub_kinematic_simulation_22.publish(&kinematic_simulation_B.BusAssignment);

  // End of Outputs for SubSystem: '<Root>/Publish'

  // BusAssignment: '<Root>/Bus Assignment' incorporates:
  //   BusCreator generated from: '<Root>/Bus Assignment'
  //   MATLAB Function: '<Root>/MATLAB Function'

  kinematic_simulation_B.BusAssignment_e.Clock_.Sec =
    kinematic_simulation_B.b_varargout_2_TimeFromStart_Sec;
  kinematic_simulation_B.BusAssignment_e.Clock_.Nsec =
    kinematic_simulation_B.Clock;

  // Outputs for Atomic SubSystem: '<Root>/Publish1'
  // MATLABSystem: '<S5>/SinkBlock'
  Pub_kinematic_simulation_201.publish(&kinematic_simulation_B.BusAssignment_e);

  // End of Outputs for SubSystem: '<Root>/Publish1'
  if (rtmIsMajorTimeStep(kinematic_simulation_M)) {
    // Update for Integrator: '<S6>/Integrator'
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

  // Derivatives for Integrator: '<S6>/Integrator'
  for (i = 0; i < 6; i++) {
    _rtXdot->Integrator_CSTATE[i] = kinematic_simulation_B.In1.Velocities[i];
  }

  // End of Derivatives for Integrator: '<S6>/Integrator'
}

// Model initialize function
void kinematic_simulation_initialize(void)
{
  // Registration code
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
    char_T initialValue[11];
    int32_T i;
    char_T tmp[14];
    char_T tmp_0[7];
    char_T tmp_1[12];
    static const char_T tmp_2[17] = { '/', 'j', 'o', 'i', 'n', 't', '_', 't',
      'r', 'a', 'j', 'e', 'c', 't', 'o', 'r', 'y' };

    static const char_T tmp_3[13] = { '/', 'j', 'o', 'i', 'n', 't', '_', 's',
      't', 'a', 't', 'e', 's' };

    static const char_T tmp_4[6] = { '/', 'c', 'l', 'o', 'c', 'k' };

    static const char_T tmp_5[24] = { '/', 'c', 'o', 'n', 't', 'r', 'o', 'l',
      'l', 'e', 'r', '_', 'j', 'o', 'i', 'n', 't', '_', 'n', 'a', 'm', 'e', '_',
      '1' };

    static const char_T tmp_6[11] = { 'e', 'd', 'o', '_', 'j', 'o', 'i', 'n',
      't', '_', '1' };

    static const char_T tmp_7[24] = { '/', 'c', 'o', 'n', 't', 'r', 'o', 'l',
      'l', 'e', 'r', '_', 'j', 'o', 'i', 'n', 't', '_', 'n', 'a', 'm', 'e', '_',
      '2' };

    static const char_T tmp_8[11] = { 'e', 'd', 'o', '_', 'j', 'o', 'i', 'n',
      't', '_', '2' };

    static const char_T tmp_9[24] = { '/', 'c', 'o', 'n', 't', 'r', 'o', 'l',
      'l', 'e', 'r', '_', 'j', 'o', 'i', 'n', 't', '_', 'n', 'a', 'm', 'e', '_',
      '3' };

    static const char_T tmp_a[11] = { 'e', 'd', 'o', '_', 'j', 'o', 'i', 'n',
      't', '_', '3' };

    static const char_T tmp_b[24] = { '/', 'c', 'o', 'n', 't', 'r', 'o', 'l',
      'l', 'e', 'r', '_', 'j', 'o', 'i', 'n', 't', '_', 'n', 'a', 'm', 'e', '_',
      '4' };

    static const char_T tmp_c[11] = { 'e', 'd', 'o', '_', 'j', 'o', 'i', 'n',
      't', '_', '4' };

    static const char_T tmp_d[24] = { '/', 'c', 'o', 'n', 't', 'r', 'o', 'l',
      'l', 'e', 'r', '_', 'j', 'o', 'i', 'n', 't', '_', 'n', 'a', 'm', 'e', '_',
      '5' };

    static const char_T tmp_e[11] = { 'e', 'd', 'o', '_', 'j', 'o', 'i', 'n',
      't', '_', '5' };

    static const char_T tmp_f[24] = { '/', 'c', 'o', 'n', 't', 'r', 'o', 'l',
      'l', 'e', 'r', '_', 'j', 'o', 'i', 'n', 't', '_', 'n', 'a', 'm', 'e', '_',
      '6' };

    static const char_T tmp_g[11] = { 'e', 'd', 'o', '_', 'j', 'o', 'i', 'n',
      't', '_', '6' };

    static const char_T tmp_h[11] = { '/', 'q', '1', '_', 'i', 'n', 'i', 't',
      'i', 'a', 'l' };

    static const char_T tmp_i[11] = { '/', 'q', '2', '_', 'i', 'n', 'i', 't',
      'i', 'a', 'l' };

    static const char_T tmp_j[11] = { '/', 'q', '3', '_', 'i', 'n', 'i', 't',
      'i', 'a', 'l' };

    static const char_T tmp_k[11] = { '/', 'q', '4', '_', 'i', 'n', 'i', 't',
      'i', 'a', 'l' };

    static const char_T tmp_l[11] = { '/', 'q', '5', '_', 'i', 'n', 'i', 't',
      'i', 'a', 'l' };

    static const char_T tmp_m[11] = { '/', 'q', '6', '_', 'i', 'n', 'i', 't',
      'i', 'a', 'l' };

    for (i = 0; i < 6; i++) {
      kinematic_simulation_PrevZCX.Integrator_Reset_ZCE[i] = UNINITIALIZED_ZCSIG;
    }

    // InitializeConditions for Integrator: '<S6>/Integrator'
    if (rtmIsFirstInitCond(kinematic_simulation_M)) {
      kinematic_simulation_X.Integrator_CSTATE[0] = 0.0;
      kinematic_simulation_X.Integrator_CSTATE[1] = 0.0;
      kinematic_simulation_X.Integrator_CSTATE[2] = 0.0;
      kinematic_simulation_X.Integrator_CSTATE[3] = 0.0;
      kinematic_simulation_X.Integrator_CSTATE[4] = 0.0;
      kinematic_simulation_X.Integrator_CSTATE[5] = 0.0;
    }

    kinematic_simulation_DW.Integrator_IWORK = 1;

    // End of InitializeConditions for Integrator: '<S6>/Integrator'

    // SystemInitialize for Atomic SubSystem: '<Root>/Subscribe'
    // SystemInitialize for Enabled SubSystem: '<S7>/Enabled Subsystem'
    // SystemInitialize for Outport: '<S12>/Out1'
    kinematic_simulation_B.In1 = kinematic_simulation_P.Out1_Y0;

    // End of SystemInitialize for SubSystem: '<S7>/Enabled Subsystem'

    // Start for MATLABSystem: '<S7>/SourceBlock'
    kinematic_simulation_DW.obj_p.matlabCodegenIsDeleted = false;
    kinematic_simulation_DW.obj_p.isInitialized = 1;
    for (i = 0; i < 17; i++) {
      kinematic_simulation_B.cv1[i] = tmp_2[i];
    }

    kinematic_simulation_B.cv1[17] = '\x00';
    Sub_kinematic_simulation_16.createSubscriber(kinematic_simulation_B.cv1, 1);
    kinematic_simulation_DW.obj_p.isSetupComplete = true;

    // End of Start for MATLABSystem: '<S7>/SourceBlock'
    // End of SystemInitialize for SubSystem: '<Root>/Subscribe'

    // SystemInitialize for Atomic SubSystem: '<Root>/Publish'
    // Start for MATLABSystem: '<S4>/SinkBlock'
    kinematic_simulation_DW.obj_nr.matlabCodegenIsDeleted = false;
    kinematic_simulation_DW.obj_nr.isInitialized = 1;
    for (i = 0; i < 13; i++) {
      tmp[i] = tmp_3[i];
    }

    tmp[13] = '\x00';
    Pub_kinematic_simulation_22.createPublisher(tmp, 1);
    kinematic_simulation_DW.obj_nr.isSetupComplete = true;

    // End of Start for MATLABSystem: '<S4>/SinkBlock'
    // End of SystemInitialize for SubSystem: '<Root>/Publish'

    // SystemInitialize for Atomic SubSystem: '<Root>/Publish1'
    // Start for MATLABSystem: '<S5>/SinkBlock'
    kinematic_simulation_DW.obj_d.matlabCodegenIsDeleted = false;
    kinematic_simulation_DW.obj_d.isInitialized = 1;
    for (i = 0; i < 6; i++) {
      tmp_0[i] = tmp_4[i];
    }

    tmp_0[6] = '\x00';
    Pub_kinematic_simulation_201.createPublisher(tmp_0, 1);
    kinematic_simulation_DW.obj_d.isSetupComplete = true;

    // End of Start for MATLABSystem: '<S5>/SinkBlock'
    // End of SystemInitialize for SubSystem: '<Root>/Publish1'

    // Start for MATLABSystem: '<S10>/Get Parameter6'
    kinematic_simulation_DW.obj_c.matlabCodegenIsDeleted = false;
    kinematic_simulation_DW.obj_c.isInitialized = 1;
    for (i = 0; i < 24; i++) {
      kinematic_simulation_B.cv[i] = tmp_5[i];
    }

    kinematic_simulation_B.cv[24] = '\x00';
    ParamGet_kinematic_simulation_212.initialize(kinematic_simulation_B.cv);
    ParamGet_kinematic_simulation_212.initialize_error_codes(0, 1, 2, 3);
    for (i = 0; i < 11; i++) {
      initialValue[i] = tmp_6[i];
    }

    ParamGet_kinematic_simulation_212.set_initial_value(initialValue, 11U);
    kinematic_simulation_DW.obj_c.isSetupComplete = true;

    // End of Start for MATLABSystem: '<S10>/Get Parameter6'

    // Start for MATLABSystem: '<S10>/Get Parameter1'
    kinematic_simulation_DW.obj_l.matlabCodegenIsDeleted = false;
    kinematic_simulation_DW.obj_l.isInitialized = 1;
    for (i = 0; i < 24; i++) {
      kinematic_simulation_B.cv[i] = tmp_7[i];
    }

    kinematic_simulation_B.cv[24] = '\x00';
    ParamGet_kinematic_simulation_184.initialize(kinematic_simulation_B.cv);
    ParamGet_kinematic_simulation_184.initialize_error_codes(0, 1, 2, 3);
    for (i = 0; i < 11; i++) {
      initialValue[i] = tmp_8[i];
    }

    ParamGet_kinematic_simulation_184.set_initial_value(initialValue, 11U);
    kinematic_simulation_DW.obj_l.isSetupComplete = true;

    // End of Start for MATLABSystem: '<S10>/Get Parameter1'

    // Start for MATLABSystem: '<S10>/Get Parameter2'
    kinematic_simulation_DW.obj_i.matlabCodegenIsDeleted = false;
    kinematic_simulation_DW.obj_i.isInitialized = 1;
    for (i = 0; i < 24; i++) {
      kinematic_simulation_B.cv[i] = tmp_9[i];
    }

    kinematic_simulation_B.cv[24] = '\x00';
    ParamGet_kinematic_simulation_185.initialize(kinematic_simulation_B.cv);
    ParamGet_kinematic_simulation_185.initialize_error_codes(0, 1, 2, 3);
    for (i = 0; i < 11; i++) {
      initialValue[i] = tmp_a[i];
    }

    ParamGet_kinematic_simulation_185.set_initial_value(initialValue, 11U);
    kinematic_simulation_DW.obj_i.isSetupComplete = true;

    // End of Start for MATLABSystem: '<S10>/Get Parameter2'

    // Start for MATLABSystem: '<S10>/Get Parameter3'
    kinematic_simulation_DW.obj_c4.matlabCodegenIsDeleted = false;
    kinematic_simulation_DW.obj_c4.isInitialized = 1;
    for (i = 0; i < 24; i++) {
      kinematic_simulation_B.cv[i] = tmp_b[i];
    }

    kinematic_simulation_B.cv[24] = '\x00';
    ParamGet_kinematic_simulation_186.initialize(kinematic_simulation_B.cv);
    ParamGet_kinematic_simulation_186.initialize_error_codes(0, 1, 2, 3);
    for (i = 0; i < 11; i++) {
      initialValue[i] = tmp_c[i];
    }

    ParamGet_kinematic_simulation_186.set_initial_value(initialValue, 11U);
    kinematic_simulation_DW.obj_c4.isSetupComplete = true;

    // End of Start for MATLABSystem: '<S10>/Get Parameter3'

    // Start for MATLABSystem: '<S10>/Get Parameter4'
    kinematic_simulation_DW.obj_cn.matlabCodegenIsDeleted = false;
    kinematic_simulation_DW.obj_cn.isInitialized = 1;
    for (i = 0; i < 24; i++) {
      kinematic_simulation_B.cv[i] = tmp_d[i];
    }

    kinematic_simulation_B.cv[24] = '\x00';
    ParamGet_kinematic_simulation_187.initialize(kinematic_simulation_B.cv);
    ParamGet_kinematic_simulation_187.initialize_error_codes(0, 1, 2, 3);
    for (i = 0; i < 11; i++) {
      initialValue[i] = tmp_e[i];
    }

    ParamGet_kinematic_simulation_187.set_initial_value(initialValue, 11U);
    kinematic_simulation_DW.obj_cn.isSetupComplete = true;

    // End of Start for MATLABSystem: '<S10>/Get Parameter4'

    // Start for MATLABSystem: '<S10>/Get Parameter5'
    kinematic_simulation_DW.obj_o.matlabCodegenIsDeleted = false;
    kinematic_simulation_DW.obj_o.isInitialized = 1;
    for (i = 0; i < 24; i++) {
      kinematic_simulation_B.cv[i] = tmp_f[i];
    }

    kinematic_simulation_B.cv[24] = '\x00';
    ParamGet_kinematic_simulation_188.initialize(kinematic_simulation_B.cv);
    ParamGet_kinematic_simulation_188.initialize_error_codes(0, 1, 2, 3);
    for (i = 0; i < 11; i++) {
      initialValue[i] = tmp_g[i];
    }

    ParamGet_kinematic_simulation_188.set_initial_value(initialValue, 11U);
    kinematic_simulation_DW.obj_o.isSetupComplete = true;

    // End of Start for MATLABSystem: '<S10>/Get Parameter5'

    // Start for MATLABSystem: '<S11>/Get Parameter'
    kinematic_simulation_DW.obj.matlabCodegenIsDeleted = false;
    kinematic_simulation_DW.obj.isInitialized = 1;
    for (i = 0; i < 11; i++) {
      tmp_1[i] = tmp_h[i];
    }

    tmp_1[11] = '\x00';
    ParamGet_kinematic_simulation_158.initialize(tmp_1);
    ParamGet_kinematic_simulation_158.initialize_error_codes(0, 1, 2, 3);
    ParamGet_kinematic_simulation_158.set_initial_value(0.0);
    kinematic_simulation_DW.obj.isSetupComplete = true;

    // End of Start for MATLABSystem: '<S11>/Get Parameter'

    // Start for MATLABSystem: '<S11>/Get Parameter2'
    kinematic_simulation_DW.obj_f.matlabCodegenIsDeleted = false;
    kinematic_simulation_DW.obj_f.isInitialized = 1;
    for (i = 0; i < 11; i++) {
      tmp_1[i] = tmp_i[i];
    }

    tmp_1[11] = '\x00';
    ParamGet_kinematic_simulation_160.initialize(tmp_1);
    ParamGet_kinematic_simulation_160.initialize_error_codes(0, 1, 2, 3);
    ParamGet_kinematic_simulation_160.set_initial_value(0.0);
    kinematic_simulation_DW.obj_f.isSetupComplete = true;

    // End of Start for MATLABSystem: '<S11>/Get Parameter2'

    // Start for MATLABSystem: '<S11>/Get Parameter5'
    kinematic_simulation_DW.obj_b.matlabCodegenIsDeleted = false;
    kinematic_simulation_DW.obj_b.isInitialized = 1;
    for (i = 0; i < 11; i++) {
      tmp_1[i] = tmp_j[i];
    }

    tmp_1[11] = '\x00';
    ParamGet_kinematic_simulation_163.initialize(tmp_1);
    ParamGet_kinematic_simulation_163.initialize_error_codes(0, 1, 2, 3);
    ParamGet_kinematic_simulation_163.set_initial_value(0.0);
    kinematic_simulation_DW.obj_b.isSetupComplete = true;

    // End of Start for MATLABSystem: '<S11>/Get Parameter5'

    // Start for MATLABSystem: '<S11>/Get Parameter4'
    kinematic_simulation_DW.obj_n.matlabCodegenIsDeleted = false;
    kinematic_simulation_DW.obj_n.isInitialized = 1;
    for (i = 0; i < 11; i++) {
      tmp_1[i] = tmp_k[i];
    }

    tmp_1[11] = '\x00';
    ParamGet_kinematic_simulation_162.initialize(tmp_1);
    ParamGet_kinematic_simulation_162.initialize_error_codes(0, 1, 2, 3);
    ParamGet_kinematic_simulation_162.set_initial_value(0.0);
    kinematic_simulation_DW.obj_n.isSetupComplete = true;

    // End of Start for MATLABSystem: '<S11>/Get Parameter4'

    // Start for MATLABSystem: '<S11>/Get Parameter3'
    kinematic_simulation_DW.obj_j.matlabCodegenIsDeleted = false;
    kinematic_simulation_DW.obj_j.isInitialized = 1;
    for (i = 0; i < 11; i++) {
      tmp_1[i] = tmp_l[i];
    }

    tmp_1[11] = '\x00';
    ParamGet_kinematic_simulation_161.initialize(tmp_1);
    ParamGet_kinematic_simulation_161.initialize_error_codes(0, 1, 2, 3);
    ParamGet_kinematic_simulation_161.set_initial_value(0.0);
    kinematic_simulation_DW.obj_j.isSetupComplete = true;

    // End of Start for MATLABSystem: '<S11>/Get Parameter3'

    // Start for MATLABSystem: '<S11>/Get Parameter1'
    kinematic_simulation_DW.obj_e.matlabCodegenIsDeleted = false;
    kinematic_simulation_DW.obj_e.isInitialized = 1;
    for (i = 0; i < 11; i++) {
      tmp_1[i] = tmp_m[i];
    }

    tmp_1[11] = '\x00';
    ParamGet_kinematic_simulation_159.initialize(tmp_1);
    ParamGet_kinematic_simulation_159.initialize_error_codes(0, 1, 2, 3);
    ParamGet_kinematic_simulation_159.set_initial_value(0.0);
    kinematic_simulation_DW.obj_e.isSetupComplete = true;

    // End of Start for MATLABSystem: '<S11>/Get Parameter1'
  }

  // set "at time zero" to false
  if (rtmIsFirstInitCond(kinematic_simulation_M)) {
    rtmSetFirstInitCond(kinematic_simulation_M, 0);
  }
}

// Model terminate function
void kinematic_simulation_terminate(void)
{
  // Terminate for MATLABSystem: '<S10>/Get Parameter6'
  matlabCodegenHandle_matlabCodeg(&kinematic_simulation_DW.obj_c);

  // Terminate for MATLABSystem: '<S10>/Get Parameter1'
  matlabCodegenHandle_matlabCodeg(&kinematic_simulation_DW.obj_l);

  // Terminate for MATLABSystem: '<S10>/Get Parameter2'
  matlabCodegenHandle_matlabCodeg(&kinematic_simulation_DW.obj_i);

  // Terminate for MATLABSystem: '<S10>/Get Parameter3'
  matlabCodegenHandle_matlabCodeg(&kinematic_simulation_DW.obj_c4);

  // Terminate for MATLABSystem: '<S10>/Get Parameter4'
  matlabCodegenHandle_matlabCodeg(&kinematic_simulation_DW.obj_cn);

  // Terminate for MATLABSystem: '<S10>/Get Parameter5'
  matlabCodegenHandle_matlabCodeg(&kinematic_simulation_DW.obj_o);

  // Terminate for MATLABSystem: '<S11>/Get Parameter'
  matlabCodegenHandle_matlabCodeg(&kinematic_simulation_DW.obj);

  // Terminate for MATLABSystem: '<S11>/Get Parameter2'
  matlabCodegenHandle_matlabCodeg(&kinematic_simulation_DW.obj_f);

  // Terminate for MATLABSystem: '<S11>/Get Parameter5'
  matlabCodegenHandle_matlabCodeg(&kinematic_simulation_DW.obj_b);

  // Terminate for MATLABSystem: '<S11>/Get Parameter4'
  matlabCodegenHandle_matlabCodeg(&kinematic_simulation_DW.obj_n);

  // Terminate for MATLABSystem: '<S11>/Get Parameter3'
  matlabCodegenHandle_matlabCodeg(&kinematic_simulation_DW.obj_j);

  // Terminate for MATLABSystem: '<S11>/Get Parameter1'
  matlabCodegenHandle_matlabCodeg(&kinematic_simulation_DW.obj_e);

  // Terminate for Atomic SubSystem: '<Root>/Subscribe'
  // Terminate for MATLABSystem: '<S7>/SourceBlock'
  matlabCodegenHan_alw5typh44u2sd(&kinematic_simulation_DW.obj_p);

  // End of Terminate for SubSystem: '<Root>/Subscribe'

  // Terminate for Atomic SubSystem: '<Root>/Publish'
  // Terminate for MATLABSystem: '<S4>/SinkBlock'
  matlabCodegenHandle_matl_alw5ty(&kinematic_simulation_DW.obj_nr);

  // End of Terminate for SubSystem: '<Root>/Publish'

  // Terminate for Atomic SubSystem: '<Root>/Publish1'
  // Terminate for MATLABSystem: '<S5>/SinkBlock'
  matlabCodegenHandle_matl_alw5ty(&kinematic_simulation_DW.obj_d);

  // End of Terminate for SubSystem: '<Root>/Publish1'
}

//
// File trailer for generated code.
//
// [EOF]
//
