//
// Academic License - for use in teaching, academic research, and meeting
// course requirements at degree granting institutions only.  Not for
// government, commercial, or other organizational use.
//
// File: forward_kinematics.h
//
// Code generated for Simulink model 'forward_kinematics'.
//
// Model version                  : 1.128
// Simulink Coder version         : 9.3 (R2020a) 18-Nov-2019
// C/C++ source code generated on : Sat May 16 00:35:08 2020
//
// Target selection: ert.tlc
// Embedded hardware selection: Generic->Unspecified (assume 32-bit Generic)
// Code generation objectives: Unspecified
// Validation result: Not run
//
#ifndef RTW_HEADER_forward_kinematics_h_
#define RTW_HEADER_forward_kinematics_h_
#include <math.h>
#include <stdlib.h>
#include <string.h>
#include <stddef.h>
#include "rtwtypes.h"
#include "rtw_continuous.h"
#include "rtw_solver.h"
#include "slros_initialize.h"
#include "forward_kinematics_types.h"
#include "rtGetNaN.h"
#include "rt_nonfinite.h"
#include "rtGetInf.h"

// Macros for accessing real-time model data structure
#ifndef rtmGetErrorStatus
# define rtmGetErrorStatus(rtm)        ((rtm)->errorStatus)
#endif

#ifndef rtmSetErrorStatus
# define rtmSetErrorStatus(rtm, val)   ((rtm)->errorStatus = (val))
#endif

// Block signals (default storage)
typedef struct {
  SL_Bus_forward_kinematics_sensor_msgs_JointState In1;// '<S12>/In1'
  SL_Bus_forward_kinematics_sensor_msgs_JointState b_varargout_2;
  SL_Bus_forward_kinematics_std_msgs_String b_varargout_2_Name[16];
  real_T b_varargout_2_Position[128];
  real_T b_varargout_2_Velocity[128];
  real_T b_varargout_2_Effort[128];
  real_T X[36];
  creal_T eigVec[16];
  creal_T At[16];
  SL_Bus_forward_kinematics_geometry_msgs_PoseStamped BusAssignment;// '<S2>/Bus Assignment' 
  SL_Bus_forward_kinematics_geometry_msgs_TwistStamped BusAssignment1;// '<S2>/Bus Assignment1' 
  SL_Bus_forward_kinematics_std_msgs_Header b_varargout_2_Header;
  real_T T1[16];
  real_T T2[16];
  real_T R[16];
  real_T T2inv[16];
  real_T T2_c[16];
  real_T T1_k[16];
  real_T T[16];
  real_T Tdh[16];
  real_T c_f1[16];
  real_T a[16];
  real_T b[16];
  real_T a_c[16];
  real_T b_V[16];
  real_T b_A[16];
  real_T c_f1_b[16];
  real_T a_p[16];
  real_T b_c[16];
  real_T a_f[16];
  f_cell_wrap_forward_kinematic_T expl_temp;
  f_cell_wrap_forward_kinematic_T expl_temp_m;
  real_T R_g[9];
  real_T R_g1[9];
  real_T R_m[9];
  real_T R_n[9];
  real_T R_p[9];
  real_T tempR[9];
  real_T R_l[9];
  real_T tempR_j[9];
  creal_T eigVal[4];
  creal_T beta1[4];
  creal_T work1[4];
  real_T MatrixMultiply[6];            // '<S1>/MatrixMultiply'
  int8_T msubspace_data[36];
  real_T cartOrn[4];
  real_T result_data[4];
  real_T work[4];
  real_T result_data_d[4];
  real_T rworka[4];
  real_T work_g[4];
  int32_T e_data[6];
  int32_T e_data_l[6];
  real_T R_d[3];
  real_T v[3];
  real_T tau[3];
  real_T v_d[3];
  real_T v_l[3];
  real_T b_v[3];
  int32_T rscale[4];
  int8_T b_I[16];
  SL_Bus_forward_kinematics_rosgraph_msgs_Clock In1_b;// '<S13>/In1'
  SL_Bus_forward_kinematics_rosgraph_msgs_Clock b_varargout_2_o;
  creal_T s;
  creal_T ctemp;
  creal_T ad22;
  creal_T ascale;
  char_T cv[15];
  char_T cv1[14];
  char_T b_b[11];
  char_T a_n[11];
  char_T b_bs[9];
  char_T b_l[9];
  char_T b_h[9];
  int8_T chainmask[8];
  char_T b_bn[8];
  char_T b_d[8];
  char_T b_e[8];
  char_T b_bj[8];
  real_T K12;
  real_T K14;
  real_T K23;
  real_T K24;
  real_T K34;
  real_T bid1;
  real_T endeffectorIndex;
  real_T s_j;
  real_T idx_idx_1;
  real_T n;
  real_T k;
  real_T sth;
  real_T tempR_tmp;
  real_T tempR_tmp_f;
  real_T tempR_tmp_a;
  real_T tempR_tmp_j;
  real_T colnorm;
  real_T scale;
  real_T absxk;
  real_T t;
  real_T anrm;
  real_T anrmto;
  real_T mul;
  real_T d;
  real_T d1;
  real_T atmp_re;
  real_T atmp_im;
  real_T anorm;
  real_T b_atol;
  real_T absxr;
  real_T absxi;
  real_T ar;
  real_T ai;
  real_T t1_re;
  real_T t1_im;
  real_T shift_re;
  real_T shift_im;
  real_T shift_im_j;
  real_T eshift_re;
  real_T eshift_im;
  real_T scale_o;
  real_T g2;
  real_T f2s;
  real_T di;
  real_T x;
  real_T fs_re;
  real_T fs_im;
  real_T gs_re;
  real_T gs_im;
  real_T a_ny;
  real_T n_i;
  real_T k_o;
  real_T sth_n;
  real_T tempR_tmp_m;
  real_T tempR_tmp_c;
  real_T tempR_tmp_md;
  real_T tempR_tmp_m3;
  real_T tst;
  real_T htmp1;
  real_T htmp2;
  real_T ba;
  real_T aa;
  real_T h12;
  real_T h21s;
  real_T unusedU1;
  real_T unusedU2;
  real_T unusedU3;
  real_T p;
  real_T bcmax;
  real_T bcmis;
  real_T scale_j;
  real_T z;
  real_T tau_h;
  real_T anorm_c;
  real_T ascale_c;
  real_T temp;
  real_T acoeff;
  real_T scale_p;
  real_T dmin;
  real_T f_y;
  real_T salpha_re;
  real_T salpha_im;
  real_T work2_idx_2_im;
  real_T work2_idx_3_re;
  real_T work2_idx_3_im;
  real_T alpha1;
  real_T xnorm;
  real_T c;
  real_T scale_p5;
  real_T g2_a;
  real_T f2s_e;
  real_T di_a;
  real_T x_a;
  real_T fs_re_i;
  real_T fs_im_l;
  real_T gs_re_o;
  char_T b_o[5];
  char_T b_i[5];
  char_T b_f[5];
  char_T b_iz[5];
  int32_T b_i_f;
  int32_T b_kstr;
  int32_T loop_ub;
  int32_T rtb_MATLABSystem_tmp;
  int32_T c_g;
  int32_T b_i_c;
  int32_T kstr;
  int32_T b_kstr_o;
  int32_T n_l;
  int32_T loop_ub_m;
  int32_T coffset_tmp;
  int32_T d_m;
  int32_T e;
  int32_T ntilecols;
  int32_T b_jtilecol;
  int32_T b_kstr_c;
  int32_T loop_ub_f;
  int32_T kstr_p;
  int32_T b_kstr_e;
  int32_T b_j;
  int32_T i;
  int32_T ihi;
  int32_T i_o;
  int32_T jcol;
  int32_T c_i;
  int32_T k_h;
  int32_T ii;
  int32_T nzcount;
  int32_T jj;
  int32_T j;
  int32_T ifirst;
  int32_T istart;
  int32_T ilast;
  int32_T ilastm1;
  int32_T iiter;
  int32_T jp1;
  int32_T jiter;
  int32_T i_l;
  int32_T ctemp_tmp;
  int32_T ctemp_tmp_tmp;
  int32_T d_h;
  int32_T e_m;
  int32_T ntilecols_m;
  int32_T b_jtilecol_h;
  int32_T b_kstr_cs;
  int32_T loop_ub_k;
  int32_T kstr_pc;
  int32_T b_kstr_p;
  int32_T i_p;
  int32_T L;
  int32_T k_a;
  int32_T m;
  int32_T nr;
  int32_T hoffset;
  int32_T j_j;
  int32_T b_j_e;
  int32_T c_j;
  int32_T ix;
  int32_T i_ol;
  int32_T c_j_b;
  int32_T e_jr;
  int32_T c_x_tmp;
  int32_T c_x_tmp_tmp;
  int32_T d_re_tmp;
  int32_T knt;
  int32_T lastc;
  int32_T rowleft;
  int32_T iac;
  int32_T g;
  int32_T b_ia;
  int32_T jy;
  int32_T b_ix;
  int32_T lastv;
  int32_T lastc_a;
  int32_T coltop;
  int32_T ix_g;
  int32_T iac_e;
  int32_T newNumel;
  int32_T i_f;
  int32_T newNumel_h;
  int32_T i_e;
  int32_T i_c;
  uint32_T b_varargout_2_Name_SL_Info_Curr;
  uint32_T b_varargout_2_Name_SL_Info_Rece;
  uint32_T b_varargout_2_Position_SL_Info_;
  uint32_T b_varargout_2_Position_SL_Inf_p;
  uint32_T b_varargout_2_Velocity_SL_Info_;
  uint32_T b_varargout_2_Velocity_SL_Inf_o;
  uint32_T b_varargout_2_Effort_SL_Info_Cu;
  uint32_T b_varargout_2_Effort_SL_Info_Re;
  boolean_T b_varargout_1;
  boolean_T b_bool;
  boolean_T b_bool_p;
  boolean_T b_bool_pt;
  boolean_T b_bool_f;
  boolean_T b_bool_i;
} B_forward_kinematics_T;

// Block states (default storage) for system '<Root>'
typedef struct {
  robotics_slmanip_internal_blo_T obj; // '<S7>/MATLAB System'
  robotics_slmanip_internal_b_e_T obj_o;// '<S8>/MATLAB System'
  n_robotics_manip_internal_Rig_T gobj_1;// '<S7>/MATLAB System'
  n_robotics_manip_internal_Rig_T gobj_2;// '<S7>/MATLAB System'
  n_robotics_manip_internal_Rig_T gobj_3;// '<S7>/MATLAB System'
  n_robotics_manip_internal_Rig_T gobj_4;// '<S7>/MATLAB System'
  n_robotics_manip_internal_Rig_T gobj_5;// '<S7>/MATLAB System'
  n_robotics_manip_internal_Rig_T gobj_6;// '<S7>/MATLAB System'
  n_robotics_manip_internal_Rig_T gobj_7;// '<S7>/MATLAB System'
  n_robotics_manip_internal_Rig_T gobj_8;// '<S7>/MATLAB System'
  n_robotics_manip_internal_Rig_T gobj_9;// '<S7>/MATLAB System'
  n_robotics_manip_internal_Rig_T gobj_10;// '<S7>/MATLAB System'
  n_robotics_manip_internal_Rig_T gobj_11;// '<S7>/MATLAB System'
  n_robotics_manip_internal_Rig_T gobj_12;// '<S7>/MATLAB System'
  n_robotics_manip_internal_Rig_T gobj_13;// '<S7>/MATLAB System'
  n_robotics_manip_internal_Rig_T gobj_14;// '<S7>/MATLAB System'
  n_robotics_manip_internal_Rig_T gobj_15;// '<S7>/MATLAB System'
  n_robotics_manip_internal_Rig_T gobj_16;// '<S7>/MATLAB System'
  n_robotics_manip_internal_R_e_T gobj_1_h;// '<S8>/MATLAB System'
  n_robotics_manip_internal_R_e_T gobj_2_g;// '<S8>/MATLAB System'
  n_robotics_manip_internal_R_e_T gobj_3_d;// '<S8>/MATLAB System'
  n_robotics_manip_internal_R_e_T gobj_4_p;// '<S8>/MATLAB System'
  n_robotics_manip_internal_R_e_T gobj_5_o;// '<S8>/MATLAB System'
  n_robotics_manip_internal_R_e_T gobj_6_m;// '<S8>/MATLAB System'
  n_robotics_manip_internal_R_e_T gobj_7_k;// '<S8>/MATLAB System'
  n_robotics_manip_internal_R_e_T gobj_8_b;// '<S8>/MATLAB System'
  n_robotics_manip_internal_R_e_T gobj_9_f;// '<S8>/MATLAB System'
  n_robotics_manip_internal_R_e_T gobj_10_l;// '<S8>/MATLAB System'
  n_robotics_manip_internal_R_e_T gobj_11_m;// '<S8>/MATLAB System'
  n_robotics_manip_internal_R_e_T gobj_12_j;// '<S8>/MATLAB System'
  n_robotics_manip_internal_R_e_T gobj_13_j;// '<S8>/MATLAB System'
  n_robotics_manip_internal_R_e_T gobj_14_p;// '<S8>/MATLAB System'
  n_robotics_manip_internal_R_e_T gobj_15_k;// '<S8>/MATLAB System'
  n_robotics_manip_internal_R_e_T gobj_16_n;// '<S8>/MATLAB System'
  ros_slros_internal_block_Publ_T obj_i;// '<S4>/SinkBlock'
  ros_slros_internal_block_Publ_T obj_l;// '<S3>/SinkBlock'
  ros_slros_internal_block_Subs_T obj_l5;// '<S6>/SourceBlock'
  ros_slros_internal_block_Subs_T obj_b;// '<S5>/SourceBlock'
} DW_forward_kinematics_T;

// Parameters (default storage)
struct P_forward_kinematics_T_ {
  SL_Bus_forward_kinematics_sensor_msgs_JointState Out1_Y0;// Computed Parameter: Out1_Y0
                                                              //  Referenced by: '<S12>/Out1'

  SL_Bus_forward_kinematics_sensor_msgs_JointState Constant_Value;// Computed Parameter: Constant_Value
                                                                     //  Referenced by: '<S5>/Constant'

  SL_Bus_forward_kinematics_geometry_msgs_PoseStamped Constant_Value_h;// Computed Parameter: Constant_Value_h
                                                                      //  Referenced by: '<S10>/Constant'

  SL_Bus_forward_kinematics_geometry_msgs_TwistStamped Constant_Value_hs;// Computed Parameter: Constant_Value_hs
                                                                      //  Referenced by: '<S11>/Constant'

  SL_Bus_forward_kinematics_rosgraph_msgs_Clock Out1_Y0_e;// Computed Parameter: Out1_Y0_e
                                                             //  Referenced by: '<S13>/Out1'

  SL_Bus_forward_kinematics_rosgraph_msgs_Clock Constant_Value_o;// Computed Parameter: Constant_Value_o
                                                                    //  Referenced by: '<S6>/Constant'

};

// Real-time Model Data Structure
struct tag_RTM_forward_kinematics_T {
  const char_T *errorStatus;
};

// Block parameters (default storage)
#ifdef __cplusplus

extern "C" {

#endif

  extern P_forward_kinematics_T forward_kinematics_P;

#ifdef __cplusplus

}
#endif

// Block signals (default storage)
#ifdef __cplusplus

extern "C" {

#endif

  extern B_forward_kinematics_T forward_kinematics_B;

#ifdef __cplusplus

}
#endif

// Block states (default storage)
extern DW_forward_kinematics_T forward_kinematics_DW;

#ifdef __cplusplus

extern "C" {

#endif

  // Model entry point functions
  extern void forward_kinematics_initialize(void);
  extern void forward_kinematics_step(void);
  extern void forward_kinematics_terminate(void);

#ifdef __cplusplus

}
#endif

// Real-time Model object
#ifdef __cplusplus

extern "C" {

#endif

  extern RT_MODEL_forward_kinematics_T *const forward_kinematics_M;

#ifdef __cplusplus

}
#endif

//-
//  The generated code includes comments that allow you to trace directly
//  back to the appropriate location in the model.  The basic format
//  is <system>/block_name, where system is the system number (uniquely
//  assigned by Simulink) and block_name is the name of the block.
//
//  Use the MATLAB hilite_system command to trace the generated code back
//  to the model.  For example,
//
//  hilite_system('<S3>')    - opens system 3
//  hilite_system('<S3>/Kp') - opens and selects block Kp which resides in S3
//
//  Here is the system hierarchy for this model
//
//  '<Root>' : 'forward_kinematics'
//  '<S1>'   : 'forward_kinematics/Forward_Kinematics'
//  '<S2>'   : 'forward_kinematics/Message_Creator'
//  '<S3>'   : 'forward_kinematics/Publish'
//  '<S4>'   : 'forward_kinematics/Publish1'
//  '<S5>'   : 'forward_kinematics/Subscribe'
//  '<S6>'   : 'forward_kinematics/Subscribe1'
//  '<S7>'   : 'forward_kinematics/Forward_Kinematics/Get Jacobian'
//  '<S8>'   : 'forward_kinematics/Forward_Kinematics/Get Transform'
//  '<S9>'   : 'forward_kinematics/Forward_Kinematics/MATLAB Function'
//  '<S10>'  : 'forward_kinematics/Message_Creator/Blank Message'
//  '<S11>'  : 'forward_kinematics/Message_Creator/Blank Message1'
//  '<S12>'  : 'forward_kinematics/Subscribe/Enabled Subsystem'
//  '<S13>'  : 'forward_kinematics/Subscribe1/Enabled Subsystem'

#endif                                 // RTW_HEADER_forward_kinematics_h_

//
// File trailer for generated code.
//
// [EOF]
//
