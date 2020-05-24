//
// Academic License - for use in teaching, academic research, and meeting
// course requirements at degree granting institutions only.  Not for
// government, commercial, or other organizational use.
//
// File: rt_defines.h
//
// Code generated for Simulink model 'cartesian_waypoints_planner'.
//
// Model version                  : 1.169
// Simulink Coder version         : 9.3 (R2020a) 18-Nov-2019
// C/C++ source code generated on : Sun May 24 12:36:19 2020
//
// Target selection: ert.tlc
// Embedded hardware selection: Generic->Unspecified (assume 32-bit Generic)
// Code generation objectives: Unspecified
// Validation result: Not run
//
#ifndef RTW_HEADER_rt_defines_h_
#define RTW_HEADER_rt_defines_h_

//===========*
//  Constants *
// ===========
#define RT_PI                          3.14159265358979323846
#define RT_PIF                         3.1415927F
#define RT_LN_10                       2.30258509299404568402
#define RT_LN_10F                      2.3025851F
#define RT_LOG10E                      0.43429448190325182765
#define RT_LOG10EF                     0.43429449F
#define RT_E                           2.7182818284590452354
#define RT_EF                          2.7182817F

//
//  UNUSED_PARAMETER(x)
//    Used to specify that a function parameter (argument) is required but not
//    accessed by the function body.

#ifndef UNUSED_PARAMETER
# if defined(__LCC__)
#   define UNUSED_PARAMETER(x)                                   // do nothing
# else

//
//  This is the semi-ANSI standard way of indicating that an
//  unused function parameter is required.

#   define UNUSED_PARAMETER(x)         (void) (x)
# endif
#endif
#endif                                 // RTW_HEADER_rt_defines_h_

//
// File trailer for generated code.
//
// [EOF]
//
