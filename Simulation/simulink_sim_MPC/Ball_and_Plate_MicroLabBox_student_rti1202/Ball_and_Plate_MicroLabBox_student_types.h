/*
 * Ball_and_Plate_MicroLabBox_student_types.h
 *
 * Academic License - for use in teaching, academic research, and meeting
 * course requirements at degree granting institutions only.  Not for
 * government, commercial, or other organizational use.
 *
 * Code generation for model "Ball_and_Plate_MicroLabBox_student".
 *
 * Model version              : 1.86
 * Simulink Coder version : 9.2 (R2019b) 18-Jul-2019
 * C source code generated on : Thu Jun 11 17:05:34 2026
 *
 * Target selection: rti1202.tlc
 * Note: GRT includes extra infrastructure and instrumentation for prototyping
 * Embedded hardware selection: Custom Processor->Custom
 * Code generation objectives: Unspecified
 * Validation result: Not run
 */

#ifndef RTW_HEADER_Ball_and_Plate_MicroLabBox_student_types_h_
#define RTW_HEADER_Ball_and_Plate_MicroLabBox_student_types_h_
#include "rtwtypes.h"
#include "multiword_types.h"
#ifndef DEFINED_TYPEDEF_FOR_struct_g6ee74EGCy7z9o7bhYM0pE_
#define DEFINED_TYPEDEF_FOR_struct_g6ee74EGCy7z9o7bhYM0pE_

typedef struct {
  real_T G[2500];
  real_T F[200];
  real_T A_condensed[30200];
  real_T b_static[604];
  real_T W_state[2416];
  real_T max_pos;
  real_T rho;
  real_T A_trans[30200];
  real_T M_inv[2500];
} struct_g6ee74EGCy7z9o7bhYM0pE;

#endif

/* Custom Type definition for MATLAB Function: '<S5>/MPC ' */
#ifndef struct_tag_sp6JfXqrPIOH71qNa8Tx03C
#define struct_tag_sp6JfXqrPIOH71qNa8Tx03C

struct tag_sp6JfXqrPIOH71qNa8Tx03C
{
  real_T G[2500];
  real_T F[200];
  real_T A_condensed[30200];
  real_T b_static[604];
  real_T W_state[2416];
  real_T max_pos;
  real_T rho;
  real_T A_trans[30200];
  real_T M_inv[2500];
};

#endif                                 /*struct_tag_sp6JfXqrPIOH71qNa8Tx03C*/

#ifndef typedef_sp6JfXqrPIOH71qNa8Tx03C_Ball__T
#define typedef_sp6JfXqrPIOH71qNa8Tx03C_Ball__T

typedef struct tag_sp6JfXqrPIOH71qNa8Tx03C sp6JfXqrPIOH71qNa8Tx03C_Ball__T;

#endif                               /*typedef_sp6JfXqrPIOH71qNa8Tx03C_Ball__T*/

/* Parameters for system: '<S34>/Enabled Subsystem' */
typedef struct P_EnabledSubsystem_Ball_and_P_T_ P_EnabledSubsystem_Ball_and_P_T;

/* Parameters for system: '<S34>/Enabled Subsystem1' */
typedef struct P_EnabledSubsystem1_Ball_and__T_ P_EnabledSubsystem1_Ball_and__T;

/* Parameters (default storage) */
typedef struct P_Ball_and_Plate_MicroLabBox_student_T_
  P_Ball_and_Plate_MicroLabBox_student_T;

/* Forward declaration for rtModel */
typedef struct tag_RTM_Ball_and_Plate_MicroLabBox_student_T
  RT_MODEL_Ball_and_Plate_MicroLabBox_student_T;

#endif              /* RTW_HEADER_Ball_and_Plate_MicroLabBox_student_types_h_ */
