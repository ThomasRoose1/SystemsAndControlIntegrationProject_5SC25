/*
 * Ball_and_Plate_MicroLabBox_student_private.h
 *
 * Academic License - for use in teaching, academic research, and meeting
 * course requirements at degree granting institutions only.  Not for
 * government, commercial, or other organizational use.
 *
 * Code generation for model "Ball_and_Plate_MicroLabBox_student".
 *
<<<<<<< HEAD
 * Model version              : 1.12
 * Simulink Coder version : 9.2 (R2019b) 18-Jul-2019
 * C source code generated on : Wed May 13 12:01:12 2026
=======
 * Model version              : 1.23
 * Simulink Coder version : 9.2 (R2019b) 18-Jul-2019
 * C source code generated on : Tue May 19 15:21:40 2026
>>>>>>> Thomas
 *
 * Target selection: rti1202.tlc
 * Note: GRT includes extra infrastructure and instrumentation for prototyping
 * Embedded hardware selection: Custom Processor->Custom
 * Code generation objectives: Unspecified
 * Validation result: Not run
 */

#ifndef RTW_HEADER_Ball_and_Plate_MicroLabBox_student_private_h_
#define RTW_HEADER_Ball_and_Plate_MicroLabBox_student_private_h_
#include "rtwtypes.h"
#include "multiword_types.h"
#include "Ball_and_Plate_MicroLabBox_student.h"

/* Private macros used by the generated code to access rtModel */
#ifndef rtmIsMajorTimeStep
# define rtmIsMajorTimeStep(rtm)       (((rtm)->Timing.simTimeStep) == MAJOR_TIME_STEP)
#endif

#ifndef rtmIsMinorTimeStep
# define rtmIsMinorTimeStep(rtm)       (((rtm)->Timing.simTimeStep) == MINOR_TIME_STEP)
#endif

#ifndef rtmSetTFinal
# define rtmSetTFinal(rtm, val)        ((rtm)->Timing.tFinal = (val))
#endif

#ifndef rtmSetTPtr
# define rtmSetTPtr(rtm, val)          ((rtm)->Timing.t = (val))
#endif

#ifndef UCHAR_MAX
#include <limits.h>
#endif

#if ( UCHAR_MAX != (0xFFU) ) || ( SCHAR_MAX != (0x7F) )
#error Code was generated for compiler with different sized uchar/char. \
Consider adjusting Test hardware word size settings on the \
Hardware Implementation pane to match your compiler word sizes as \
defined in limits.h of the compiler. Alternatively, you can \
select the Test hardware is the same as production hardware option and \
select the Enable portable word sizes option on the Code Generation > \
Verification pane for ERT based targets, which will disable the \
preprocessor word size checks.
#endif

#if ( USHRT_MAX != (0xFFFFU) ) || ( SHRT_MAX != (0x7FFF) )
#error Code was generated for compiler with different sized ushort/short. \
Consider adjusting Test hardware word size settings on the \
Hardware Implementation pane to match your compiler word sizes as \
defined in limits.h of the compiler. Alternatively, you can \
select the Test hardware is the same as production hardware option and \
select the Enable portable word sizes option on the Code Generation > \
Verification pane for ERT based targets, which will disable the \
preprocessor word size checks.
#endif

#if ( UINT_MAX != (0xFFFFFFFFU) ) || ( INT_MAX != (0x7FFFFFFF) )
#error Code was generated for compiler with different sized uint/int. \
Consider adjusting Test hardware word size settings on the \
Hardware Implementation pane to match your compiler word sizes as \
defined in limits.h of the compiler. Alternatively, you can \
select the Test hardware is the same as production hardware option and \
select the Enable portable word sizes option on the Code Generation > \
Verification pane for ERT based targets, which will disable the \
preprocessor word size checks.
#endif

#if ( ULONG_MAX != (0xFFFFFFFFU) ) || ( LONG_MAX != (0x7FFFFFFF) )
#error Code was generated for compiler with different sized ulong/long. \
Consider adjusting Test hardware word size settings on the \
Hardware Implementation pane to match your compiler word sizes as \
defined in limits.h of the compiler. Alternatively, you can \
select the Test hardware is the same as production hardware option and \
select the Enable portable word sizes option on the Code Generation > \
Verification pane for ERT based targets, which will disable the \
preprocessor word size checks.
#endif

/* Skipping ulong_long/long_long check: insufficient preprocessor integer range. */
extern DacCl1AnalogOutSDrvObject *pRTIDacC1AnalogOut_Ch_1;
extern DioCl2EncoderInSDrvObject *pRTIEmcEncoder_Unit_1_DioCl_2_Port_1_Ch1;
extern DacCl1AnalogOutSDrvObject *pRTIDacC1AnalogOut_Ch_2;
extern DioCl2EncoderInSDrvObject *pRTIEmcEncoder_Unit_2_DioCl_2_Port_1_Ch3;
extern DacCl1AnalogOutSDrvObject *pRTIDacC1AnalogOut_Ch_3;
extern DioCl2EncoderInSDrvObject *pRTIEmcEncoder_Unit_3_DioCl_2_Port_1_Ch5;
extern real_T rt_powd_snf(real_T u0, real_T u1);
extern void dleadlag(SimStruct *rts);
extern void dlowpass1(SimStruct *rts);
extern void Ball_and_Pl_MATLABFunction_Init(DW_MATLABFunction_Ball_and_Pl_T
  *localDW);
extern void Ball_and_Plate_M_MATLABFunction(real_T rtu_enable, const real_T
  rtu_u[3001], B_MATLABFunction_Ball_and_Pla_T *localB,
  DW_MATLABFunction_Ball_and_Pl_T *localDW);
extern void Ball_and_Plate__MATLABFunction2(real_T rtu_y, real_T rtu_start,
  real_T rtu_init_value, B_MATLABFunction2_Ball_and_Pl_T *localB);
extern void Ball_and_P_MATLABFunction1_Init(DW_MATLABFunction1_Ball_and_P_T
  *localDW);
extern void Ball_and_Plate__MATLABFunction1(real_T rtu_reference_end, real_T
  rtu_end_time, real_T rtu_Ts, B_MATLABFunction1_Ball_and_Pl_T *localB,
  DW_MATLABFunction1_Ball_and_P_T *localDW);
extern void Ball_and__MATLABFunction_o_Init(DW_MATLABFunction_Ball_and__j_T
  *localDW);
extern void Ball_and_Plate_MATLABFunction_e(real_T rtu_enable, const real_T
  rtu_u[200000], B_MATLABFunction_Ball_and_P_m_T *localB,
  DW_MATLABFunction_Ball_and__j_T *localDW);
extern void Ball_and__EnabledSubsystem_Init(B_EnabledSubsystem_Ball_and_P_T
  *localB, P_EnabledSubsystem_Ball_and_P_T *localP);
extern void Ball_and_EnabledSubsystem_Start(DW_EnabledSubsystem_Ball_and__T
  *localDW);
extern void Ball_a_EnabledSubsystem_Disable(B_EnabledSubsystem_Ball_and_P_T
  *localB, DW_EnabledSubsystem_Ball_and__T *localDW,
  P_EnabledSubsystem_Ball_and_P_T *localP);
extern void Ball_and_Plate_EnabledSubsystem
  (RT_MODEL_Ball_and_Plate_MicroLabBox_student_T * const
   Ball_and_Plate_MicroLabBox_student_M, real_T rtu_Enable,
   B_EnabledSubsystem_Ball_and_P_T *localB, DW_EnabledSubsystem_Ball_and__T
   *localDW, P_EnabledSubsystem_Ball_and_P_T *localP);
extern void Ball_and_EnabledSubsystem1_Init(B_EnabledSubsystem1_Ball_and__T
  *localB, P_EnabledSubsystem1_Ball_and__T *localP);
extern void Ball_an_EnabledSubsystem1_Start(DW_EnabledSubsystem1_Ball_and_T
  *localDW);
extern void Ball__EnabledSubsystem1_Disable(DW_EnabledSubsystem1_Ball_and_T
  *localDW);
extern void Ball_and_Plat_EnabledSubsystem1
  (RT_MODEL_Ball_and_Plate_MicroLabBox_student_T * const
   Ball_and_Plate_MicroLabBox_student_M, boolean_T rtu_Enable, real_T rtu_In1,
   B_EnabledSubsystem1_Ball_and__T *localB, DW_EnabledSubsystem1_Ball_and_T
   *localDW);

/* private model entry point functions */
extern void Ball_and_Plate_MicroLabBox_student_derivatives(void);

#endif            /* RTW_HEADER_Ball_and_Plate_MicroLabBox_student_private_h_ */
