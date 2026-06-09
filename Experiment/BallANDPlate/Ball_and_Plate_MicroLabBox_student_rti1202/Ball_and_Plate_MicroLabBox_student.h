/*
 * Ball_and_Plate_MicroLabBox_student.h
 *
 * Academic License - for use in teaching, academic research, and meeting
 * course requirements at degree granting institutions only.  Not for
 * government, commercial, or other organizational use.
 *
 * Code generation for model "Ball_and_Plate_MicroLabBox_student".
 *
 * Model version              : 1.74
 * Simulink Coder version : 9.2 (R2019b) 18-Jul-2019
 * C source code generated on : Mon Jun  8 18:07:52 2026
 *
 * Target selection: rti1202.tlc
 * Note: GRT includes extra infrastructure and instrumentation for prototyping
 * Embedded hardware selection: Custom Processor->Custom
 * Code generation objectives: Unspecified
 * Validation result: Not run
 */

#ifndef RTW_HEADER_Ball_and_Plate_MicroLabBox_student_h_
#define RTW_HEADER_Ball_and_Plate_MicroLabBox_student_h_
#include <stddef.h>
#include <string.h>
#include <math.h>
#ifndef Ball_and_Plate_MicroLabBox_student_COMMON_INCLUDES_
# define Ball_and_Plate_MicroLabBox_student_COMMON_INCLUDES_
#include <brtenv.h>
#include <rtkernel.h>
#include <rti_assert.h>
#include <rtidefineddatatypes.h>
#include <dsIoEth.h>
#include "rtwtypes.h"
#include "simstruc.h"
#include "fixedpoint.h"
#endif                 /* Ball_and_Plate_MicroLabBox_student_COMMON_INCLUDES_ */

#include "Ball_and_Plate_MicroLabBox_student_types.h"

/* Shared type includes */
#include "multiword_types.h"
#include "rt_zcfcn.h"
#include "rt_nonfinite.h"
#include "rtGetInf.h"

/* Macros for accessing real-time model data structure */
#ifndef rtmGetContStateDisabled
# define rtmGetContStateDisabled(rtm)  ((rtm)->contStateDisabled)
#endif

#ifndef rtmSetContStateDisabled
# define rtmSetContStateDisabled(rtm, val) ((rtm)->contStateDisabled = (val))
#endif

#ifndef rtmGetContStates
# define rtmGetContStates(rtm)         ((rtm)->contStates)
#endif

#ifndef rtmSetContStates
# define rtmSetContStates(rtm, val)    ((rtm)->contStates = (val))
#endif

#ifndef rtmGetContTimeOutputInconsistentWithStateAtMajorStepFlag
# define rtmGetContTimeOutputInconsistentWithStateAtMajorStepFlag(rtm) ((rtm)->CTOutputIncnstWithState)
#endif

#ifndef rtmSetContTimeOutputInconsistentWithStateAtMajorStepFlag
# define rtmSetContTimeOutputInconsistentWithStateAtMajorStepFlag(rtm, val) ((rtm)->CTOutputIncnstWithState = (val))
#endif

#ifndef rtmGetDerivCacheNeedsReset
# define rtmGetDerivCacheNeedsReset(rtm) ((rtm)->derivCacheNeedsReset)
#endif

#ifndef rtmSetDerivCacheNeedsReset
# define rtmSetDerivCacheNeedsReset(rtm, val) ((rtm)->derivCacheNeedsReset = (val))
#endif

#ifndef rtmGetFinalTime
# define rtmGetFinalTime(rtm)          ((rtm)->Timing.tFinal)
#endif

#ifndef rtmGetIntgData
# define rtmGetIntgData(rtm)           ((rtm)->intgData)
#endif

#ifndef rtmSetIntgData
# define rtmSetIntgData(rtm, val)      ((rtm)->intgData = (val))
#endif

#ifndef rtmGetOdeF
# define rtmGetOdeF(rtm)               ((rtm)->odeF)
#endif

#ifndef rtmSetOdeF
# define rtmSetOdeF(rtm, val)          ((rtm)->odeF = (val))
#endif

#ifndef rtmGetPeriodicContStateIndices
# define rtmGetPeriodicContStateIndices(rtm) ((rtm)->periodicContStateIndices)
#endif

#ifndef rtmSetPeriodicContStateIndices
# define rtmSetPeriodicContStateIndices(rtm, val) ((rtm)->periodicContStateIndices = (val))
#endif

#ifndef rtmGetPeriodicContStateRanges
# define rtmGetPeriodicContStateRanges(rtm) ((rtm)->periodicContStateRanges)
#endif

#ifndef rtmSetPeriodicContStateRanges
# define rtmSetPeriodicContStateRanges(rtm, val) ((rtm)->periodicContStateRanges = (val))
#endif

#ifndef rtmGetSampleHitArray
# define rtmGetSampleHitArray(rtm)     ((rtm)->Timing.sampleHitArray)
#endif

#ifndef rtmGetStepSize
# define rtmGetStepSize(rtm)           ((rtm)->Timing.stepSize)
#endif

#ifndef rtmGetZCCacheNeedsReset
# define rtmGetZCCacheNeedsReset(rtm)  ((rtm)->zCCacheNeedsReset)
#endif

#ifndef rtmSetZCCacheNeedsReset
# define rtmSetZCCacheNeedsReset(rtm, val) ((rtm)->zCCacheNeedsReset = (val))
#endif

#ifndef rtmGet_TimeOfLastOutput
# define rtmGet_TimeOfLastOutput(rtm)  ((rtm)->Timing.timeOfLastOutput)
#endif

#ifndef rtmGetdX
# define rtmGetdX(rtm)                 ((rtm)->derivs)
#endif

#ifndef rtmSetdX
# define rtmSetdX(rtm, val)            ((rtm)->derivs = (val))
#endif

#ifndef rtmGetErrorStatus
# define rtmGetErrorStatus(rtm)        ((rtm)->errorStatus)
#endif

#ifndef rtmSetErrorStatus
# define rtmSetErrorStatus(rtm, val)   ((rtm)->errorStatus = (val))
#endif

#ifndef rtmGetStopRequested
# define rtmGetStopRequested(rtm)      ((rtm)->Timing.stopRequestedFlag)
#endif

#ifndef rtmSetStopRequested
# define rtmSetStopRequested(rtm, val) ((rtm)->Timing.stopRequestedFlag = (val))
#endif

#ifndef rtmGetStopRequestedPtr
# define rtmGetStopRequestedPtr(rtm)   (&((rtm)->Timing.stopRequestedFlag))
#endif

#ifndef rtmGetT
# define rtmGetT(rtm)                  (rtmGetTPtr((rtm))[0])
#endif

#ifndef rtmGetTFinal
# define rtmGetTFinal(rtm)             ((rtm)->Timing.tFinal)
#endif

#ifndef rtmGetTPtr
# define rtmGetTPtr(rtm)               ((rtm)->Timing.t)
#endif

#ifndef rtmGetTStart
# define rtmGetTStart(rtm)             ((rtm)->Timing.tStart)
#endif

#ifndef rtmGetTimeOfLastOutput
# define rtmGetTimeOfLastOutput(rtm)   ((rtm)->Timing.timeOfLastOutput)
#endif

/* Block signals for system '<S21>/MATLAB Function' */
typedef struct {
  real_T y;                            /* '<S21>/MATLAB Function' */
} B_MATLABFunction_Ball_and_Pla_T;

/* Block states (default storage) for system '<S21>/MATLAB Function' */
typedef struct {
  real_T index;                        /* '<S21>/MATLAB Function' */
  real_T previous_enable;              /* '<S21>/MATLAB Function' */
} DW_MATLABFunction_Ball_and_Pl_T;

/* Block signals for system '<S21>/MATLAB Function2' */
typedef struct {
  real_T path;                         /* '<S21>/MATLAB Function2' */
} B_MATLABFunction2_Ball_and_Pl_T;

/* Block signals for system '<S17>/MATLAB Function1' */
typedef struct {
  real_T path;                         /* '<S17>/MATLAB Function1' */
} B_MATLABFunction1_Ball_and_Pl_T;

/* Block states (default storage) for system '<S17>/MATLAB Function1' */
typedef struct {
  real_T coeffs[6];                    /* '<S17>/MATLAB Function1' */
  real_T t_elapsed;                    /* '<S17>/MATLAB Function1' */
  real_T prev_ref_end;                 /* '<S17>/MATLAB Function1' */
  real_T tf_internal;                  /* '<S17>/MATLAB Function1' */
  boolean_T prev_ref_end_not_empty;    /* '<S17>/MATLAB Function1' */
} DW_MATLABFunction1_Ball_and_P_T;

/* Block signals for system '<S23>/MATLAB Function' */
typedef struct {
  real_T y;                            /* '<S23>/MATLAB Function' */
} B_MATLABFunction_Ball_and_P_m_T;

/* Block states (default storage) for system '<S23>/MATLAB Function' */
typedef struct {
  real_T index;                        /* '<S23>/MATLAB Function' */
  real_T previous_enable;              /* '<S23>/MATLAB Function' */
} DW_MATLABFunction_Ball_and__j_T;

/* Block signals for system '<S33>/Enabled Subsystem' */
typedef struct {
  real_T OutportBufferForOut1;         /* '<S34>/Constant' */
} B_EnabledSubsystem_Ball_and_P_T;

/* Block states (default storage) for system '<S33>/Enabled Subsystem' */
typedef struct {
  boolean_T EnabledSubsystem_MODE;     /* '<S33>/Enabled Subsystem' */
} DW_EnabledSubsystem_Ball_and__T;

/* Block signals for system '<S33>/Enabled Subsystem1' */
typedef struct {
  real_T In1;                          /* '<S35>/In1' */
} B_EnabledSubsystem1_Ball_and__T;

/* Block states (default storage) for system '<S33>/Enabled Subsystem1' */
typedef struct {
  boolean_T EnabledSubsystem1_MODE;    /* '<S33>/Enabled Subsystem1' */
} DW_EnabledSubsystem1_Ball_and_T;

/* Block signals (default storage) */
typedef struct {
  real_T RateTransition3;              /* '<Root>/Rate Transition3' */
  real_T Uk1;                          /* '<S1>/UD' */
  real_T Diff;                         /* '<S1>/Diff' */
  real_T Gain;                         /* '<Root>/Gain' */
  real_T Gain2;                        /* '<Root>/Gain2' */
  real_T RateTransition4;              /* '<Root>/Rate Transition4' */
  real_T Gain1;                        /* '<S8>/Gain1' */
  real_T Dct1lowpass2;                 /* '<S8>/Dct1lowpass2' */
  real_T Gain3;                        /* '<Root>/Gain3' */
  real_T RateTransition5;              /* '<Root>/Rate Transition5' */
  real_T Gain1_m;                      /* '<S9>/Gain1' */
  real_T Dct1lowpass2_b;               /* '<S9>/Dct1lowpass2' */
  real_T Switch[4];                    /* '<Root>/Switch' */
  real_T Gain1_j[2];                   /* '<Root>/Gain1' */
  real_T Gain1_c;                      /* '<S10>/Gain1' */
  real_T Dct1lowpass2_b2;              /* '<S10>/Dct1lowpass2' */
  real_T Gain4;                        /* '<Root>/Gain4' */
  real_T Alpha_sat;                    /* '<Root>/Alpha_sat ' */
  real_T Gain1_d;                      /* '<S11>/Gain1' */
  real_T Dct1lowpass2_g;               /* '<S11>/Dct1lowpass2' */
  real_T Gain5;                        /* '<Root>/Gain5' */
  real_T Beta_sat;                     /* '<Root>/Beta_sat ' */
  real_T SFunction1[2];                /* '<S14>/S-Function1' */
  real_T Switch_h;                     /* '<S17>/Switch' */
  real_T enable_ref;                   /* '<S17>/enable_ref ' */
  real_T Sum1;                         /* '<S17>/Sum1' */
  real_T Gain1_b;                      /* '<S26>/Gain1' */
  real_T Dctleadlag2;                  /* '<S26>/Dctleadlag2' */
  real_T Dct1lowpass3;                 /* '<S26>/Dct1lowpass3' */
  real_T Switch2;                      /* '<S17>/Switch2' */
  real_T Sum;                          /* '<S17>/Sum' */
  real_T Switch3;                      /* '<S17>/Switch3' */
  real_T Switch_f;                     /* '<S21>/Switch' */
  real_T Constant1;                    /* '<S21>/Constant1' */
  real_T Constant2;                    /* '<S21>/Constant2' */
  real_T Switch1;                      /* '<S18>/Switch1' */
  real_T Switch_j;                     /* '<S18>/Switch' */
  real_T Sum1_f;                       /* '<S18>/Sum1' */
  real_T Gain1_f;                      /* '<S41>/Gain1' */
  real_T Dctleadlag2_k;                /* '<S41>/Dctleadlag2' */
  real_T Dct1lowpass3_c;               /* '<S41>/Dct1lowpass3' */
  real_T Switch2_p;                    /* '<S18>/Switch2' */
  real_T Sum_g;                        /* '<S18>/Sum' */
  real_T Switch3_d;                    /* '<S18>/Switch3' */
  real_T Switch_n;                     /* '<S36>/Switch' */
  real_T Constant1_c;                  /* '<S36>/Constant1' */
  real_T Constant2_b;                  /* '<S36>/Constant2' */
  real_T Switch1_k;                    /* '<S19>/Switch1' */
  real_T Switch_jo;                    /* '<S19>/Switch' */
  real_T Sum1_h;                       /* '<S19>/Sum1' */
  real_T Gain1_mn;                     /* '<S57>/Gain1' */
  real_T Dctleadlag2_a;                /* '<S57>/Dctleadlag2' */
  real_T Dct1lowpass3_e;               /* '<S57>/Dct1lowpass3' */
  real_T Switch2_h;                    /* '<S19>/Switch2' */
  real_T Sum_h;                        /* '<S19>/Sum' */
  real_T Switch3_l;                    /* '<S19>/Switch3' */
  real_T Switch_e;                     /* '<S51>/Switch' */
  real_T Constant1_h;                  /* '<S51>/Constant1' */
  real_T Constant2_h;                  /* '<S51>/Constant2' */
  real_T reser_integrator;             /* '<S3>/reser_integrator' */
  real_T Alpha_sine;                   /* '<Root>/Alpha_sine ' */
  real_T Beta_sine;                    /* '<Root>/Beta_sine' */
  real_T TmpSignalConversionAtSFunctionI[2];/* '<Root>/MATLAB Function' */
  real_T TmpSignalConversionAtSFunctio_e[2];/* '<Root>/MATLAB Function' */
  real_T x_est[4];                     /* '<Root>/MATLAB Function' */
  real_T alpha;                        /* '<S3>/PosToAngle ' */
  real_T beta;                         /* '<S3>/PosToAngle ' */
  real_T psi;                          /* '<S3>/PosToAngle ' */
  real_T SFunction1_o1;                /* '<S63>/S-Function1' */
  real_T SFunction1_o2;                /* '<S63>/S-Function1' */
  real_T Inc2Pos;                      /* '<S56>/Inc2Pos' */
  real_T AbsPosition;                  /* '<S56>/AbsPosition' */
  real_T Abs;                          /* '<S65>/Abs' */
  real_T TransferFcn;                  /* '<S65>/Transfer Fcn' */
  real_T mm2m;                         /* '<S56>/mm2m' */
  real_T Saturation_i;                 /* '<S55>/Saturation' */
  real_T Current2V;                    /* '<S55>/Current2V' */
  real_T DSPscale;                     /* '<S55>/DSPscale' */
  real_T Add3;                         /* '<S3>/Add3' */
  real_T SFunction1_o1_i;              /* '<S46>/S-Function1' */
  real_T SFunction1_o2_p;              /* '<S46>/S-Function1' */
  real_T Inc2Pos_e;                    /* '<S40>/Inc2Pos' */
  real_T AbsPosition_i;                /* '<S40>/AbsPosition' */
  real_T Abs_j;                        /* '<S48>/Abs' */
  real_T TransferFcn_o;                /* '<S48>/Transfer Fcn' */
  real_T mm2m_p;                       /* '<S40>/mm2m' */
  real_T Saturation_l;                 /* '<S39>/Saturation' */
  real_T Current2V_f;                  /* '<S39>/Current2V' */
  real_T DSPscale_o;                   /* '<S39>/DSPscale' */
  real_T Add2;                         /* '<S3>/Add2' */
  real_T SFunction1_o1_p;              /* '<S31>/S-Function1' */
  real_T SFunction1_o2_d;              /* '<S31>/S-Function1' */
  real_T Inc2Pos_p;                    /* '<S25>/Inc2Pos' */
  real_T AbsPosition_m;                /* '<S25>/AbsPosition' */
  real_T Abs_h;                        /* '<S33>/Abs' */
  real_T TransferFcn_f;                /* '<S33>/Transfer Fcn' */
  real_T mm2m_po;                      /* '<S25>/mm2m' */
  real_T Saturation_a;                 /* '<S24>/Saturation' */
  real_T Current2V_h;                  /* '<S24>/Current2V' */
  real_T DSPscale_m;                   /* '<S24>/DSPscale' */
  real_T Add1;                         /* '<S3>/Add1' */
  real_T pos1;                         /* '<S3>/AngleToPos ' */
  real_T pos2;                         /* '<S3>/AngleToPos ' */
  real_T pos3;                         /* '<S3>/AngleToPos ' */
  real_T x;                            /* '<S2>/MATLAB Function' */
  real_T y;                            /* '<S2>/MATLAB Function' */
  real_T z;                            /* '<S2>/MATLAB Function' */
  uint32_T SFunction1_o4;              /* '<S13>/S-Function1' */
  uint32_T SFunction1_o1_m[3];         /* '<S12>/S-Function1' */
  uint16_T SFunction1_o6;              /* '<S13>/S-Function1' */
  uint8_T SFunction1_o1_c[12];         /* '<S13>/S-Function1' */
  uint8_T SFunction1_o5[4];            /* '<S13>/S-Function1' */
  uint8_T SFunction1_o2_f[4];          /* '<S12>/S-Function1' */
  uint8_T flag;                        /* '<S2>/MATLAB Function' */
  boolean_T DataTypeConversion;        /* '<S2>/Data Type Conversion' */
  boolean_T RelationalOperator1;       /* '<S65>/Relational Operator1' */
  boolean_T RelationalOperator;        /* '<S65>/Relational Operator' */
  boolean_T LogicalOperator2;          /* '<S65>/Logical Operator2' */
  boolean_T RelationalOperator1_c;     /* '<S48>/Relational Operator1' */
  boolean_T RelationalOperator_n;      /* '<S48>/Relational Operator' */
  boolean_T LogicalOperator2_l;        /* '<S48>/Logical Operator2' */
  boolean_T RelationalOperator1_f;     /* '<S33>/Relational Operator1' */
  boolean_T RelationalOperator_a;      /* '<S33>/Relational Operator' */
  boolean_T LogicalOperator2_a;        /* '<S33>/Logical Operator2' */
  B_MATLABFunction1_Ball_and_Pl_T sf_MATLABFunction2_a;/* '<Root>/MATLAB Function2' */
  B_MATLABFunction1_Ball_and_Pl_T sf_MATLABFunction1_m;/* '<Root>/MATLAB Function1' */
  B_EnabledSubsystem1_Ball_and__T EnabledSubsystem1_b;/* '<S65>/Enabled Subsystem1' */
  B_EnabledSubsystem_Ball_and_P_T EnabledSubsystem_n;/* '<S65>/Enabled Subsystem' */
  B_MATLABFunction_Ball_and_P_m_T sf_MATLABFunction_hg;/* '<S54>/MATLAB Function' */
  B_MATLABFunction_Ball_and_P_m_T sf_MATLABFunction_g;/* '<S53>/MATLAB Function' */
  B_MATLABFunction1_Ball_and_Pl_T sf_MATLABFunction1_h;/* '<S19>/MATLAB Function1' */
  B_MATLABFunction2_Ball_and_Pl_T sf_MATLABFunction2_e;/* '<S51>/MATLAB Function2' */
  B_MATLABFunction_Ball_and_Pla_T sf_MATLABFunction_h;/* '<S51>/MATLAB Function' */
  B_EnabledSubsystem1_Ball_and__T EnabledSubsystem1_f;/* '<S48>/Enabled Subsystem1' */
  B_EnabledSubsystem_Ball_and_P_T EnabledSubsystem_a;/* '<S48>/Enabled Subsystem' */
  B_MATLABFunction_Ball_and_P_m_T sf_MATLABFunction_ko;/* '<S38>/MATLAB Function' */
  B_MATLABFunction1_Ball_and_Pl_T sf_MATLABFunction2_d;/* '<S18>/MATLAB Function2' */
  B_MATLABFunction2_Ball_and_Pl_T sf_MATLABFunction2_m;/* '<S36>/MATLAB Function2' */
  B_MATLABFunction_Ball_and_Pla_T sf_MATLABFunction_k;/* '<S36>/MATLAB Function' */
  B_EnabledSubsystem1_Ball_and__T EnabledSubsystem1;/* '<S33>/Enabled Subsystem1' */
  B_EnabledSubsystem_Ball_and_P_T EnabledSubsystem;/* '<S33>/Enabled Subsystem' */
  B_MATLABFunction_Ball_and_P_m_T sf_MATLABFunction_e;/* '<S23>/MATLAB Function' */
  B_MATLABFunction1_Ball_and_Pl_T sf_MATLABFunction1;/* '<S17>/MATLAB Function1' */
  B_MATLABFunction2_Ball_and_Pl_T sf_MATLABFunction2;/* '<S21>/MATLAB Function2' */
  B_MATLABFunction_Ball_and_Pla_T sf_MATLABFunction_i;/* '<S21>/MATLAB Function' */
} B_Ball_and_Plate_MicroLabBox_student_T;

/* Block states (default storage) for system '<Root>' */
typedef struct {
  real_T UD_DSTATE;                    /* '<S1>/UD' */
  volatile real_T RateTransition3_Buffer0;/* '<Root>/Rate Transition3' */
  volatile real_T RateTransition4_Buffer0;/* '<Root>/Rate Transition4' */
  volatile real_T RateTransition5_Buffer0;/* '<Root>/Rate Transition5' */
  real_T x_hat[4];                     /* '<Root>/MATLAB Function' */
  real_T P[16];                        /* '<Root>/MATLAB Function' */
  struct {
    real_T RECEIVED_FRAMES;
  } SFunction1_RWORK;                  /* '<S13>/S-Function1' */

  real_T Dct1lowpass2_RWORK[2];        /* '<S8>/Dct1lowpass2' */
  real_T Dct1lowpass2_RWORK_p[2];      /* '<S9>/Dct1lowpass2' */
  real_T Dct1lowpass2_RWORK_g[2];      /* '<S10>/Dct1lowpass2' */
  real_T Dct1lowpass2_RWORK_gt[2];     /* '<S11>/Dct1lowpass2' */
  struct {
    real_T RX_DROPPED_FRAMES[2];
  } SFunction1_RWORK_l;                /* '<S14>/S-Function1' */

  real_T Dctleadlag2_RWORK[2];         /* '<S26>/Dctleadlag2' */
  real_T Dct1lowpass3_RWORK[2];        /* '<S26>/Dct1lowpass3' */
  real_T Dctleadlag2_RWORK_o[2];       /* '<S41>/Dctleadlag2' */
  real_T Dct1lowpass3_RWORK_b[2];      /* '<S41>/Dct1lowpass3' */
  real_T Dctleadlag2_RWORK_p[2];       /* '<S57>/Dctleadlag2' */
  real_T Dct1lowpass3_RWORK_d[2];      /* '<S57>/Dct1lowpass3' */
  volatile int8_T RateTransition3_semaphoreTaken;/* '<Root>/Rate Transition3' */
  volatile int8_T RateTransition4_semaphoreTaken;/* '<Root>/Rate Transition4' */
  volatile int8_T RateTransition5_semaphoreTaken;/* '<Root>/Rate Transition5' */
  DW_MATLABFunction1_Ball_and_P_T sf_MATLABFunction2_a;/* '<Root>/MATLAB Function2' */
  DW_MATLABFunction1_Ball_and_P_T sf_MATLABFunction1_m;/* '<Root>/MATLAB Function1' */
  DW_EnabledSubsystem1_Ball_and_T EnabledSubsystem1_b;/* '<S65>/Enabled Subsystem1' */
  DW_EnabledSubsystem_Ball_and__T EnabledSubsystem_n;/* '<S65>/Enabled Subsystem' */
  DW_MATLABFunction_Ball_and__j_T sf_MATLABFunction_hg;/* '<S54>/MATLAB Function' */
  DW_MATLABFunction_Ball_and__j_T sf_MATLABFunction_g;/* '<S53>/MATLAB Function' */
  DW_MATLABFunction1_Ball_and_P_T sf_MATLABFunction1_h;/* '<S19>/MATLAB Function1' */
  DW_MATLABFunction_Ball_and_Pl_T sf_MATLABFunction_h;/* '<S51>/MATLAB Function' */
  DW_EnabledSubsystem1_Ball_and_T EnabledSubsystem1_f;/* '<S48>/Enabled Subsystem1' */
  DW_EnabledSubsystem_Ball_and__T EnabledSubsystem_a;/* '<S48>/Enabled Subsystem' */
  DW_MATLABFunction_Ball_and__j_T sf_MATLABFunction_ko;/* '<S38>/MATLAB Function' */
  DW_MATLABFunction1_Ball_and_P_T sf_MATLABFunction2_d;/* '<S18>/MATLAB Function2' */
  DW_MATLABFunction_Ball_and_Pl_T sf_MATLABFunction_k;/* '<S36>/MATLAB Function' */
  DW_EnabledSubsystem1_Ball_and_T EnabledSubsystem1;/* '<S33>/Enabled Subsystem1' */
  DW_EnabledSubsystem_Ball_and__T EnabledSubsystem;/* '<S33>/Enabled Subsystem' */
  DW_MATLABFunction_Ball_and__j_T sf_MATLABFunction_e;/* '<S23>/MATLAB Function' */
  DW_MATLABFunction1_Ball_and_P_T sf_MATLABFunction1;/* '<S17>/MATLAB Function1' */
  DW_MATLABFunction_Ball_and_Pl_T sf_MATLABFunction_i;/* '<S21>/MATLAB Function' */
} DW_Ball_and_Plate_MicroLabBox_student_T;

/* Continuous states (default storage) */
typedef struct {
  real_T TransferFcn_CSTATE;           /* '<S65>/Transfer Fcn' */
  real_T TransferFcn_CSTATE_h;         /* '<S48>/Transfer Fcn' */
  real_T TransferFcn_CSTATE_l;         /* '<S33>/Transfer Fcn' */
} X_Ball_and_Plate_MicroLabBox_student_T;

/* State derivatives (default storage) */
typedef struct {
  real_T TransferFcn_CSTATE;           /* '<S65>/Transfer Fcn' */
  real_T TransferFcn_CSTATE_h;         /* '<S48>/Transfer Fcn' */
  real_T TransferFcn_CSTATE_l;         /* '<S33>/Transfer Fcn' */
} XDot_Ball_and_Plate_MicroLabBox_student_T;

/* State disabled  */
typedef struct {
  boolean_T TransferFcn_CSTATE;        /* '<S65>/Transfer Fcn' */
  boolean_T TransferFcn_CSTATE_h;      /* '<S48>/Transfer Fcn' */
  boolean_T TransferFcn_CSTATE_l;      /* '<S33>/Transfer Fcn' */
} XDis_Ball_and_Plate_MicroLabBox_student_T;

/* Zero-crossing (trigger) state */
typedef struct {
  ZCSigState EMC_ENCODER_POS_SET_BL1_Trig_ZC;/* '<S56>/EMC_ENCODER_POS_SET_BL1' */
  ZCSigState EMC_ENCODER_POS_SET_BL1_Trig__f;/* '<S40>/EMC_ENCODER_POS_SET_BL1' */
  ZCSigState EMC_ENCODER_POS_SET_BL1_Trig_fh;/* '<S25>/EMC_ENCODER_POS_SET_BL1' */
} PrevZCX_Ball_and_Plate_MicroLabBox_student_T;

#ifndef ODE1_INTG
#define ODE1_INTG

/* ODE1 Integration Data */
typedef struct {
  real_T *f[1];                        /* derivatives */
} ODE1_IntgData;

#endif

/* External outputs (root outports fed by signals with default storage) */
typedef struct {
  real_T NumRXFrames;                  /* '<Root>/NumRXFrames' */
  uint32_T Status;                     /* '<Root>/Status' */
  real_T FrameRate;                    /* '<Root>/FrameRate ' */
  real_T x_obs;                        /* '<Root>/x_obs' */
  real_T x_dot_obs;                    /* '<Root>/x_dot_obs' */
  real_T y_obs;                        /* '<Root>/y_obs' */
  real_T y_dot_obs;                    /* '<Root>/y_dot_obs' */
} ExtY_Ball_and_Plate_MicroLabBox_student_T;

/* Parameters for system: '<S33>/Enabled Subsystem' */
struct P_EnabledSubsystem_Ball_and_P_T_ {
  real_T Out1_Y0;                      /* Expression: [0]
                                        * Referenced by: '<S34>/Out1'
                                        */
  real_T Constant_Value;               /* Expression: -1
                                        * Referenced by: '<S34>/Constant'
                                        */
};

/* Parameters for system: '<S33>/Enabled Subsystem1' */
struct P_EnabledSubsystem1_Ball_and__T_ {
  real_T Out1_Y0;                      /* Expression: [0]
                                        * Referenced by: '<S35>/Out1'
                                        */
};

/* Parameters (default storage) */
struct P_Ball_and_Plate_MicroLabBox_student_T_ {
  real_T K_lqr[8];                     /* Variable: K_lqr
                                        * Referenced by: '<Root>/Gain1'
                                        */
  real_T Q_kf[16];                     /* Variable: Q_kf
                                        * Referenced by: '<Root>/Constant'
                                        */
  real_T R_kf[4];                      /* Variable: R_kf
                                        * Referenced by: '<Root>/Constant1'
                                        */
  real_T Ts_Outer;                     /* Variable: Ts_Outer
                                        * Referenced by:
                                        *   '<Root>/Constant2'
                                        *   '<Root>/Constant5'
                                        *   '<Root>/Gain'
                                        */
  real_T Ts_fast;                      /* Variable: Ts_fast
                                        * Referenced by: '<Root>/Constant6'
                                        */
  real_T path[3001];                   /* Variable: path
                                        * Referenced by:
                                        *   '<S21>/Constant4'
                                        *   '<S36>/Constant4'
                                        *   '<S51>/Constant4'
                                        */
  real_T plate_angle_sat;              /* Variable: plate_angle_sat
                                        * Referenced by:
                                        *   '<Root>/Alpha_sat '
                                        *   '<Root>/Beta_sat '
                                        */
  real_T uA[200000];                   /* Variable: uA
                                        * Referenced by:
                                        *   '<S23>/Constant'
                                        *   '<S53>/Constant'
                                        */
  real_T uB[200000];                   /* Variable: uB
                                        * Referenced by: '<S38>/Constant'
                                        */
  real_T uC[200000];                   /* Variable: uC
                                        * Referenced by: '<S54>/Constant'
                                        */
  real_T vel_sat;                      /* Variable: vel_sat
                                        * Referenced by:
                                        *   '<Root>/Saturation'
                                        *   '<Root>/Saturation1'
                                        */
  real_T Difference_ICPrevInput;       /* Mask Parameter: Difference_ICPrevInput
                                        * Referenced by: '<S1>/UD'
                                        */
  real_T Constant3_Value[4];           /* Expression: [0;0;0;0]
                                        * Referenced by: '<Root>/Constant3'
                                        */
  real_T Constant_Value;               /* Expression: 0.32
                                        * Referenced by: '<S3>/Constant'
                                        */
  real_T Constant_Value_h;             /* Expression: 0
                                        * Referenced by: '<S17>/Constant'
                                        */
  real_T Constant3_Value_d;            /* Expression: -0.0289
                                        * Referenced by: '<S17>/Constant3'
                                        */
  real_T Constant3_Value_b;            /* Expression: 0
                                        * Referenced by: '<S21>/Constant3'
                                        */
  real_T Saturation_UpperSat;          /* Expression: 3
                                        * Referenced by: '<S24>/Saturation'
                                        */
  real_T Saturation_LowerSat;          /* Expression: -3
                                        * Referenced by: '<S24>/Saturation'
                                        */
  real_T Current2V_Gain;               /* Expression: 10/3
                                        * Referenced by: '<S24>/Current2V'
                                        */
  real_T DSPscale_Gain;                /* Expression: 1/10
                                        * Referenced by: '<S24>/DSPscale'
                                        */
  real_T Constant_Value_j;             /* Expression: 10
                                        * Referenced by: '<S25>/Constant'
                                        */
  real_T ZP_Value;                     /* Expression: 29
                                        * Referenced by: '<S25>/ZP'
                                        */
  real_T Inc2Pos_Gain;                 /* Expression: 8e-3
                                        * Referenced by: '<S25>/Inc2Pos'
                                        */
  real_T Pos_offset_Value;             /* Expression: 0
                                        * Referenced by: '<S25>/Pos_offset'
                                        */
  real_T Const_Value;                  /* Expression: 0.01
                                        * Referenced by: '<S33>/Const'
                                        */
  real_T Const1_Value;                 /* Expression: -0.98
                                        * Referenced by: '<S33>/Const1'
                                        */
  real_T TransferFcn_A;                /* Computed Parameter: TransferFcn_A
                                        * Referenced by: '<S33>/Transfer Fcn'
                                        */
  real_T TransferFcn_C;                /* Computed Parameter: TransferFcn_C
                                        * Referenced by: '<S33>/Transfer Fcn'
                                        */
  real_T mm2m_Gain;                    /* Expression: 0.001
                                        * Referenced by: '<S25>/mm2m'
                                        */
  real_T Constant1_Value;              /* Expression: 0
                                        * Referenced by: '<S18>/Constant1'
                                        */
  real_T Constant5_Value;              /* Expression: -0.0289
                                        * Referenced by: '<S18>/Constant5'
                                        */
  real_T Constant1_Value_n;            /* Expression: 0.32
                                        * Referenced by: '<S3>/Constant1'
                                        */
  real_T Constant3_Value_i;            /* Expression: 0
                                        * Referenced by: '<S36>/Constant3'
                                        */
  real_T Saturation_UpperSat_e;        /* Expression: 3
                                        * Referenced by: '<S39>/Saturation'
                                        */
  real_T Saturation_LowerSat_f;        /* Expression: -3
                                        * Referenced by: '<S39>/Saturation'
                                        */
  real_T Current2V_Gain_f;             /* Expression: 10/3
                                        * Referenced by: '<S39>/Current2V'
                                        */
  real_T DSPscale_Gain_d;              /* Expression: 1/10
                                        * Referenced by: '<S39>/DSPscale'
                                        */
  real_T Constant_Value_j0;            /* Expression: 10
                                        * Referenced by: '<S40>/Constant'
                                        */
  real_T ZP_Value_m;                   /* Expression: 29
                                        * Referenced by: '<S40>/ZP'
                                        */
  real_T Inc2Pos_Gain_k;               /* Expression: 8e-3
                                        * Referenced by: '<S40>/Inc2Pos'
                                        */
  real_T Pos_offset_Value_k;           /* Expression: 0
                                        * Referenced by: '<S40>/Pos_offset'
                                        */
  real_T Const_Value_g;                /* Expression: 0.01
                                        * Referenced by: '<S48>/Const'
                                        */
  real_T Const1_Value_a;               /* Expression: -0.98
                                        * Referenced by: '<S48>/Const1'
                                        */
  real_T TransferFcn_A_g;              /* Computed Parameter: TransferFcn_A_g
                                        * Referenced by: '<S48>/Transfer Fcn'
                                        */
  real_T TransferFcn_C_e;              /* Computed Parameter: TransferFcn_C_e
                                        * Referenced by: '<S48>/Transfer Fcn'
                                        */
  real_T mm2m_Gain_f;                  /* Expression: 0.001
                                        * Referenced by: '<S40>/mm2m'
                                        */
  real_T Constant_Value_e;             /* Expression: 0
                                        * Referenced by: '<S19>/Constant'
                                        */
  real_T Constant3_Value_n;            /* Expression: -0.0289
                                        * Referenced by: '<S19>/Constant3'
                                        */
  real_T Constant2_Value;              /* Expression: 0.32
                                        * Referenced by: '<S3>/Constant2'
                                        */
  real_T Constant3_Value_p;            /* Expression: 0
                                        * Referenced by: '<S51>/Constant3'
                                        */
  real_T Saturation_UpperSat_h;        /* Expression: 3
                                        * Referenced by: '<S55>/Saturation'
                                        */
  real_T Saturation_LowerSat_p;        /* Expression: -3
                                        * Referenced by: '<S55>/Saturation'
                                        */
  real_T Current2V_Gain_p;             /* Expression: 10/3
                                        * Referenced by: '<S55>/Current2V'
                                        */
  real_T DSPscale_Gain_h;              /* Expression: 1/10
                                        * Referenced by: '<S55>/DSPscale'
                                        */
  real_T Constant_Value_f;             /* Expression: 10
                                        * Referenced by: '<S56>/Constant'
                                        */
  real_T ZP_Value_p;                   /* Expression: 29
                                        * Referenced by: '<S56>/ZP'
                                        */
  real_T Inc2Pos_Gain_h;               /* Expression: 8e-3
                                        * Referenced by: '<S56>/Inc2Pos'
                                        */
  real_T Pos_offset_Value_f;           /* Expression: 0
                                        * Referenced by: '<S56>/Pos_offset'
                                        */
  real_T Const_Value_m;                /* Expression: 0.01
                                        * Referenced by: '<S65>/Const'
                                        */
  real_T Const1_Value_p;               /* Expression: -0.98
                                        * Referenced by: '<S65>/Const1'
                                        */
  real_T TransferFcn_A_m;              /* Computed Parameter: TransferFcn_A_m
                                        * Referenced by: '<S65>/Transfer Fcn'
                                        */
  real_T TransferFcn_C_c;              /* Computed Parameter: TransferFcn_C_c
                                        * Referenced by: '<S65>/Transfer Fcn'
                                        */
  real_T mm2m_Gain_o;                  /* Expression: 0.001
                                        * Referenced by: '<S56>/mm2m'
                                        */
  real_T Constant_Value_i;             /* Expression: 1
                                        * Referenced by: '<S2>/Constant'
                                        */
  real_T Gain2_Gain;                   /* Expression: 1/1000
                                        * Referenced by: '<Root>/Gain2'
                                        */
  real_T Gain1_Gain;                   /* Expression: 1
                                        * Referenced by: '<S8>/Gain1'
                                        */
  real_T Dct1lowpass2_P1_Size[2];    /* Computed Parameter: Dct1lowpass2_P1_Size
                                      * Referenced by: '<S8>/Dct1lowpass2'
                                      */
  real_T Dct1lowpass2_P1;              /* Expression: f_den
                                        * Referenced by: '<S8>/Dct1lowpass2'
                                        */
  real_T Dct1lowpass2_P2_Size[2];    /* Computed Parameter: Dct1lowpass2_P2_Size
                                      * Referenced by: '<S8>/Dct1lowpass2'
                                      */
  real_T Dct1lowpass2_P2;              /* Expression: 0.001
                                        * Referenced by: '<S8>/Dct1lowpass2'
                                        */
  real_T Gain3_Gain;                   /* Expression: 1/1000
                                        * Referenced by: '<Root>/Gain3'
                                        */
  real_T Gain1_Gain_b;                 /* Expression: 1
                                        * Referenced by: '<S9>/Gain1'
                                        */
  real_T Dct1lowpass2_P1_Size_b[2];/* Computed Parameter: Dct1lowpass2_P1_Size_b
                                    * Referenced by: '<S9>/Dct1lowpass2'
                                    */
  real_T Dct1lowpass2_P1_p;            /* Expression: f_den
                                        * Referenced by: '<S9>/Dct1lowpass2'
                                        */
  real_T Dct1lowpass2_P2_Size_n[2];/* Computed Parameter: Dct1lowpass2_P2_Size_n
                                    * Referenced by: '<S9>/Dct1lowpass2'
                                    */
  real_T Dct1lowpass2_P2_h;            /* Expression: 0.001
                                        * Referenced by: '<S9>/Dct1lowpass2'
                                        */
  real_T Init_Value;                   /* Expression: 0
                                        * Referenced by: '<S3>/Init  '
                                        */
  real_T Outer_loop_enable_Value;      /* Expression: 0
                                        * Referenced by: '<Root>/Outer_loop_enable'
                                        */
  real_T Switch_Threshold;             /* Expression: 0
                                        * Referenced by: '<Root>/Switch'
                                        */
  real_T Gain1_Gain_bk;                /* Expression: 1
                                        * Referenced by: '<S10>/Gain1'
                                        */
  real_T Dct1lowpass2_P1_Size_j[2];/* Computed Parameter: Dct1lowpass2_P1_Size_j
                                    * Referenced by: '<S10>/Dct1lowpass2'
                                    */
  real_T Dct1lowpass2_P1_e;            /* Expression: f_den
                                        * Referenced by: '<S10>/Dct1lowpass2'
                                        */
  real_T Dct1lowpass2_P2_Size_g[2];/* Computed Parameter: Dct1lowpass2_P2_Size_g
                                    * Referenced by: '<S10>/Dct1lowpass2'
                                    */
  real_T Dct1lowpass2_P2_k;            /* Expression: 0.001
                                        * Referenced by: '<S10>/Dct1lowpass2'
                                        */
  real_T Gain4_Gain;                   /* Expression: 1
                                        * Referenced by: '<Root>/Gain4'
                                        */
  real_T Gain1_Gain_by;                /* Expression: 1
                                        * Referenced by: '<S11>/Gain1'
                                        */
  real_T Dct1lowpass2_P1_Size_p[2];/* Computed Parameter: Dct1lowpass2_P1_Size_p
                                    * Referenced by: '<S11>/Dct1lowpass2'
                                    */
  real_T Dct1lowpass2_P1_f;            /* Expression: f_den
                                        * Referenced by: '<S11>/Dct1lowpass2'
                                        */
  real_T Dct1lowpass2_P2_Size_c[2];/* Computed Parameter: Dct1lowpass2_P2_Size_c
                                    * Referenced by: '<S11>/Dct1lowpass2'
                                    */
  real_T Dct1lowpass2_P2_b;            /* Expression: 0.001
                                        * Referenced by: '<S11>/Dct1lowpass2'
                                        */
  real_T Gain5_Gain;                   /* Expression: 1
                                        * Referenced by: '<Root>/Gain5'
                                        */
  real_T Psi_ref_Value;                /* Expression: 0
                                        * Referenced by: '<Root>/Psi_ref '
                                        */
  real_T CloseLoop_disable_Value;      /* Expression: 1
                                        * Referenced by: '<S3>/CloseLoop_disable'
                                        */
  real_T controller_disable_Value;     /* Expression: 1
                                        * Referenced by: '<S3>/controller_disable'
                                        */
  real_T enable_quintic_Value;         /* Expression: 1
                                        * Referenced by: '<S3>/enable_quintic  '
                                        */
  real_T quintic_ref_Value;            /* Expression: 0
                                        * Referenced by: '<S3>/quintic_ref '
                                        */
  real_T Switch_Threshold_j;           /* Expression: 0.5
                                        * Referenced by: '<S17>/Switch'
                                        */
  real_T Constant2_Value_j;            /* Expression: 1
                                        * Referenced by: '<S17>/Constant2'
                                        */
  real_T Constant1_Value_na;           /* Expression: 0.001
                                        * Referenced by: '<S17>/Constant1'
                                        */
  real_T enable_ref_Value;             /* Expression: 1
                                        * Referenced by: '<S3>/enable_ref '
                                        */
  real_T enable_ref_Threshold;         /* Expression: 0
                                        * Referenced by: '<S17>/enable_ref '
                                        */
  real_T Gain1_Gain_j;                 /* Expression: 1400
                                        * Referenced by: '<S26>/Gain1'
                                        */
  real_T Dctleadlag2_P1_Size[2];      /* Computed Parameter: Dctleadlag2_P1_Size
                                       * Referenced by: '<S26>/Dctleadlag2'
                                       */
  real_T Dctleadlag2_P1;               /* Expression: f_num
                                        * Referenced by: '<S26>/Dctleadlag2'
                                        */
  real_T Dctleadlag2_P2_Size[2];      /* Computed Parameter: Dctleadlag2_P2_Size
                                       * Referenced by: '<S26>/Dctleadlag2'
                                       */
  real_T Dctleadlag2_P2;               /* Expression: f_den
                                        * Referenced by: '<S26>/Dctleadlag2'
                                        */
  real_T Dctleadlag2_P3_Size[2];      /* Computed Parameter: Dctleadlag2_P3_Size
                                       * Referenced by: '<S26>/Dctleadlag2'
                                       */
  real_T Dctleadlag2_P3;               /* Expression: 0.001
                                        * Referenced by: '<S26>/Dctleadlag2'
                                        */
  real_T Dct1lowpass3_P1_Size[2];    /* Computed Parameter: Dct1lowpass3_P1_Size
                                      * Referenced by: '<S26>/Dct1lowpass3'
                                      */
  real_T Dct1lowpass3_P1;              /* Expression: f_den
                                        * Referenced by: '<S26>/Dct1lowpass3'
                                        */
  real_T Dct1lowpass3_P2_Size[2];    /* Computed Parameter: Dct1lowpass3_P2_Size
                                      * Referenced by: '<S26>/Dct1lowpass3'
                                      */
  real_T Dct1lowpass3_P2;              /* Expression: 0.001
                                        * Referenced by: '<S26>/Dct1lowpass3'
                                        */
  real_T Switch2_Threshold;            /* Expression: 0.5
                                        * Referenced by: '<S17>/Switch2'
                                        */
  real_T enable_ID_A_Value;            /* Expression: 0
                                        * Referenced by: '<S17>/enable_ID_A '
                                        */
  real_T Switch3_Threshold;            /* Expression: 0.5
                                        * Referenced by: '<S17>/Switch3'
                                        */
  real_T u_no_0_init_motion_Value;     /* Expression: 1
                                        * Referenced by: '<S17>/1_no_0_init_motion'
                                        */
  real_T Constant4_Value;              /* Expression: -0.0289
                                        * Referenced by: '<S17>/Constant4'
                                        */
  real_T Switch_Threshold_c;           /* Expression: 0.5
                                        * Referenced by: '<S21>/Switch'
                                        */
  real_T Constant1_Value_i;            /* Expression: 3
                                        * Referenced by: '<S21>/Constant1'
                                        */
  real_T Constant2_Value_e;            /* Expression: 0.001
                                        * Referenced by: '<S21>/Constant2'
                                        */
  real_T Switch1_Threshold;            /* Expression: 0.5
                                        * Referenced by: '<S18>/Switch1'
                                        */
  real_T Constant3_Value_k;            /* Expression: 1
                                        * Referenced by: '<S18>/Constant3'
                                        */
  real_T Constant2_Value_c;            /* Expression: 0.001
                                        * Referenced by: '<S18>/Constant2'
                                        */
  real_T Switch_Threshold_a;           /* Expression: 0
                                        * Referenced by: '<S18>/Switch'
                                        */
  real_T Gain1_Gain_l;                 /* Expression: 1300
                                        * Referenced by: '<S41>/Gain1'
                                        */
  real_T Dctleadlag2_P1_Size_o[2];  /* Computed Parameter: Dctleadlag2_P1_Size_o
                                     * Referenced by: '<S41>/Dctleadlag2'
                                     */
  real_T Dctleadlag2_P1_g;             /* Expression: f_num
                                        * Referenced by: '<S41>/Dctleadlag2'
                                        */
  real_T Dctleadlag2_P2_Size_b[2];  /* Computed Parameter: Dctleadlag2_P2_Size_b
                                     * Referenced by: '<S41>/Dctleadlag2'
                                     */
  real_T Dctleadlag2_P2_b;             /* Expression: f_den
                                        * Referenced by: '<S41>/Dctleadlag2'
                                        */
  real_T Dctleadlag2_P3_Size_f[2];  /* Computed Parameter: Dctleadlag2_P3_Size_f
                                     * Referenced by: '<S41>/Dctleadlag2'
                                     */
  real_T Dctleadlag2_P3_j;             /* Expression: 0.001
                                        * Referenced by: '<S41>/Dctleadlag2'
                                        */
  real_T Dct1lowpass3_P1_Size_m[2];/* Computed Parameter: Dct1lowpass3_P1_Size_m
                                    * Referenced by: '<S41>/Dct1lowpass3'
                                    */
  real_T Dct1lowpass3_P1_a;            /* Expression: f_den
                                        * Referenced by: '<S41>/Dct1lowpass3'
                                        */
  real_T Dct1lowpass3_P2_Size_i[2];/* Computed Parameter: Dct1lowpass3_P2_Size_i
                                    * Referenced by: '<S41>/Dct1lowpass3'
                                    */
  real_T Dct1lowpass3_P2_h;            /* Expression: 0.001
                                        * Referenced by: '<S41>/Dct1lowpass3'
                                        */
  real_T Switch2_Threshold_k;          /* Expression: 0.5
                                        * Referenced by: '<S18>/Switch2'
                                        */
  real_T enable_ID_B_Value;            /* Expression: 0
                                        * Referenced by: '<S18>/enable_ID_B '
                                        */
  real_T Switch3_Threshold_m;          /* Expression: 0.5
                                        * Referenced by: '<S18>/Switch3'
                                        */
  real_T u_no_0_init_motion_Value_e;   /* Expression: 1
                                        * Referenced by: '<S18>/1_no_0_init_motion'
                                        */
  real_T Constant4_Value_b;            /* Expression: -0.0289
                                        * Referenced by: '<S18>/Constant4'
                                        */
  real_T Switch_Threshold_k;           /* Expression: 0.5
                                        * Referenced by: '<S36>/Switch'
                                        */
  real_T Constant1_Value_p;            /* Expression: 3
                                        * Referenced by: '<S36>/Constant1'
                                        */
  real_T Constant2_Value_g;            /* Expression: 0.001
                                        * Referenced by: '<S36>/Constant2'
                                        */
  real_T Switch1_Threshold_j;          /* Expression: 0.5
                                        * Referenced by: '<S19>/Switch1'
                                        */
  real_T Constant2_Value_b;            /* Expression: 1
                                        * Referenced by: '<S19>/Constant2'
                                        */
  real_T Constant1_Value_c;            /* Expression: 0.001
                                        * Referenced by: '<S19>/Constant1'
                                        */
  real_T Switch_Threshold_e;           /* Expression: 0
                                        * Referenced by: '<S19>/Switch'
                                        */
  real_T Gain1_Gain_p;                 /* Expression: 450
                                        * Referenced by: '<S57>/Gain1'
                                        */
  real_T Dctleadlag2_P1_Size_f[2];  /* Computed Parameter: Dctleadlag2_P1_Size_f
                                     * Referenced by: '<S57>/Dctleadlag2'
                                     */
  real_T Dctleadlag2_P1_h;             /* Expression: f_num
                                        * Referenced by: '<S57>/Dctleadlag2'
                                        */
  real_T Dctleadlag2_P2_Size_d[2];  /* Computed Parameter: Dctleadlag2_P2_Size_d
                                     * Referenced by: '<S57>/Dctleadlag2'
                                     */
  real_T Dctleadlag2_P2_l;             /* Expression: f_den
                                        * Referenced by: '<S57>/Dctleadlag2'
                                        */
  real_T Dctleadlag2_P3_Size_p[2];  /* Computed Parameter: Dctleadlag2_P3_Size_p
                                     * Referenced by: '<S57>/Dctleadlag2'
                                     */
  real_T Dctleadlag2_P3_l;             /* Expression: 0.001
                                        * Referenced by: '<S57>/Dctleadlag2'
                                        */
  real_T Dct1lowpass3_P1_Size_d[2];/* Computed Parameter: Dct1lowpass3_P1_Size_d
                                    * Referenced by: '<S57>/Dct1lowpass3'
                                    */
  real_T Dct1lowpass3_P1_g;            /* Expression: f_den
                                        * Referenced by: '<S57>/Dct1lowpass3'
                                        */
  real_T Dct1lowpass3_P2_Size_k[2];/* Computed Parameter: Dct1lowpass3_P2_Size_k
                                    * Referenced by: '<S57>/Dct1lowpass3'
                                    */
  real_T Dct1lowpass3_P2_f;            /* Expression: 0.001
                                        * Referenced by: '<S57>/Dct1lowpass3'
                                        */
  real_T Switch2_Threshold_b;          /* Expression: 0.5
                                        * Referenced by: '<S19>/Switch2'
                                        */
  real_T Enable_ID_C_Value;            /* Expression: 0
                                        * Referenced by: '<S19>/Enable_ID_C '
                                        */
  real_T Switch3_Threshold_g;          /* Expression: 0.5
                                        * Referenced by: '<S19>/Switch3'
                                        */
  real_T u_no_0_init_motion_Value_n;   /* Expression: 1
                                        * Referenced by: '<S19>/1_no_0_init_motion'
                                        */
  real_T Constant4_Value_c;            /* Expression: -0.0289
                                        * Referenced by: '<S19>/Constant4'
                                        */
  real_T Switch_Threshold_l;           /* Expression: 0.5
                                        * Referenced by: '<S51>/Switch'
                                        */
  real_T Constant1_Value_f;            /* Expression: 3
                                        * Referenced by: '<S51>/Constant1'
                                        */
  real_T Constant2_Value_i;            /* Expression: 0.001
                                        * Referenced by: '<S51>/Constant2'
                                        */
  real_T u_to_enable_id_Value;         /* Expression: 0
                                        * Referenced by: '<S19>/1_to_enable_id'
                                        */
  real_T reser_integrator_Value;       /* Expression: 0
                                        * Referenced by: '<S3>/reser_integrator'
                                        */
  real_T Alpha_sine_Amp;               /* Expression: 0.01
                                        * Referenced by: '<Root>/Alpha_sine '
                                        */
  real_T Alpha_sine_Bias;              /* Expression: 0
                                        * Referenced by: '<Root>/Alpha_sine '
                                        */
  real_T Alpha_sine_Freq;              /* Expression: 1
                                        * Referenced by: '<Root>/Alpha_sine '
                                        */
  real_T Alpha_sine_Phase;             /* Expression: 0
                                        * Referenced by: '<Root>/Alpha_sine '
                                        */
  real_T Beta_sine_Amp;                /* Expression: 0.01
                                        * Referenced by: '<Root>/Beta_sine'
                                        */
  real_T Beta_sine_Bias;               /* Expression: 0
                                        * Referenced by: '<Root>/Beta_sine'
                                        */
  real_T Beta_sine_Freq;               /* Expression: 1
                                        * Referenced by: '<Root>/Beta_sine'
                                        */
  real_T Beta_sine_Phase;              /* Expression: 0
                                        * Referenced by: '<Root>/Beta_sine'
                                        */
  P_EnabledSubsystem1_Ball_and__T EnabledSubsystem1_b;/* '<S65>/Enabled Subsystem1' */
  P_EnabledSubsystem_Ball_and_P_T EnabledSubsystem_n;/* '<S65>/Enabled Subsystem' */
  P_EnabledSubsystem1_Ball_and__T EnabledSubsystem1_f;/* '<S48>/Enabled Subsystem1' */
  P_EnabledSubsystem_Ball_and_P_T EnabledSubsystem_a;/* '<S48>/Enabled Subsystem' */
  P_EnabledSubsystem1_Ball_and__T EnabledSubsystem1;/* '<S33>/Enabled Subsystem1' */
  P_EnabledSubsystem_Ball_and_P_T EnabledSubsystem;/* '<S33>/Enabled Subsystem' */
};

/* Real-time Model Data Structure */
struct tag_RTM_Ball_and_Plate_MicroLabBox_student_T {
  struct SimStruct_tag * *childSfunctions;
  const char_T *errorStatus;
  SS_SimMode simMode;
  RTWSolverInfo solverInfo;
  RTWSolverInfo *solverInfoPtr;
  void *sfcnInfo;

  /*
   * NonInlinedSFcns:
   * The following substructure contains information regarding
   * non-inlined s-functions used in the model.
   */
  struct {
    RTWSfcnInfo sfcnInfo;
    time_T *taskTimePtrs[3];
    SimStruct childSFunctions[10];
    SimStruct *childSFunctionPtrs[10];
    struct _ssBlkInfo2 blkInfo2[10];
    struct _ssSFcnModelMethods2 methods2[10];
    struct _ssSFcnModelMethods3 methods3[10];
    struct _ssSFcnModelMethods4 methods4[10];
    struct _ssStatesInfo2 statesInfo2[10];
    ssPeriodicStatesInfo periodicStatesInfo[10];
    struct _ssPortInfo2 inputOutputPortInfo2[10];
    struct {
      time_T sfcnPeriod[1];
      time_T sfcnOffset[1];
      int_T sfcnTsMap[1];
      struct _ssPortInputs inputPortInfo[1];
      struct _ssInPortUnit inputPortUnits[1];
      struct _ssInPortCoSimAttribute inputPortCoSimAttribute[1];
      real_T const *UPtrs0[1];
      struct _ssPortOutputs outputPortInfo[1];
      struct _ssOutPortUnit outputPortUnits[1];
      struct _ssOutPortCoSimAttribute outputPortCoSimAttribute[1];
      uint_T attribs[2];
      mxArray *params[2];
      struct _ssDWorkRecord dWork[1];
      struct _ssDWorkAuxRecord dWorkAux[1];
    } Sfcn0;

    struct {
      time_T sfcnPeriod[1];
      time_T sfcnOffset[1];
      int_T sfcnTsMap[1];
      struct _ssPortInputs inputPortInfo[1];
      struct _ssInPortUnit inputPortUnits[1];
      struct _ssInPortCoSimAttribute inputPortCoSimAttribute[1];
      real_T const *UPtrs0[1];
      struct _ssPortOutputs outputPortInfo[1];
      struct _ssOutPortUnit outputPortUnits[1];
      struct _ssOutPortCoSimAttribute outputPortCoSimAttribute[1];
      uint_T attribs[2];
      mxArray *params[2];
      struct _ssDWorkRecord dWork[1];
      struct _ssDWorkAuxRecord dWorkAux[1];
    } Sfcn1;

    struct {
      time_T sfcnPeriod[1];
      time_T sfcnOffset[1];
      int_T sfcnTsMap[1];
      struct _ssPortInputs inputPortInfo[1];
      struct _ssInPortUnit inputPortUnits[1];
      struct _ssInPortCoSimAttribute inputPortCoSimAttribute[1];
      real_T const *UPtrs0[1];
      struct _ssPortOutputs outputPortInfo[1];
      struct _ssOutPortUnit outputPortUnits[1];
      struct _ssOutPortCoSimAttribute outputPortCoSimAttribute[1];
      uint_T attribs[2];
      mxArray *params[2];
      struct _ssDWorkRecord dWork[1];
      struct _ssDWorkAuxRecord dWorkAux[1];
    } Sfcn2;

    struct {
      time_T sfcnPeriod[1];
      time_T sfcnOffset[1];
      int_T sfcnTsMap[1];
      struct _ssPortInputs inputPortInfo[1];
      struct _ssInPortUnit inputPortUnits[1];
      struct _ssInPortCoSimAttribute inputPortCoSimAttribute[1];
      real_T const *UPtrs0[1];
      struct _ssPortOutputs outputPortInfo[1];
      struct _ssOutPortUnit outputPortUnits[1];
      struct _ssOutPortCoSimAttribute outputPortCoSimAttribute[1];
      uint_T attribs[2];
      mxArray *params[2];
      struct _ssDWorkRecord dWork[1];
      struct _ssDWorkAuxRecord dWorkAux[1];
    } Sfcn3;

    struct {
      time_T sfcnPeriod[1];
      time_T sfcnOffset[1];
      int_T sfcnTsMap[1];
      struct _ssPortInputs inputPortInfo[1];
      struct _ssInPortUnit inputPortUnits[1];
      struct _ssInPortCoSimAttribute inputPortCoSimAttribute[1];
      real_T const *UPtrs0[1];
      struct _ssPortOutputs outputPortInfo[1];
      struct _ssOutPortUnit outputPortUnits[1];
      struct _ssOutPortCoSimAttribute outputPortCoSimAttribute[1];
      uint_T attribs[3];
      mxArray *params[3];
      struct _ssDWorkRecord dWork[1];
      struct _ssDWorkAuxRecord dWorkAux[1];
    } Sfcn4;

    struct {
      time_T sfcnPeriod[1];
      time_T sfcnOffset[1];
      int_T sfcnTsMap[1];
      struct _ssPortInputs inputPortInfo[1];
      struct _ssInPortUnit inputPortUnits[1];
      struct _ssInPortCoSimAttribute inputPortCoSimAttribute[1];
      real_T const *UPtrs0[1];
      struct _ssPortOutputs outputPortInfo[1];
      struct _ssOutPortUnit outputPortUnits[1];
      struct _ssOutPortCoSimAttribute outputPortCoSimAttribute[1];
      uint_T attribs[2];
      mxArray *params[2];
      struct _ssDWorkRecord dWork[1];
      struct _ssDWorkAuxRecord dWorkAux[1];
    } Sfcn5;

    struct {
      time_T sfcnPeriod[1];
      time_T sfcnOffset[1];
      int_T sfcnTsMap[1];
      struct _ssPortInputs inputPortInfo[1];
      struct _ssInPortUnit inputPortUnits[1];
      struct _ssInPortCoSimAttribute inputPortCoSimAttribute[1];
      real_T const *UPtrs0[1];
      struct _ssPortOutputs outputPortInfo[1];
      struct _ssOutPortUnit outputPortUnits[1];
      struct _ssOutPortCoSimAttribute outputPortCoSimAttribute[1];
      uint_T attribs[3];
      mxArray *params[3];
      struct _ssDWorkRecord dWork[1];
      struct _ssDWorkAuxRecord dWorkAux[1];
    } Sfcn6;

    struct {
      time_T sfcnPeriod[1];
      time_T sfcnOffset[1];
      int_T sfcnTsMap[1];
      struct _ssPortInputs inputPortInfo[1];
      struct _ssInPortUnit inputPortUnits[1];
      struct _ssInPortCoSimAttribute inputPortCoSimAttribute[1];
      real_T const *UPtrs0[1];
      struct _ssPortOutputs outputPortInfo[1];
      struct _ssOutPortUnit outputPortUnits[1];
      struct _ssOutPortCoSimAttribute outputPortCoSimAttribute[1];
      uint_T attribs[2];
      mxArray *params[2];
      struct _ssDWorkRecord dWork[1];
      struct _ssDWorkAuxRecord dWorkAux[1];
    } Sfcn7;

    struct {
      time_T sfcnPeriod[1];
      time_T sfcnOffset[1];
      int_T sfcnTsMap[1];
      struct _ssPortInputs inputPortInfo[1];
      struct _ssInPortUnit inputPortUnits[1];
      struct _ssInPortCoSimAttribute inputPortCoSimAttribute[1];
      real_T const *UPtrs0[1];
      struct _ssPortOutputs outputPortInfo[1];
      struct _ssOutPortUnit outputPortUnits[1];
      struct _ssOutPortCoSimAttribute outputPortCoSimAttribute[1];
      uint_T attribs[3];
      mxArray *params[3];
      struct _ssDWorkRecord dWork[1];
      struct _ssDWorkAuxRecord dWorkAux[1];
    } Sfcn8;

    struct {
      time_T sfcnPeriod[1];
      time_T sfcnOffset[1];
      int_T sfcnTsMap[1];
      struct _ssPortInputs inputPortInfo[1];
      struct _ssInPortUnit inputPortUnits[1];
      struct _ssInPortCoSimAttribute inputPortCoSimAttribute[1];
      real_T const *UPtrs0[1];
      struct _ssPortOutputs outputPortInfo[1];
      struct _ssOutPortUnit outputPortUnits[1];
      struct _ssOutPortCoSimAttribute outputPortCoSimAttribute[1];
      uint_T attribs[2];
      mxArray *params[2];
      struct _ssDWorkRecord dWork[1];
      struct _ssDWorkAuxRecord dWorkAux[1];
    } Sfcn9;
  } NonInlinedSFcns;

  X_Ball_and_Plate_MicroLabBox_student_T *contStates;
  int_T *periodicContStateIndices;
  real_T *periodicContStateRanges;
  real_T *derivs;
  boolean_T *contStateDisabled;
  boolean_T zCCacheNeedsReset;
  boolean_T derivCacheNeedsReset;
  boolean_T CTOutputIncnstWithState;
  real_T odeF[1][3];
  ODE1_IntgData intgData;

  /*
   * Sizes:
   * The following substructure contains sizes information
   * for many of the model attributes such as inputs, outputs,
   * dwork, sample times, etc.
   */
  struct {
    uint32_T options;
    int_T numContStates;
    int_T numPeriodicContStates;
    int_T numU;
    int_T numY;
    int_T numSampTimes;
    int_T numBlocks;
    int_T numBlockIO;
    int_T numBlockPrms;
    int_T numDwork;
    int_T numSFcnPrms;
    int_T numSFcns;
    int_T numIports;
    int_T numOports;
    int_T numNonSampZCs;
    int_T sysDirFeedThru;
    int_T rtwGenSfcn;
  } Sizes;

  /*
   * Timing:
   * The following substructure contains information regarding
   * the timing information for the model.
   */
  struct {
    time_T stepSize;
    uint32_T clockTick0;
    uint32_T clockTickH0;
    time_T stepSize0;
    uint32_T clockTick1;
    uint32_T clockTickH1;
    time_T stepSize1;
    struct {
      uint8_T TID[3];
    } TaskCounters;

    time_T tStart;
    time_T tFinal;
    time_T timeOfLastOutput;
    SimTimeStep simTimeStep;
    boolean_T stopRequestedFlag;
    time_T *sampleTimes;
    time_T *offsetTimes;
    int_T *sampleTimeTaskIDPtr;
    int_T *sampleHits;
    int_T *perTaskSampleHits;
    time_T *t;
    time_T sampleTimesArray[3];
    time_T offsetTimesArray[3];
    int_T sampleTimeTaskIDArray[3];
    int_T sampleHitArray[3];
    int_T perTaskSampleHitsArray[9];
    time_T tArray[3];
  } Timing;
};

/* Block parameters (default storage) */
extern P_Ball_and_Plate_MicroLabBox_student_T
  Ball_and_Plate_MicroLabBox_student_P;

/* Block signals (default storage) */
extern B_Ball_and_Plate_MicroLabBox_student_T
  Ball_and_Plate_MicroLabBox_student_B;

/* Continuous states (default storage) */
extern X_Ball_and_Plate_MicroLabBox_student_T
  Ball_and_Plate_MicroLabBox_student_X;

/* Block states (default storage) */
extern DW_Ball_and_Plate_MicroLabBox_student_T
  Ball_and_Plate_MicroLabBox_student_DW;

/* External outputs (root outports fed by signals with default storage) */
extern ExtY_Ball_and_Plate_MicroLabBox_student_T
  Ball_and_Plate_MicroLabBox_student_Y;

/* External data declarations for dependent source files */

/* Zero-crossing (trigger) state */
extern PrevZCX_Ball_and_Plate_MicroLabBox_student_T
  Ball_and_Plate_MicroLabBox_student_PrevZCX;

/* Model entry point functions */
extern void Ball_and_Plate_MicroLabBox_student_initialize(void);
extern void Ball_and_Plate_MicroLabBox_student_output(void);
extern void Ball_and_Plate_MicroLabBox_student_update(void);
extern void Ball_and_Plate_MicroLabBox_student_terminate(void);

/* Real-time Model object */
extern RT_MODEL_Ball_and_Plate_MicroLabBox_student_T *const
  Ball_and_Plate_MicroLabBox_student_M;

/*-
 * The generated code includes comments that allow you to trace directly
 * back to the appropriate location in the model.  The basic format
 * is <system>/block_name, where system is the system number (uniquely
 * assigned by Simulink) and block_name is the name of the block.
 *
 * Use the MATLAB hilite_system command to trace the generated code back
 * to the model.  For example,
 *
 * hilite_system('<S3>')    - opens system 3
 * hilite_system('<S3>/Kp') - opens and selects block Kp which resides in S3
 *
 * Here is the system hierarchy for this model
 *
 * '<Root>' : 'Ball_and_Plate_MicroLabBox_student'
 * '<S1>'   : 'Ball_and_Plate_MicroLabBox_student/Difference'
 * '<S2>'   : 'Ball_and_Plate_MicroLabBox_student/Ethernet communication'
 * '<S3>'   : 'Ball_and_Plate_MicroLabBox_student/Innerloop_Actuator'
 * '<S4>'   : 'Ball_and_Plate_MicroLabBox_student/MATLAB Function'
 * '<S5>'   : 'Ball_and_Plate_MicroLabBox_student/MATLAB Function1'
 * '<S6>'   : 'Ball_and_Plate_MicroLabBox_student/MATLAB Function2'
 * '<S7>'   : 'Ball_and_Plate_MicroLabBox_student/RTI Data'
 * '<S8>'   : 'Ball_and_Plate_MicroLabBox_student/lowpass '
 * '<S9>'   : 'Ball_and_Plate_MicroLabBox_student/lowpass 1'
 * '<S10>'  : 'Ball_and_Plate_MicroLabBox_student/lowpass 2'
 * '<S11>'  : 'Ball_and_Plate_MicroLabBox_student/lowpass 3'
 * '<S12>'  : 'Ball_and_Plate_MicroLabBox_student/Ethernet communication/ETHERNET_SETUP_BL1'
 * '<S13>'  : 'Ball_and_Plate_MicroLabBox_student/Ethernet communication/ETHERNET_UDP_RX_BL1'
 * '<S14>'  : 'Ball_and_Plate_MicroLabBox_student/Ethernet communication/ETHERNET_UDP_SETUP_BL1'
 * '<S15>'  : 'Ball_and_Plate_MicroLabBox_student/Ethernet communication/MATLAB Function'
 * '<S16>'  : 'Ball_and_Plate_MicroLabBox_student/Innerloop_Actuator/AngleToPos '
 * '<S17>'  : 'Ball_and_Plate_MicroLabBox_student/Innerloop_Actuator/Motor_A'
 * '<S18>'  : 'Ball_and_Plate_MicroLabBox_student/Innerloop_Actuator/Motor_B'
 * '<S19>'  : 'Ball_and_Plate_MicroLabBox_student/Innerloop_Actuator/Motor_C'
 * '<S20>'  : 'Ball_and_Plate_MicroLabBox_student/Innerloop_Actuator/PosToAngle '
 * '<S21>'  : 'Ball_and_Plate_MicroLabBox_student/Innerloop_Actuator/Motor_A/Init_motion'
 * '<S22>'  : 'Ball_and_Plate_MicroLabBox_student/Innerloop_Actuator/Motor_A/MATLAB Function1'
 * '<S23>'  : 'Ball_and_Plate_MicroLabBox_student/Innerloop_Actuator/Motor_A/Multisine'
 * '<S24>'  : 'Ball_and_Plate_MicroLabBox_student/Innerloop_Actuator/Motor_A/Outputs to Amplifier'
 * '<S25>'  : 'Ball_and_Plate_MicroLabBox_student/Innerloop_Actuator/Motor_A/Position Measurement'
 * '<S26>'  : 'Ball_and_Plate_MicroLabBox_student/Innerloop_Actuator/Motor_A/motorA '
 * '<S27>'  : 'Ball_and_Plate_MicroLabBox_student/Innerloop_Actuator/Motor_A/Init_motion/MATLAB Function'
 * '<S28>'  : 'Ball_and_Plate_MicroLabBox_student/Innerloop_Actuator/Motor_A/Init_motion/MATLAB Function2'
 * '<S29>'  : 'Ball_and_Plate_MicroLabBox_student/Innerloop_Actuator/Motor_A/Multisine/MATLAB Function'
 * '<S30>'  : 'Ball_and_Plate_MicroLabBox_student/Innerloop_Actuator/Motor_A/Outputs to Amplifier/DAC_CLASS1_BL1'
 * '<S31>'  : 'Ball_and_Plate_MicroLabBox_student/Innerloop_Actuator/Motor_A/Position Measurement/EMC_ENCODER_BL1'
 * '<S32>'  : 'Ball_and_Plate_MicroLabBox_student/Innerloop_Actuator/Motor_A/Position Measurement/EMC_ENCODER_POS_SET_BL1'
 * '<S33>'  : 'Ball_and_Plate_MicroLabBox_student/Innerloop_Actuator/Motor_A/Position Measurement/Initial'
 * '<S34>'  : 'Ball_and_Plate_MicroLabBox_student/Innerloop_Actuator/Motor_A/Position Measurement/Initial/Enabled Subsystem'
 * '<S35>'  : 'Ball_and_Plate_MicroLabBox_student/Innerloop_Actuator/Motor_A/Position Measurement/Initial/Enabled Subsystem1'
 * '<S36>'  : 'Ball_and_Plate_MicroLabBox_student/Innerloop_Actuator/Motor_B/Init_motion'
 * '<S37>'  : 'Ball_and_Plate_MicroLabBox_student/Innerloop_Actuator/Motor_B/MATLAB Function2'
 * '<S38>'  : 'Ball_and_Plate_MicroLabBox_student/Innerloop_Actuator/Motor_B/Multisine'
 * '<S39>'  : 'Ball_and_Plate_MicroLabBox_student/Innerloop_Actuator/Motor_B/Outputs to Amplifier'
 * '<S40>'  : 'Ball_and_Plate_MicroLabBox_student/Innerloop_Actuator/Motor_B/Position Measurement'
 * '<S41>'  : 'Ball_and_Plate_MicroLabBox_student/Innerloop_Actuator/Motor_B/motorB'
 * '<S42>'  : 'Ball_and_Plate_MicroLabBox_student/Innerloop_Actuator/Motor_B/Init_motion/MATLAB Function'
 * '<S43>'  : 'Ball_and_Plate_MicroLabBox_student/Innerloop_Actuator/Motor_B/Init_motion/MATLAB Function2'
 * '<S44>'  : 'Ball_and_Plate_MicroLabBox_student/Innerloop_Actuator/Motor_B/Multisine/MATLAB Function'
 * '<S45>'  : 'Ball_and_Plate_MicroLabBox_student/Innerloop_Actuator/Motor_B/Outputs to Amplifier/DAC_CLASS1_BL1'
 * '<S46>'  : 'Ball_and_Plate_MicroLabBox_student/Innerloop_Actuator/Motor_B/Position Measurement/EMC_ENCODER_BL1'
 * '<S47>'  : 'Ball_and_Plate_MicroLabBox_student/Innerloop_Actuator/Motor_B/Position Measurement/EMC_ENCODER_POS_SET_BL1'
 * '<S48>'  : 'Ball_and_Plate_MicroLabBox_student/Innerloop_Actuator/Motor_B/Position Measurement/Initial'
 * '<S49>'  : 'Ball_and_Plate_MicroLabBox_student/Innerloop_Actuator/Motor_B/Position Measurement/Initial/Enabled Subsystem'
 * '<S50>'  : 'Ball_and_Plate_MicroLabBox_student/Innerloop_Actuator/Motor_B/Position Measurement/Initial/Enabled Subsystem1'
 * '<S51>'  : 'Ball_and_Plate_MicroLabBox_student/Innerloop_Actuator/Motor_C/Init_motion'
 * '<S52>'  : 'Ball_and_Plate_MicroLabBox_student/Innerloop_Actuator/Motor_C/MATLAB Function1'
 * '<S53>'  : 'Ball_and_Plate_MicroLabBox_student/Innerloop_Actuator/Motor_C/Multisine'
 * '<S54>'  : 'Ball_and_Plate_MicroLabBox_student/Innerloop_Actuator/Motor_C/Multisine1'
 * '<S55>'  : 'Ball_and_Plate_MicroLabBox_student/Innerloop_Actuator/Motor_C/Outputs to Amplifier'
 * '<S56>'  : 'Ball_and_Plate_MicroLabBox_student/Innerloop_Actuator/Motor_C/Position Measurement'
 * '<S57>'  : 'Ball_and_Plate_MicroLabBox_student/Innerloop_Actuator/Motor_C/motorC'
 * '<S58>'  : 'Ball_and_Plate_MicroLabBox_student/Innerloop_Actuator/Motor_C/Init_motion/MATLAB Function'
 * '<S59>'  : 'Ball_and_Plate_MicroLabBox_student/Innerloop_Actuator/Motor_C/Init_motion/MATLAB Function2'
 * '<S60>'  : 'Ball_and_Plate_MicroLabBox_student/Innerloop_Actuator/Motor_C/Multisine/MATLAB Function'
 * '<S61>'  : 'Ball_and_Plate_MicroLabBox_student/Innerloop_Actuator/Motor_C/Multisine1/MATLAB Function'
 * '<S62>'  : 'Ball_and_Plate_MicroLabBox_student/Innerloop_Actuator/Motor_C/Outputs to Amplifier/DAC_CLASS1_BL1'
 * '<S63>'  : 'Ball_and_Plate_MicroLabBox_student/Innerloop_Actuator/Motor_C/Position Measurement/EMC_ENCODER_BL1'
 * '<S64>'  : 'Ball_and_Plate_MicroLabBox_student/Innerloop_Actuator/Motor_C/Position Measurement/EMC_ENCODER_POS_SET_BL1'
 * '<S65>'  : 'Ball_and_Plate_MicroLabBox_student/Innerloop_Actuator/Motor_C/Position Measurement/Initial'
 * '<S66>'  : 'Ball_and_Plate_MicroLabBox_student/Innerloop_Actuator/Motor_C/Position Measurement/Initial/Enabled Subsystem'
 * '<S67>'  : 'Ball_and_Plate_MicroLabBox_student/Innerloop_Actuator/Motor_C/Position Measurement/Initial/Enabled Subsystem1'
 * '<S68>'  : 'Ball_and_Plate_MicroLabBox_student/RTI Data/RTI Data Store'
 * '<S69>'  : 'Ball_and_Plate_MicroLabBox_student/RTI Data/RTI Data Store/RTI Data Store'
 * '<S70>'  : 'Ball_and_Plate_MicroLabBox_student/RTI Data/RTI Data Store/RTI Data Store/RTI Data Store'
 */
#endif                    /* RTW_HEADER_Ball_and_Plate_MicroLabBox_student_h_ */
