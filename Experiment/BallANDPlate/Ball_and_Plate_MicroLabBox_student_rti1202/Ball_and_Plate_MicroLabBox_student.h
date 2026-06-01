/*
 * Ball_and_Plate_MicroLabBox_student.h
 *
 * Academic License - for use in teaching, academic research, and meeting
 * course requirements at degree granting institutions only.  Not for
 * government, commercial, or other organizational use.
 *
 * Code generation for model "Ball_and_Plate_MicroLabBox_student".
 *
 * Model version              : 1.64
 * Simulink Coder version : 9.2 (R2019b) 18-Jul-2019
 * C source code generated on : Mon Jun  1 17:18:51 2026
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

/* Block signals for system '<S16>/MATLAB Function' */
typedef struct {
  real_T y;                            /* '<S16>/MATLAB Function' */
} B_MATLABFunction_Ball_and_Pla_T;

/* Block states (default storage) for system '<S16>/MATLAB Function' */
typedef struct {
  real_T index;                        /* '<S16>/MATLAB Function' */
  real_T previous_enable;              /* '<S16>/MATLAB Function' */
} DW_MATLABFunction_Ball_and_Pl_T;

/* Block signals for system '<S16>/MATLAB Function2' */
typedef struct {
  real_T path;                         /* '<S16>/MATLAB Function2' */
} B_MATLABFunction2_Ball_and_Pl_T;

/* Block signals for system '<S12>/MATLAB Function1' */
typedef struct {
  real_T path;                         /* '<S12>/MATLAB Function1' */
} B_MATLABFunction1_Ball_and_Pl_T;

/* Block states (default storage) for system '<S12>/MATLAB Function1' */
typedef struct {
  real_T coeffs[6];                    /* '<S12>/MATLAB Function1' */
  real_T t_elapsed;                    /* '<S12>/MATLAB Function1' */
  real_T prev_ref_end;                 /* '<S12>/MATLAB Function1' */
  real_T tf_internal;                  /* '<S12>/MATLAB Function1' */
  boolean_T prev_ref_end_not_empty;    /* '<S12>/MATLAB Function1' */
} DW_MATLABFunction1_Ball_and_P_T;

/* Block signals for system '<S18>/MATLAB Function' */
typedef struct {
  real_T y;                            /* '<S18>/MATLAB Function' */
} B_MATLABFunction_Ball_and_P_m_T;

/* Block states (default storage) for system '<S18>/MATLAB Function' */
typedef struct {
  real_T index;                        /* '<S18>/MATLAB Function' */
  real_T previous_enable;              /* '<S18>/MATLAB Function' */
} DW_MATLABFunction_Ball_and__j_T;

/* Block signals for system '<S28>/Enabled Subsystem' */
typedef struct {
  real_T OutportBufferForOut1;         /* '<S29>/Constant' */
} B_EnabledSubsystem_Ball_and_P_T;

/* Block states (default storage) for system '<S28>/Enabled Subsystem' */
typedef struct {
  boolean_T EnabledSubsystem_MODE;     /* '<S28>/Enabled Subsystem' */
} DW_EnabledSubsystem_Ball_and__T;

/* Block signals for system '<S28>/Enabled Subsystem1' */
typedef struct {
  real_T In1;                          /* '<S30>/In1' */
} B_EnabledSubsystem1_Ball_and__T;

/* Block states (default storage) for system '<S28>/Enabled Subsystem1' */
typedef struct {
  boolean_T EnabledSubsystem1_MODE;    /* '<S28>/Enabled Subsystem1' */
} DW_EnabledSubsystem1_Ball_and_T;

/* Block signals (default storage) */
typedef struct {
  real_T RateTransition3;              /* '<Root>/Rate Transition3' */
  real_T Uk1;                          /* '<S1>/UD' */
  real_T Diff;                         /* '<S1>/Diff' */
  real_T Gain;                         /* '<Root>/Gain' */
  real_T Observer[4];                  /* '<Root>/Observer ' */
  real_T Saturation1;                  /* '<Root>/Saturation1' */
  real_T Saturation;                   /* '<Root>/Saturation' */
  real_T DiscreteStateSpace[2];        /* '<Root>/Discrete State-Space' */
  real_T Alpha_sat;                    /* '<Root>/Alpha_sat ' */
  real_T Beta_sat;                     /* '<Root>/Beta_sat ' */
  real_T SFunction1[2];                /* '<S9>/S-Function1' */
  real_T Add1;                         /* '<S3>/Add1' */
  real_T Add2;                         /* '<S3>/Add2' */
  real_T Add3;                         /* '<S3>/Add3' */
  real_T Switch;                       /* '<S12>/Switch' */
  real_T enable_ref;                   /* '<S12>/enable_ref ' */
  real_T Sum1;                         /* '<S12>/Sum1' */
  real_T Gain1;                        /* '<S21>/Gain1' */
  real_T Dctleadlag2;                  /* '<S21>/Dctleadlag2' */
  real_T Dct1lowpass3;                 /* '<S21>/Dct1lowpass3' */
  real_T Switch2;                      /* '<S12>/Switch2' */
  real_T Sum;                          /* '<S12>/Sum' */
  real_T Switch3;                      /* '<S12>/Switch3' */
  real_T Switch_f;                     /* '<S16>/Switch' */
  real_T Constant1;                    /* '<S16>/Constant1' */
  real_T Constant2;                    /* '<S16>/Constant2' */
  real_T Switch1;                      /* '<S13>/Switch1' */
  real_T Switch_j;                     /* '<S13>/Switch' */
  real_T Sum1_f;                       /* '<S13>/Sum1' */
  real_T Gain1_f;                      /* '<S36>/Gain1' */
  real_T Dctleadlag2_k;                /* '<S36>/Dctleadlag2' */
  real_T Dct1lowpass3_c;               /* '<S36>/Dct1lowpass3' */
  real_T Switch2_p;                    /* '<S13>/Switch2' */
  real_T Sum_g;                        /* '<S13>/Sum' */
  real_T Switch3_d;                    /* '<S13>/Switch3' */
  real_T Switch_n;                     /* '<S31>/Switch' */
  real_T Constant1_c;                  /* '<S31>/Constant1' */
  real_T Constant2_b;                  /* '<S31>/Constant2' */
  real_T Switch1_k;                    /* '<S14>/Switch1' */
  real_T Switch_jo;                    /* '<S14>/Switch' */
  real_T Sum1_h;                       /* '<S14>/Sum1' */
  real_T Gain1_m;                      /* '<S52>/Gain1' */
  real_T Dctleadlag2_a;                /* '<S52>/Dctleadlag2' */
  real_T Dct1lowpass3_e;               /* '<S52>/Dct1lowpass3' */
  real_T Switch2_h;                    /* '<S14>/Switch2' */
  real_T Sum_h;                        /* '<S14>/Sum' */
  real_T Switch3_l;                    /* '<S14>/Switch3' */
  real_T Switch_e;                     /* '<S46>/Switch' */
  real_T Constant1_h;                  /* '<S46>/Constant1' */
  real_T Constant2_h;                  /* '<S46>/Constant2' */
  real_T reser_integrator;             /* '<S3>/reser_integrator' */
  real_T RateTransition4;              /* '<Root>/Rate Transition4' */
  real_T RateTransition5;              /* '<Root>/Rate Transition5' */
  real_T Switch_d[4];                  /* '<Root>/Switch' */
  real_T Gain1_p;                      /* '<S5>/Gain1' */
  real_T Dct1lowpass2;                 /* '<S5>/Dct1lowpass2' */
  real_T Gain1_i;                      /* '<S6>/Gain1' */
  real_T Dct1lowpass2_g;               /* '<S6>/Dct1lowpass2' */
  real_T TmpSignalConversionAtObserverIn[4];
  real_T Alpha_sine;                   /* '<Root>/Alpha_sine ' */
  real_T Beta_sine;                    /* '<Root>/Beta_sine' */
  real_T alpha;                        /* '<S3>/PosToAngle ' */
  real_T beta;                         /* '<S3>/PosToAngle ' */
  real_T psi;                          /* '<S3>/PosToAngle ' */
  real_T SFunction1_o1;                /* '<S58>/S-Function1' */
  real_T SFunction1_o2;                /* '<S58>/S-Function1' */
  real_T Inc2Pos;                      /* '<S51>/Inc2Pos' */
  real_T AbsPosition;                  /* '<S51>/AbsPosition' */
  real_T Abs;                          /* '<S60>/Abs' */
  real_T TransferFcn;                  /* '<S60>/Transfer Fcn' */
  real_T mm2m;                         /* '<S51>/mm2m' */
  real_T Saturation_i;                 /* '<S50>/Saturation' */
  real_T Current2V;                    /* '<S50>/Current2V' */
  real_T DSPscale;                     /* '<S50>/DSPscale' */
  real_T SFunction1_o1_i;              /* '<S41>/S-Function1' */
  real_T SFunction1_o2_p;              /* '<S41>/S-Function1' */
  real_T Inc2Pos_e;                    /* '<S35>/Inc2Pos' */
  real_T AbsPosition_i;                /* '<S35>/AbsPosition' */
  real_T Abs_j;                        /* '<S43>/Abs' */
  real_T TransferFcn_o;                /* '<S43>/Transfer Fcn' */
  real_T mm2m_p;                       /* '<S35>/mm2m' */
  real_T Saturation_l;                 /* '<S34>/Saturation' */
  real_T Current2V_f;                  /* '<S34>/Current2V' */
  real_T DSPscale_o;                   /* '<S34>/DSPscale' */
  real_T SFunction1_o1_p;              /* '<S26>/S-Function1' */
  real_T SFunction1_o2_d;              /* '<S26>/S-Function1' */
  real_T Inc2Pos_p;                    /* '<S20>/Inc2Pos' */
  real_T AbsPosition_m;                /* '<S20>/AbsPosition' */
  real_T Abs_h;                        /* '<S28>/Abs' */
  real_T TransferFcn_f;                /* '<S28>/Transfer Fcn' */
  real_T mm2m_po;                      /* '<S20>/mm2m' */
  real_T Saturation_a;                 /* '<S19>/Saturation' */
  real_T Current2V_h;                  /* '<S19>/Current2V' */
  real_T DSPscale_m;                   /* '<S19>/DSPscale' */
  real_T pos1;                         /* '<S3>/AngleToPos ' */
  real_T pos2;                         /* '<S3>/AngleToPos ' */
  real_T pos3;                         /* '<S3>/AngleToPos ' */
  real_T x;                            /* '<S2>/MATLAB Function' */
  real_T y;                            /* '<S2>/MATLAB Function' */
  real_T z;                            /* '<S2>/MATLAB Function' */
  uint32_T SFunction1_o4;              /* '<S8>/S-Function1' */
  uint32_T SFunction1_o1_m[3];         /* '<S7>/S-Function1' */
  uint16_T SFunction1_o6;              /* '<S8>/S-Function1' */
  uint8_T SFunction1_o1_c[12];         /* '<S8>/S-Function1' */
  uint8_T SFunction1_o5[4];            /* '<S8>/S-Function1' */
  uint8_T SFunction1_o2_f[4];          /* '<S7>/S-Function1' */
  boolean_T DataTypeConversion;        /* '<S2>/Data Type Conversion' */
  boolean_T RelationalOperator1;       /* '<S60>/Relational Operator1' */
  boolean_T RelationalOperator;        /* '<S60>/Relational Operator' */
  boolean_T LogicalOperator2;          /* '<S60>/Logical Operator2' */
  boolean_T RelationalOperator1_c;     /* '<S43>/Relational Operator1' */
  boolean_T RelationalOperator_n;      /* '<S43>/Relational Operator' */
  boolean_T LogicalOperator2_l;        /* '<S43>/Logical Operator2' */
  boolean_T RelationalOperator1_f;     /* '<S28>/Relational Operator1' */
  boolean_T RelationalOperator_a;      /* '<S28>/Relational Operator' */
  boolean_T LogicalOperator2_a;        /* '<S28>/Logical Operator2' */
  B_EnabledSubsystem1_Ball_and__T EnabledSubsystem1_b;/* '<S60>/Enabled Subsystem1' */
  B_EnabledSubsystem_Ball_and_P_T EnabledSubsystem_n;/* '<S60>/Enabled Subsystem' */
  B_MATLABFunction_Ball_and_P_m_T sf_MATLABFunction_hg;/* '<S49>/MATLAB Function' */
  B_MATLABFunction_Ball_and_P_m_T sf_MATLABFunction_g;/* '<S48>/MATLAB Function' */
  B_MATLABFunction1_Ball_and_Pl_T sf_MATLABFunction1_h;/* '<S14>/MATLAB Function1' */
  B_MATLABFunction2_Ball_and_Pl_T sf_MATLABFunction2_e;/* '<S46>/MATLAB Function2' */
  B_MATLABFunction_Ball_and_Pla_T sf_MATLABFunction_h;/* '<S46>/MATLAB Function' */
  B_EnabledSubsystem1_Ball_and__T EnabledSubsystem1_f;/* '<S43>/Enabled Subsystem1' */
  B_EnabledSubsystem_Ball_and_P_T EnabledSubsystem_a;/* '<S43>/Enabled Subsystem' */
  B_MATLABFunction_Ball_and_P_m_T sf_MATLABFunction_ko;/* '<S33>/MATLAB Function' */
  B_MATLABFunction1_Ball_and_Pl_T sf_MATLABFunction2_d;/* '<S13>/MATLAB Function2' */
  B_MATLABFunction2_Ball_and_Pl_T sf_MATLABFunction2_m;/* '<S31>/MATLAB Function2' */
  B_MATLABFunction_Ball_and_Pla_T sf_MATLABFunction_k;/* '<S31>/MATLAB Function' */
  B_EnabledSubsystem1_Ball_and__T EnabledSubsystem1;/* '<S28>/Enabled Subsystem1' */
  B_EnabledSubsystem_Ball_and_P_T EnabledSubsystem;/* '<S28>/Enabled Subsystem' */
  B_MATLABFunction_Ball_and_P_m_T sf_MATLABFunction_e;/* '<S18>/MATLAB Function' */
  B_MATLABFunction1_Ball_and_Pl_T sf_MATLABFunction1;/* '<S12>/MATLAB Function1' */
  B_MATLABFunction2_Ball_and_Pl_T sf_MATLABFunction2;/* '<S16>/MATLAB Function2' */
  B_MATLABFunction_Ball_and_Pla_T sf_MATLABFunction_i;/* '<S16>/MATLAB Function' */
} B_Ball_and_Plate_MicroLabBox_student_T;

/* Block states (default storage) for system '<Root>' */
typedef struct {
  real_T UD_DSTATE;                    /* '<S1>/UD' */
  real_T Observer_DSTATE[4];           /* '<Root>/Observer ' */
  real_T DiscreteStateSpace_DSTATE[14];/* '<Root>/Discrete State-Space' */
  volatile real_T RateTransition3_Buffer0;/* '<Root>/Rate Transition3' */
  volatile real_T RateTransition4_Buffer0;/* '<Root>/Rate Transition4' */
  volatile real_T RateTransition5_Buffer0;/* '<Root>/Rate Transition5' */
  struct {
    real_T RECEIVED_FRAMES;
  } SFunction1_RWORK;                  /* '<S8>/S-Function1' */

  struct {
    real_T RX_DROPPED_FRAMES[2];
  } SFunction1_RWORK_l;                /* '<S9>/S-Function1' */

  real_T Dctleadlag2_RWORK[2];         /* '<S21>/Dctleadlag2' */
  real_T Dct1lowpass3_RWORK[2];        /* '<S21>/Dct1lowpass3' */
  real_T Dctleadlag2_RWORK_o[2];       /* '<S36>/Dctleadlag2' */
  real_T Dct1lowpass3_RWORK_b[2];      /* '<S36>/Dct1lowpass3' */
  real_T Dctleadlag2_RWORK_p[2];       /* '<S52>/Dctleadlag2' */
  real_T Dct1lowpass3_RWORK_d[2];      /* '<S52>/Dct1lowpass3' */
  real_T Dct1lowpass2_RWORK[2];        /* '<S5>/Dct1lowpass2' */
  real_T Dct1lowpass2_RWORK_d[2];      /* '<S6>/Dct1lowpass2' */
  volatile int8_T RateTransition3_semaphoreTaken;/* '<Root>/Rate Transition3' */
  volatile int8_T RateTransition4_semaphoreTaken;/* '<Root>/Rate Transition4' */
  volatile int8_T RateTransition5_semaphoreTaken;/* '<Root>/Rate Transition5' */
  DW_EnabledSubsystem1_Ball_and_T EnabledSubsystem1_b;/* '<S60>/Enabled Subsystem1' */
  DW_EnabledSubsystem_Ball_and__T EnabledSubsystem_n;/* '<S60>/Enabled Subsystem' */
  DW_MATLABFunction_Ball_and__j_T sf_MATLABFunction_hg;/* '<S49>/MATLAB Function' */
  DW_MATLABFunction_Ball_and__j_T sf_MATLABFunction_g;/* '<S48>/MATLAB Function' */
  DW_MATLABFunction1_Ball_and_P_T sf_MATLABFunction1_h;/* '<S14>/MATLAB Function1' */
  DW_MATLABFunction_Ball_and_Pl_T sf_MATLABFunction_h;/* '<S46>/MATLAB Function' */
  DW_EnabledSubsystem1_Ball_and_T EnabledSubsystem1_f;/* '<S43>/Enabled Subsystem1' */
  DW_EnabledSubsystem_Ball_and__T EnabledSubsystem_a;/* '<S43>/Enabled Subsystem' */
  DW_MATLABFunction_Ball_and__j_T sf_MATLABFunction_ko;/* '<S33>/MATLAB Function' */
  DW_MATLABFunction1_Ball_and_P_T sf_MATLABFunction2_d;/* '<S13>/MATLAB Function2' */
  DW_MATLABFunction_Ball_and_Pl_T sf_MATLABFunction_k;/* '<S31>/MATLAB Function' */
  DW_EnabledSubsystem1_Ball_and_T EnabledSubsystem1;/* '<S28>/Enabled Subsystem1' */
  DW_EnabledSubsystem_Ball_and__T EnabledSubsystem;/* '<S28>/Enabled Subsystem' */
  DW_MATLABFunction_Ball_and__j_T sf_MATLABFunction_e;/* '<S18>/MATLAB Function' */
  DW_MATLABFunction1_Ball_and_P_T sf_MATLABFunction1;/* '<S12>/MATLAB Function1' */
  DW_MATLABFunction_Ball_and_Pl_T sf_MATLABFunction_i;/* '<S16>/MATLAB Function' */
} DW_Ball_and_Plate_MicroLabBox_student_T;

/* Continuous states (default storage) */
typedef struct {
  real_T TransferFcn_CSTATE;           /* '<S60>/Transfer Fcn' */
  real_T TransferFcn_CSTATE_h;         /* '<S43>/Transfer Fcn' */
  real_T TransferFcn_CSTATE_l;         /* '<S28>/Transfer Fcn' */
} X_Ball_and_Plate_MicroLabBox_student_T;

/* State derivatives (default storage) */
typedef struct {
  real_T TransferFcn_CSTATE;           /* '<S60>/Transfer Fcn' */
  real_T TransferFcn_CSTATE_h;         /* '<S43>/Transfer Fcn' */
  real_T TransferFcn_CSTATE_l;         /* '<S28>/Transfer Fcn' */
} XDot_Ball_and_Plate_MicroLabBox_student_T;

/* State disabled  */
typedef struct {
  boolean_T TransferFcn_CSTATE;        /* '<S60>/Transfer Fcn' */
  boolean_T TransferFcn_CSTATE_h;      /* '<S43>/Transfer Fcn' */
  boolean_T TransferFcn_CSTATE_l;      /* '<S28>/Transfer Fcn' */
} XDis_Ball_and_Plate_MicroLabBox_student_T;

/* Zero-crossing (trigger) state */
typedef struct {
  ZCSigState EMC_ENCODER_POS_SET_BL1_Trig_ZC;/* '<S51>/EMC_ENCODER_POS_SET_BL1' */
  ZCSigState EMC_ENCODER_POS_SET_BL1_Trig__f;/* '<S35>/EMC_ENCODER_POS_SET_BL1' */
  ZCSigState EMC_ENCODER_POS_SET_BL1_Trig_fh;/* '<S20>/EMC_ENCODER_POS_SET_BL1' */
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

/* Parameters for system: '<S28>/Enabled Subsystem' */
struct P_EnabledSubsystem_Ball_and_P_T_ {
  real_T Out1_Y0;                      /* Expression: [0]
                                        * Referenced by: '<S29>/Out1'
                                        */
  real_T Constant_Value;               /* Expression: -1
                                        * Referenced by: '<S29>/Constant'
                                        */
};

/* Parameters for system: '<S28>/Enabled Subsystem1' */
struct P_EnabledSubsystem1_Ball_and__T_ {
  real_T Out1_Y0;                      /* Expression: [0]
                                        * Referenced by: '<S30>/Out1'
                                        */
};

/* Parameters (default storage) */
struct P_Ball_and_Plate_MicroLabBox_student_T_ {
  real_T A_obs[16];                    /* Variable: A_obs
                                        * Referenced by: '<Root>/Observer '
                                        */
  real_T A_robust[196];                /* Variable: A_robust
                                        * Referenced by: '<Root>/Discrete State-Space'
                                        */
  real_T B_obs[16];                    /* Variable: B_obs
                                        * Referenced by: '<Root>/Observer '
                                        */
  real_T B_robust[56];                 /* Variable: B_robust
                                        * Referenced by: '<Root>/Discrete State-Space'
                                        */
  real_T C_obs[16];                    /* Variable: C_obs
                                        * Referenced by: '<Root>/Observer '
                                        */
  real_T C_robust[28];                 /* Variable: C_robust
                                        * Referenced by: '<Root>/Discrete State-Space'
                                        */
  real_T D_obs[16];                    /* Variable: D_obs
                                        * Referenced by: '<Root>/Observer '
                                        */
  real_T D_robust[8];                  /* Variable: D_robust
                                        * Referenced by: '<Root>/Discrete State-Space'
                                        */
  real_T Ts_Outer;                     /* Variable: Ts_Outer
                                        * Referenced by: '<Root>/Gain'
                                        */
  real_T path[3001];                   /* Variable: path
                                        * Referenced by:
                                        *   '<S16>/Constant4'
                                        *   '<S31>/Constant4'
                                        *   '<S46>/Constant4'
                                        */
  real_T plate_angle_sat;              /* Variable: plate_angle_sat
                                        * Referenced by:
                                        *   '<Root>/Alpha_sat '
                                        *   '<Root>/Beta_sat '
                                        */
  real_T uA[200000];                   /* Variable: uA
                                        * Referenced by:
                                        *   '<S18>/Constant'
                                        *   '<S48>/Constant'
                                        */
  real_T uB[200000];                   /* Variable: uB
                                        * Referenced by: '<S33>/Constant'
                                        */
  real_T uC[200000];                   /* Variable: uC
                                        * Referenced by: '<S49>/Constant'
                                        */
  real_T vel_sat;                      /* Variable: vel_sat
                                        * Referenced by:
                                        *   '<Root>/Saturation'
                                        *   '<Root>/Saturation1'
                                        */
  real_T Difference_ICPrevInput;       /* Mask Parameter: Difference_ICPrevInput
                                        * Referenced by: '<S1>/UD'
                                        */
  real_T Constant_Value;               /* Expression: 0
                                        * Referenced by: '<S12>/Constant'
                                        */
  real_T Constant3_Value;              /* Expression: -0.0289
                                        * Referenced by: '<S12>/Constant3'
                                        */
  real_T Constant3_Value_b;            /* Expression: 0
                                        * Referenced by: '<S16>/Constant3'
                                        */
  real_T Saturation_UpperSat;          /* Expression: 3
                                        * Referenced by: '<S19>/Saturation'
                                        */
  real_T Saturation_LowerSat;          /* Expression: -3
                                        * Referenced by: '<S19>/Saturation'
                                        */
  real_T Current2V_Gain;               /* Expression: 10/3
                                        * Referenced by: '<S19>/Current2V'
                                        */
  real_T DSPscale_Gain;                /* Expression: 1/10
                                        * Referenced by: '<S19>/DSPscale'
                                        */
  real_T Constant_Value_j;             /* Expression: 10
                                        * Referenced by: '<S20>/Constant'
                                        */
  real_T ZP_Value;                     /* Expression: 29
                                        * Referenced by: '<S20>/ZP'
                                        */
  real_T Inc2Pos_Gain;                 /* Expression: 8e-3
                                        * Referenced by: '<S20>/Inc2Pos'
                                        */
  real_T Pos_offset_Value;             /* Expression: 0
                                        * Referenced by: '<S20>/Pos_offset'
                                        */
  real_T Const_Value;                  /* Expression: 0.01
                                        * Referenced by: '<S28>/Const'
                                        */
  real_T Const1_Value;                 /* Expression: -0.98
                                        * Referenced by: '<S28>/Const1'
                                        */
  real_T TransferFcn_A;                /* Computed Parameter: TransferFcn_A
                                        * Referenced by: '<S28>/Transfer Fcn'
                                        */
  real_T TransferFcn_C;                /* Computed Parameter: TransferFcn_C
                                        * Referenced by: '<S28>/Transfer Fcn'
                                        */
  real_T mm2m_Gain;                    /* Expression: 0.001
                                        * Referenced by: '<S20>/mm2m'
                                        */
  real_T Constant1_Value;              /* Expression: 0
                                        * Referenced by: '<S13>/Constant1'
                                        */
  real_T Constant5_Value;              /* Expression: -0.0289
                                        * Referenced by: '<S13>/Constant5'
                                        */
  real_T Constant3_Value_i;            /* Expression: 0
                                        * Referenced by: '<S31>/Constant3'
                                        */
  real_T Saturation_UpperSat_e;        /* Expression: 3
                                        * Referenced by: '<S34>/Saturation'
                                        */
  real_T Saturation_LowerSat_f;        /* Expression: -3
                                        * Referenced by: '<S34>/Saturation'
                                        */
  real_T Current2V_Gain_f;             /* Expression: 10/3
                                        * Referenced by: '<S34>/Current2V'
                                        */
  real_T DSPscale_Gain_d;              /* Expression: 1/10
                                        * Referenced by: '<S34>/DSPscale'
                                        */
  real_T Constant_Value_j0;            /* Expression: 10
                                        * Referenced by: '<S35>/Constant'
                                        */
  real_T ZP_Value_m;                   /* Expression: 29
                                        * Referenced by: '<S35>/ZP'
                                        */
  real_T Inc2Pos_Gain_k;               /* Expression: 8e-3
                                        * Referenced by: '<S35>/Inc2Pos'
                                        */
  real_T Pos_offset_Value_k;           /* Expression: 0
                                        * Referenced by: '<S35>/Pos_offset'
                                        */
  real_T Const_Value_g;                /* Expression: 0.01
                                        * Referenced by: '<S43>/Const'
                                        */
  real_T Const1_Value_a;               /* Expression: -0.98
                                        * Referenced by: '<S43>/Const1'
                                        */
  real_T TransferFcn_A_g;              /* Computed Parameter: TransferFcn_A_g
                                        * Referenced by: '<S43>/Transfer Fcn'
                                        */
  real_T TransferFcn_C_e;              /* Computed Parameter: TransferFcn_C_e
                                        * Referenced by: '<S43>/Transfer Fcn'
                                        */
  real_T mm2m_Gain_f;                  /* Expression: 0.001
                                        * Referenced by: '<S35>/mm2m'
                                        */
  real_T Constant_Value_e;             /* Expression: 0
                                        * Referenced by: '<S14>/Constant'
                                        */
  real_T Constant3_Value_n;            /* Expression: -0.0289
                                        * Referenced by: '<S14>/Constant3'
                                        */
  real_T Constant3_Value_p;            /* Expression: 0
                                        * Referenced by: '<S46>/Constant3'
                                        */
  real_T Saturation_UpperSat_h;        /* Expression: 3
                                        * Referenced by: '<S50>/Saturation'
                                        */
  real_T Saturation_LowerSat_p;        /* Expression: -3
                                        * Referenced by: '<S50>/Saturation'
                                        */
  real_T Current2V_Gain_p;             /* Expression: 10/3
                                        * Referenced by: '<S50>/Current2V'
                                        */
  real_T DSPscale_Gain_h;              /* Expression: 1/10
                                        * Referenced by: '<S50>/DSPscale'
                                        */
  real_T Constant_Value_f;             /* Expression: 10
                                        * Referenced by: '<S51>/Constant'
                                        */
  real_T ZP_Value_p;                   /* Expression: 29
                                        * Referenced by: '<S51>/ZP'
                                        */
  real_T Inc2Pos_Gain_h;               /* Expression: 8e-3
                                        * Referenced by: '<S51>/Inc2Pos'
                                        */
  real_T Pos_offset_Value_f;           /* Expression: 0
                                        * Referenced by: '<S51>/Pos_offset'
                                        */
  real_T Const_Value_m;                /* Expression: 0.01
                                        * Referenced by: '<S60>/Const'
                                        */
  real_T Const1_Value_p;               /* Expression: -0.98
                                        * Referenced by: '<S60>/Const1'
                                        */
  real_T TransferFcn_A_m;              /* Computed Parameter: TransferFcn_A_m
                                        * Referenced by: '<S60>/Transfer Fcn'
                                        */
  real_T TransferFcn_C_c;              /* Computed Parameter: TransferFcn_C_c
                                        * Referenced by: '<S60>/Transfer Fcn'
                                        */
  real_T mm2m_Gain_o;                  /* Expression: 0.001
                                        * Referenced by: '<S51>/mm2m'
                                        */
  real_T Constant_Value_i;             /* Expression: 1
                                        * Referenced by: '<S2>/Constant'
                                        */
  real_T Observer_InitialCondition;    /* Expression: 0
                                        * Referenced by: '<Root>/Observer '
                                        */
  real_T Saturation1_LowerSat;         /* Expression: -0.5
                                        * Referenced by: '<Root>/Saturation1'
                                        */
  real_T Saturation_LowerSat_k;        /* Expression: -0.5
                                        * Referenced by: '<Root>/Saturation'
                                        */
  real_T DiscreteStateSpace_InitialCondi;/* Expression: 0
                                          * Referenced by: '<Root>/Discrete State-Space'
                                          */
  real_T Alpha_sat_LowerSat;           /* Expression: -0.5
                                        * Referenced by: '<Root>/Alpha_sat '
                                        */
  real_T Beta_sat_LowerSat;            /* Expression: -0.5
                                        * Referenced by: '<Root>/Beta_sat '
                                        */
  real_T Psi_ref_Value;                /* Expression: 0
                                        * Referenced by: '<Root>/Psi_ref '
                                        */
  real_T Constant_Value_o;             /* Expression: 0.32
                                        * Referenced by: '<S3>/Constant'
                                        */
  real_T Constant1_Value_n;            /* Expression: 0.32
                                        * Referenced by: '<S3>/Constant1'
                                        */
  real_T Constant2_Value;              /* Expression: 0.32
                                        * Referenced by: '<S3>/Constant2'
                                        */
  real_T CloseLoop_disable_Value;      /* Expression: 1
                                        * Referenced by: '<S3>/CloseLoop_disable'
                                        */
  real_T Init_Value;                   /* Expression: 0
                                        * Referenced by: '<S3>/Init  '
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
  real_T Switch_Threshold;             /* Expression: 0.5
                                        * Referenced by: '<S12>/Switch'
                                        */
  real_T Constant2_Value_j;            /* Expression: 1
                                        * Referenced by: '<S12>/Constant2'
                                        */
  real_T Constant1_Value_na;           /* Expression: 0.001
                                        * Referenced by: '<S12>/Constant1'
                                        */
  real_T enable_ref_Value;             /* Expression: 1
                                        * Referenced by: '<S3>/enable_ref '
                                        */
  real_T enable_ref_Threshold;         /* Expression: 0
                                        * Referenced by: '<S12>/enable_ref '
                                        */
  real_T Gain1_Gain;                   /* Expression: 1400
                                        * Referenced by: '<S21>/Gain1'
                                        */
  real_T Dctleadlag2_P1_Size[2];      /* Computed Parameter: Dctleadlag2_P1_Size
                                       * Referenced by: '<S21>/Dctleadlag2'
                                       */
  real_T Dctleadlag2_P1;               /* Expression: f_num
                                        * Referenced by: '<S21>/Dctleadlag2'
                                        */
  real_T Dctleadlag2_P2_Size[2];      /* Computed Parameter: Dctleadlag2_P2_Size
                                       * Referenced by: '<S21>/Dctleadlag2'
                                       */
  real_T Dctleadlag2_P2;               /* Expression: f_den
                                        * Referenced by: '<S21>/Dctleadlag2'
                                        */
  real_T Dctleadlag2_P3_Size[2];      /* Computed Parameter: Dctleadlag2_P3_Size
                                       * Referenced by: '<S21>/Dctleadlag2'
                                       */
  real_T Dctleadlag2_P3;               /* Expression: 0.001
                                        * Referenced by: '<S21>/Dctleadlag2'
                                        */
  real_T Dct1lowpass3_P1_Size[2];    /* Computed Parameter: Dct1lowpass3_P1_Size
                                      * Referenced by: '<S21>/Dct1lowpass3'
                                      */
  real_T Dct1lowpass3_P1;              /* Expression: f_den
                                        * Referenced by: '<S21>/Dct1lowpass3'
                                        */
  real_T Dct1lowpass3_P2_Size[2];    /* Computed Parameter: Dct1lowpass3_P2_Size
                                      * Referenced by: '<S21>/Dct1lowpass3'
                                      */
  real_T Dct1lowpass3_P2;              /* Expression: 0.001
                                        * Referenced by: '<S21>/Dct1lowpass3'
                                        */
  real_T Switch2_Threshold;            /* Expression: 0.5
                                        * Referenced by: '<S12>/Switch2'
                                        */
  real_T enable_ID_A_Value;            /* Expression: 0
                                        * Referenced by: '<S12>/enable_ID_A '
                                        */
  real_T Switch3_Threshold;            /* Expression: 0.5
                                        * Referenced by: '<S12>/Switch3'
                                        */
  real_T u_no_0_init_motion_Value;     /* Expression: 1
                                        * Referenced by: '<S12>/1_no_0_init_motion'
                                        */
  real_T Constant4_Value;              /* Expression: -0.0289
                                        * Referenced by: '<S12>/Constant4'
                                        */
  real_T Switch_Threshold_c;           /* Expression: 0.5
                                        * Referenced by: '<S16>/Switch'
                                        */
  real_T Constant1_Value_i;            /* Expression: 3
                                        * Referenced by: '<S16>/Constant1'
                                        */
  real_T Constant2_Value_e;            /* Expression: 0.001
                                        * Referenced by: '<S16>/Constant2'
                                        */
  real_T Switch1_Threshold;            /* Expression: 0.5
                                        * Referenced by: '<S13>/Switch1'
                                        */
  real_T Constant3_Value_k;            /* Expression: 1
                                        * Referenced by: '<S13>/Constant3'
                                        */
  real_T Constant2_Value_c;            /* Expression: 0.001
                                        * Referenced by: '<S13>/Constant2'
                                        */
  real_T Switch_Threshold_a;           /* Expression: 0
                                        * Referenced by: '<S13>/Switch'
                                        */
  real_T Gain1_Gain_l;                 /* Expression: 1300
                                        * Referenced by: '<S36>/Gain1'
                                        */
  real_T Dctleadlag2_P1_Size_o[2];  /* Computed Parameter: Dctleadlag2_P1_Size_o
                                     * Referenced by: '<S36>/Dctleadlag2'
                                     */
  real_T Dctleadlag2_P1_g;             /* Expression: f_num
                                        * Referenced by: '<S36>/Dctleadlag2'
                                        */
  real_T Dctleadlag2_P2_Size_b[2];  /* Computed Parameter: Dctleadlag2_P2_Size_b
                                     * Referenced by: '<S36>/Dctleadlag2'
                                     */
  real_T Dctleadlag2_P2_b;             /* Expression: f_den
                                        * Referenced by: '<S36>/Dctleadlag2'
                                        */
  real_T Dctleadlag2_P3_Size_f[2];  /* Computed Parameter: Dctleadlag2_P3_Size_f
                                     * Referenced by: '<S36>/Dctleadlag2'
                                     */
  real_T Dctleadlag2_P3_j;             /* Expression: 0.001
                                        * Referenced by: '<S36>/Dctleadlag2'
                                        */
  real_T Dct1lowpass3_P1_Size_m[2];/* Computed Parameter: Dct1lowpass3_P1_Size_m
                                    * Referenced by: '<S36>/Dct1lowpass3'
                                    */
  real_T Dct1lowpass3_P1_a;            /* Expression: f_den
                                        * Referenced by: '<S36>/Dct1lowpass3'
                                        */
  real_T Dct1lowpass3_P2_Size_i[2];/* Computed Parameter: Dct1lowpass3_P2_Size_i
                                    * Referenced by: '<S36>/Dct1lowpass3'
                                    */
  real_T Dct1lowpass3_P2_h;            /* Expression: 0.001
                                        * Referenced by: '<S36>/Dct1lowpass3'
                                        */
  real_T Switch2_Threshold_k;          /* Expression: 0.5
                                        * Referenced by: '<S13>/Switch2'
                                        */
  real_T enable_ID_B_Value;            /* Expression: 0
                                        * Referenced by: '<S13>/enable_ID_B '
                                        */
  real_T Switch3_Threshold_m;          /* Expression: 0.5
                                        * Referenced by: '<S13>/Switch3'
                                        */
  real_T u_no_0_init_motion_Value_e;   /* Expression: 1
                                        * Referenced by: '<S13>/1_no_0_init_motion'
                                        */
  real_T Constant4_Value_b;            /* Expression: -0.0289
                                        * Referenced by: '<S13>/Constant4'
                                        */
  real_T Switch_Threshold_k;           /* Expression: 0.5
                                        * Referenced by: '<S31>/Switch'
                                        */
  real_T Constant1_Value_p;            /* Expression: 3
                                        * Referenced by: '<S31>/Constant1'
                                        */
  real_T Constant2_Value_g;            /* Expression: 0.001
                                        * Referenced by: '<S31>/Constant2'
                                        */
  real_T Switch1_Threshold_j;          /* Expression: 0.5
                                        * Referenced by: '<S14>/Switch1'
                                        */
  real_T Constant2_Value_b;            /* Expression: 1
                                        * Referenced by: '<S14>/Constant2'
                                        */
  real_T Constant1_Value_c;            /* Expression: 0.001
                                        * Referenced by: '<S14>/Constant1'
                                        */
  real_T Switch_Threshold_e;           /* Expression: 0
                                        * Referenced by: '<S14>/Switch'
                                        */
  real_T Gain1_Gain_p;                 /* Expression: 450
                                        * Referenced by: '<S52>/Gain1'
                                        */
  real_T Dctleadlag2_P1_Size_f[2];  /* Computed Parameter: Dctleadlag2_P1_Size_f
                                     * Referenced by: '<S52>/Dctleadlag2'
                                     */
  real_T Dctleadlag2_P1_h;             /* Expression: f_num
                                        * Referenced by: '<S52>/Dctleadlag2'
                                        */
  real_T Dctleadlag2_P2_Size_d[2];  /* Computed Parameter: Dctleadlag2_P2_Size_d
                                     * Referenced by: '<S52>/Dctleadlag2'
                                     */
  real_T Dctleadlag2_P2_l;             /* Expression: f_den
                                        * Referenced by: '<S52>/Dctleadlag2'
                                        */
  real_T Dctleadlag2_P3_Size_p[2];  /* Computed Parameter: Dctleadlag2_P3_Size_p
                                     * Referenced by: '<S52>/Dctleadlag2'
                                     */
  real_T Dctleadlag2_P3_l;             /* Expression: 0.001
                                        * Referenced by: '<S52>/Dctleadlag2'
                                        */
  real_T Dct1lowpass3_P1_Size_d[2];/* Computed Parameter: Dct1lowpass3_P1_Size_d
                                    * Referenced by: '<S52>/Dct1lowpass3'
                                    */
  real_T Dct1lowpass3_P1_g;            /* Expression: f_den
                                        * Referenced by: '<S52>/Dct1lowpass3'
                                        */
  real_T Dct1lowpass3_P2_Size_k[2];/* Computed Parameter: Dct1lowpass3_P2_Size_k
                                    * Referenced by: '<S52>/Dct1lowpass3'
                                    */
  real_T Dct1lowpass3_P2_f;            /* Expression: 0.001
                                        * Referenced by: '<S52>/Dct1lowpass3'
                                        */
  real_T Switch2_Threshold_b;          /* Expression: 0.5
                                        * Referenced by: '<S14>/Switch2'
                                        */
  real_T Enable_ID_C_Value;            /* Expression: 0
                                        * Referenced by: '<S14>/Enable_ID_C '
                                        */
  real_T Switch3_Threshold_g;          /* Expression: 0.5
                                        * Referenced by: '<S14>/Switch3'
                                        */
  real_T u_no_0_init_motion_Value_n;   /* Expression: 1
                                        * Referenced by: '<S14>/1_no_0_init_motion'
                                        */
  real_T Constant4_Value_c;            /* Expression: -0.0289
                                        * Referenced by: '<S14>/Constant4'
                                        */
  real_T Switch_Threshold_l;           /* Expression: 0.5
                                        * Referenced by: '<S46>/Switch'
                                        */
  real_T Constant1_Value_f;            /* Expression: 3
                                        * Referenced by: '<S46>/Constant1'
                                        */
  real_T Constant2_Value_i;            /* Expression: 0.001
                                        * Referenced by: '<S46>/Constant2'
                                        */
  real_T u_to_enable_id_Value;         /* Expression: 0
                                        * Referenced by: '<S14>/1_to_enable_id'
                                        */
  real_T reser_integrator_Value;       /* Expression: 0
                                        * Referenced by: '<S3>/reser_integrator'
                                        */
  real_T Outer_loop_enable_Value;      /* Expression: 0
                                        * Referenced by: '<Root>/Outer_loop_enable'
                                        */
  real_T Switch_Threshold_h;           /* Expression: 0
                                        * Referenced by: '<Root>/Switch'
                                        */
  real_T Gain1_Gain_a;                 /* Expression: 1
                                        * Referenced by: '<S5>/Gain1'
                                        */
  real_T Dct1lowpass2_P1_Size[2];    /* Computed Parameter: Dct1lowpass2_P1_Size
                                      * Referenced by: '<S5>/Dct1lowpass2'
                                      */
  real_T Dct1lowpass2_P1;              /* Expression: f_den
                                        * Referenced by: '<S5>/Dct1lowpass2'
                                        */
  real_T Dct1lowpass2_P2_Size[2];    /* Computed Parameter: Dct1lowpass2_P2_Size
                                      * Referenced by: '<S5>/Dct1lowpass2'
                                      */
  real_T Dct1lowpass2_P2;              /* Expression: 0.001
                                        * Referenced by: '<S5>/Dct1lowpass2'
                                        */
  real_T Gain1_Gain_g;                 /* Expression: 1
                                        * Referenced by: '<S6>/Gain1'
                                        */
  real_T Dct1lowpass2_P1_Size_b[2];/* Computed Parameter: Dct1lowpass2_P1_Size_b
                                    * Referenced by: '<S6>/Dct1lowpass2'
                                    */
  real_T Dct1lowpass2_P1_d;            /* Expression: f_den
                                        * Referenced by: '<S6>/Dct1lowpass2'
                                        */
  real_T Dct1lowpass2_P2_Size_l[2];/* Computed Parameter: Dct1lowpass2_P2_Size_l
                                    * Referenced by: '<S6>/Dct1lowpass2'
                                    */
  real_T Dct1lowpass2_P2_k;            /* Expression: 0.001
                                        * Referenced by: '<S6>/Dct1lowpass2'
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
  P_EnabledSubsystem1_Ball_and__T EnabledSubsystem1_b;/* '<S60>/Enabled Subsystem1' */
  P_EnabledSubsystem_Ball_and_P_T EnabledSubsystem_n;/* '<S60>/Enabled Subsystem' */
  P_EnabledSubsystem1_Ball_and__T EnabledSubsystem1_f;/* '<S43>/Enabled Subsystem1' */
  P_EnabledSubsystem_Ball_and_P_T EnabledSubsystem_a;/* '<S43>/Enabled Subsystem' */
  P_EnabledSubsystem1_Ball_and__T EnabledSubsystem1;/* '<S28>/Enabled Subsystem1' */
  P_EnabledSubsystem_Ball_and_P_T EnabledSubsystem;/* '<S28>/Enabled Subsystem' */
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
    SimStruct childSFunctions[8];
    SimStruct *childSFunctionPtrs[8];
    struct _ssBlkInfo2 blkInfo2[8];
    struct _ssSFcnModelMethods2 methods2[8];
    struct _ssSFcnModelMethods3 methods3[8];
    struct _ssSFcnModelMethods4 methods4[8];
    struct _ssStatesInfo2 statesInfo2[8];
    ssPeriodicStatesInfo periodicStatesInfo[8];
    struct _ssPortInfo2 inputOutputPortInfo2[8];
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
      uint_T attribs[3];
      mxArray *params[3];
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
      uint_T attribs[2];
      mxArray *params[2];
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
 * '<S4>'   : 'Ball_and_Plate_MicroLabBox_student/RTI Data'
 * '<S5>'   : 'Ball_and_Plate_MicroLabBox_student/lowpass '
 * '<S6>'   : 'Ball_and_Plate_MicroLabBox_student/lowpass 1'
 * '<S7>'   : 'Ball_and_Plate_MicroLabBox_student/Ethernet communication/ETHERNET_SETUP_BL1'
 * '<S8>'   : 'Ball_and_Plate_MicroLabBox_student/Ethernet communication/ETHERNET_UDP_RX_BL1'
 * '<S9>'   : 'Ball_and_Plate_MicroLabBox_student/Ethernet communication/ETHERNET_UDP_SETUP_BL1'
 * '<S10>'  : 'Ball_and_Plate_MicroLabBox_student/Ethernet communication/MATLAB Function'
 * '<S11>'  : 'Ball_and_Plate_MicroLabBox_student/Innerloop_Actuator/AngleToPos '
 * '<S12>'  : 'Ball_and_Plate_MicroLabBox_student/Innerloop_Actuator/Motor_A'
 * '<S13>'  : 'Ball_and_Plate_MicroLabBox_student/Innerloop_Actuator/Motor_B'
 * '<S14>'  : 'Ball_and_Plate_MicroLabBox_student/Innerloop_Actuator/Motor_C'
 * '<S15>'  : 'Ball_and_Plate_MicroLabBox_student/Innerloop_Actuator/PosToAngle '
 * '<S16>'  : 'Ball_and_Plate_MicroLabBox_student/Innerloop_Actuator/Motor_A/Init_motion'
 * '<S17>'  : 'Ball_and_Plate_MicroLabBox_student/Innerloop_Actuator/Motor_A/MATLAB Function1'
 * '<S18>'  : 'Ball_and_Plate_MicroLabBox_student/Innerloop_Actuator/Motor_A/Multisine'
 * '<S19>'  : 'Ball_and_Plate_MicroLabBox_student/Innerloop_Actuator/Motor_A/Outputs to Amplifier'
 * '<S20>'  : 'Ball_and_Plate_MicroLabBox_student/Innerloop_Actuator/Motor_A/Position Measurement'
 * '<S21>'  : 'Ball_and_Plate_MicroLabBox_student/Innerloop_Actuator/Motor_A/motorA '
 * '<S22>'  : 'Ball_and_Plate_MicroLabBox_student/Innerloop_Actuator/Motor_A/Init_motion/MATLAB Function'
 * '<S23>'  : 'Ball_and_Plate_MicroLabBox_student/Innerloop_Actuator/Motor_A/Init_motion/MATLAB Function2'
 * '<S24>'  : 'Ball_and_Plate_MicroLabBox_student/Innerloop_Actuator/Motor_A/Multisine/MATLAB Function'
 * '<S25>'  : 'Ball_and_Plate_MicroLabBox_student/Innerloop_Actuator/Motor_A/Outputs to Amplifier/DAC_CLASS1_BL1'
 * '<S26>'  : 'Ball_and_Plate_MicroLabBox_student/Innerloop_Actuator/Motor_A/Position Measurement/EMC_ENCODER_BL1'
 * '<S27>'  : 'Ball_and_Plate_MicroLabBox_student/Innerloop_Actuator/Motor_A/Position Measurement/EMC_ENCODER_POS_SET_BL1'
 * '<S28>'  : 'Ball_and_Plate_MicroLabBox_student/Innerloop_Actuator/Motor_A/Position Measurement/Initial'
 * '<S29>'  : 'Ball_and_Plate_MicroLabBox_student/Innerloop_Actuator/Motor_A/Position Measurement/Initial/Enabled Subsystem'
 * '<S30>'  : 'Ball_and_Plate_MicroLabBox_student/Innerloop_Actuator/Motor_A/Position Measurement/Initial/Enabled Subsystem1'
 * '<S31>'  : 'Ball_and_Plate_MicroLabBox_student/Innerloop_Actuator/Motor_B/Init_motion'
 * '<S32>'  : 'Ball_and_Plate_MicroLabBox_student/Innerloop_Actuator/Motor_B/MATLAB Function2'
 * '<S33>'  : 'Ball_and_Plate_MicroLabBox_student/Innerloop_Actuator/Motor_B/Multisine'
 * '<S34>'  : 'Ball_and_Plate_MicroLabBox_student/Innerloop_Actuator/Motor_B/Outputs to Amplifier'
 * '<S35>'  : 'Ball_and_Plate_MicroLabBox_student/Innerloop_Actuator/Motor_B/Position Measurement'
 * '<S36>'  : 'Ball_and_Plate_MicroLabBox_student/Innerloop_Actuator/Motor_B/motorB'
 * '<S37>'  : 'Ball_and_Plate_MicroLabBox_student/Innerloop_Actuator/Motor_B/Init_motion/MATLAB Function'
 * '<S38>'  : 'Ball_and_Plate_MicroLabBox_student/Innerloop_Actuator/Motor_B/Init_motion/MATLAB Function2'
 * '<S39>'  : 'Ball_and_Plate_MicroLabBox_student/Innerloop_Actuator/Motor_B/Multisine/MATLAB Function'
 * '<S40>'  : 'Ball_and_Plate_MicroLabBox_student/Innerloop_Actuator/Motor_B/Outputs to Amplifier/DAC_CLASS1_BL1'
 * '<S41>'  : 'Ball_and_Plate_MicroLabBox_student/Innerloop_Actuator/Motor_B/Position Measurement/EMC_ENCODER_BL1'
 * '<S42>'  : 'Ball_and_Plate_MicroLabBox_student/Innerloop_Actuator/Motor_B/Position Measurement/EMC_ENCODER_POS_SET_BL1'
 * '<S43>'  : 'Ball_and_Plate_MicroLabBox_student/Innerloop_Actuator/Motor_B/Position Measurement/Initial'
 * '<S44>'  : 'Ball_and_Plate_MicroLabBox_student/Innerloop_Actuator/Motor_B/Position Measurement/Initial/Enabled Subsystem'
 * '<S45>'  : 'Ball_and_Plate_MicroLabBox_student/Innerloop_Actuator/Motor_B/Position Measurement/Initial/Enabled Subsystem1'
 * '<S46>'  : 'Ball_and_Plate_MicroLabBox_student/Innerloop_Actuator/Motor_C/Init_motion'
 * '<S47>'  : 'Ball_and_Plate_MicroLabBox_student/Innerloop_Actuator/Motor_C/MATLAB Function1'
 * '<S48>'  : 'Ball_and_Plate_MicroLabBox_student/Innerloop_Actuator/Motor_C/Multisine'
 * '<S49>'  : 'Ball_and_Plate_MicroLabBox_student/Innerloop_Actuator/Motor_C/Multisine1'
 * '<S50>'  : 'Ball_and_Plate_MicroLabBox_student/Innerloop_Actuator/Motor_C/Outputs to Amplifier'
 * '<S51>'  : 'Ball_and_Plate_MicroLabBox_student/Innerloop_Actuator/Motor_C/Position Measurement'
 * '<S52>'  : 'Ball_and_Plate_MicroLabBox_student/Innerloop_Actuator/Motor_C/motorC'
 * '<S53>'  : 'Ball_and_Plate_MicroLabBox_student/Innerloop_Actuator/Motor_C/Init_motion/MATLAB Function'
 * '<S54>'  : 'Ball_and_Plate_MicroLabBox_student/Innerloop_Actuator/Motor_C/Init_motion/MATLAB Function2'
 * '<S55>'  : 'Ball_and_Plate_MicroLabBox_student/Innerloop_Actuator/Motor_C/Multisine/MATLAB Function'
 * '<S56>'  : 'Ball_and_Plate_MicroLabBox_student/Innerloop_Actuator/Motor_C/Multisine1/MATLAB Function'
 * '<S57>'  : 'Ball_and_Plate_MicroLabBox_student/Innerloop_Actuator/Motor_C/Outputs to Amplifier/DAC_CLASS1_BL1'
 * '<S58>'  : 'Ball_and_Plate_MicroLabBox_student/Innerloop_Actuator/Motor_C/Position Measurement/EMC_ENCODER_BL1'
 * '<S59>'  : 'Ball_and_Plate_MicroLabBox_student/Innerloop_Actuator/Motor_C/Position Measurement/EMC_ENCODER_POS_SET_BL1'
 * '<S60>'  : 'Ball_and_Plate_MicroLabBox_student/Innerloop_Actuator/Motor_C/Position Measurement/Initial'
 * '<S61>'  : 'Ball_and_Plate_MicroLabBox_student/Innerloop_Actuator/Motor_C/Position Measurement/Initial/Enabled Subsystem'
 * '<S62>'  : 'Ball_and_Plate_MicroLabBox_student/Innerloop_Actuator/Motor_C/Position Measurement/Initial/Enabled Subsystem1'
 * '<S63>'  : 'Ball_and_Plate_MicroLabBox_student/RTI Data/RTI Data Store'
 * '<S64>'  : 'Ball_and_Plate_MicroLabBox_student/RTI Data/RTI Data Store/RTI Data Store'
 * '<S65>'  : 'Ball_and_Plate_MicroLabBox_student/RTI Data/RTI Data Store/RTI Data Store/RTI Data Store'
 */
#endif                    /* RTW_HEADER_Ball_and_Plate_MicroLabBox_student_h_ */
