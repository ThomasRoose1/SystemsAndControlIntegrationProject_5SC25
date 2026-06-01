/*
 * Ball_and_Plate_MicroLabBox_student.h
 *
 * Academic License - for use in teaching, academic research, and meeting
 * course requirements at degree granting institutions only.  Not for
 * government, commercial, or other organizational use.
 *
 * Code generation for model "Ball_and_Plate_MicroLabBox_student".
 *
 * Model version              : 1.54
 * Simulink Coder version : 9.2 (R2019b) 18-Jul-2019
 * C source code generated on : Wed May 27 15:36:08 2026
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

/* Block signals for system '<S14>/MATLAB Function' */
typedef struct {
  real_T y;                            /* '<S14>/MATLAB Function' */
} B_MATLABFunction_Ball_and_Pla_T;

/* Block states (default storage) for system '<S14>/MATLAB Function' */
typedef struct {
  real_T index;                        /* '<S14>/MATLAB Function' */
  real_T previous_enable;              /* '<S14>/MATLAB Function' */
} DW_MATLABFunction_Ball_and_Pl_T;

/* Block signals for system '<S14>/MATLAB Function2' */
typedef struct {
  real_T path;                         /* '<S14>/MATLAB Function2' */
} B_MATLABFunction2_Ball_and_Pl_T;

/* Block signals for system '<S10>/MATLAB Function1' */
typedef struct {
  real_T path;                         /* '<S10>/MATLAB Function1' */
} B_MATLABFunction1_Ball_and_Pl_T;

/* Block states (default storage) for system '<S10>/MATLAB Function1' */
typedef struct {
  real_T coeffs[6];                    /* '<S10>/MATLAB Function1' */
  real_T t_elapsed;                    /* '<S10>/MATLAB Function1' */
  real_T prev_ref_end;                 /* '<S10>/MATLAB Function1' */
  real_T tf_internal;                  /* '<S10>/MATLAB Function1' */
  boolean_T prev_ref_end_not_empty;    /* '<S10>/MATLAB Function1' */
} DW_MATLABFunction1_Ball_and_P_T;

/* Block signals for system '<S16>/MATLAB Function' */
typedef struct {
  real_T y;                            /* '<S16>/MATLAB Function' */
} B_MATLABFunction_Ball_and_P_m_T;

/* Block states (default storage) for system '<S16>/MATLAB Function' */
typedef struct {
  real_T index;                        /* '<S16>/MATLAB Function' */
  real_T previous_enable;              /* '<S16>/MATLAB Function' */
} DW_MATLABFunction_Ball_and__j_T;

/* Block signals for system '<S26>/Enabled Subsystem' */
typedef struct {
  real_T OutportBufferForOut1;         /* '<S27>/Constant' */
} B_EnabledSubsystem_Ball_and_P_T;

/* Block states (default storage) for system '<S26>/Enabled Subsystem' */
typedef struct {
  boolean_T EnabledSubsystem_MODE;     /* '<S26>/Enabled Subsystem' */
} DW_EnabledSubsystem_Ball_and__T;

/* Block signals for system '<S26>/Enabled Subsystem1' */
typedef struct {
  real_T In1;                          /* '<S28>/In1' */
} B_EnabledSubsystem1_Ball_and__T;

/* Block states (default storage) for system '<S26>/Enabled Subsystem1' */
typedef struct {
  boolean_T EnabledSubsystem1_MODE;    /* '<S26>/Enabled Subsystem1' */
} DW_EnabledSubsystem1_Ball_and_T;

/* Block signals (default storage) */
typedef struct {
  real_T RateTransition3;              /* '<Root>/Rate Transition3' */
  real_T Uk1;                          /* '<S1>/UD' */
  real_T Diff;                         /* '<S1>/Diff' */
  real_T Gain;                         /* '<Root>/Gain' */
  real_T Observer[4];                  /* '<Root>/Observer ' */
  real_T Alpha_sine;                   /* '<Root>/Alpha_sine ' */
  real_T Beta_sine;                    /* '<Root>/Beta_sine' */
  real_T SFunction1[2];                /* '<S7>/S-Function1' */
  real_T Add1;                         /* '<S3>/Add1' */
  real_T Add2;                         /* '<S3>/Add2' */
  real_T Add3;                         /* '<S3>/Add3' */
  real_T Divide1;                      /* '<S3>/Divide1' */
  real_T Divide2;                      /* '<S3>/Divide2' */
  real_T Divide3;                      /* '<S3>/Divide3' */
  real_T Switch;                       /* '<S10>/Switch' */
  real_T enable_ref;                   /* '<S10>/enable_ref ' */
  real_T Sum1;                         /* '<S10>/Sum1' */
  real_T Gain1;                        /* '<S19>/Gain1' */
  real_T Dctleadlag2;                  /* '<S19>/Dctleadlag2' */
  real_T Dct1lowpass3;                 /* '<S19>/Dct1lowpass3' */
  real_T Switch2;                      /* '<S10>/Switch2' */
  real_T Sum;                          /* '<S10>/Sum' */
  real_T Switch3;                      /* '<S10>/Switch3' */
  real_T Switch_f;                     /* '<S14>/Switch' */
  real_T Constant1;                    /* '<S14>/Constant1' */
  real_T Constant2;                    /* '<S14>/Constant2' */
  real_T Switch1;                      /* '<S11>/Switch1' */
  real_T Switch_j;                     /* '<S11>/Switch' */
  real_T Sum1_f;                       /* '<S11>/Sum1' */
  real_T Gain1_f;                      /* '<S34>/Gain1' */
  real_T Dctleadlag2_k;                /* '<S34>/Dctleadlag2' */
  real_T Dct1lowpass3_c;               /* '<S34>/Dct1lowpass3' */
  real_T Switch2_p;                    /* '<S11>/Switch2' */
  real_T Sum_g;                        /* '<S11>/Sum' */
  real_T Switch3_d;                    /* '<S11>/Switch3' */
  real_T Switch_n;                     /* '<S29>/Switch' */
  real_T Constant1_c;                  /* '<S29>/Constant1' */
  real_T Constant2_b;                  /* '<S29>/Constant2' */
  real_T Switch1_k;                    /* '<S12>/Switch1' */
  real_T Switch_jo;                    /* '<S12>/Switch' */
  real_T Sum1_h;                       /* '<S12>/Sum1' */
  real_T Gain1_m;                      /* '<S50>/Gain1' */
  real_T Dctleadlag2_a;                /* '<S50>/Dctleadlag2' */
  real_T Dct1lowpass3_e;               /* '<S50>/Dct1lowpass3' */
  real_T Switch2_h;                    /* '<S12>/Switch2' */
  real_T Sum_h;                        /* '<S12>/Sum' */
  real_T Switch3_l;                    /* '<S12>/Switch3' */
  real_T Switch_e;                     /* '<S44>/Switch' */
  real_T Constant1_h;                  /* '<S44>/Constant1' */
  real_T Constant2_h;                  /* '<S44>/Constant2' */
  real_T reser_integrator;             /* '<S3>/reser_integrator' */
  real_T TmpSignalConversionAtObserverIn[4];
  real_T Beta_ref;                     /* '<Root>/Beta_ref ' */
  real_T alpha;                        /* '<S3>/PosToAngle ' */
  real_T beta;                         /* '<S3>/PosToAngle ' */
  real_T psi;                          /* '<S3>/PosToAngle ' */
  real_T SFunction1_o1;                /* '<S56>/S-Function1' */
  real_T SFunction1_o2;                /* '<S56>/S-Function1' */
  real_T Inc2Pos;                      /* '<S49>/Inc2Pos' */
  real_T AbsPosition;                  /* '<S49>/AbsPosition' */
  real_T Abs;                          /* '<S58>/Abs' */
  real_T TransferFcn;                  /* '<S58>/Transfer Fcn' */
  real_T mm2m;                         /* '<S49>/mm2m' */
  real_T Saturation;                   /* '<S48>/Saturation' */
  real_T Current2V;                    /* '<S48>/Current2V' */
  real_T DSPscale;                     /* '<S48>/DSPscale' */
  real_T SFunction1_o1_i;              /* '<S39>/S-Function1' */
  real_T SFunction1_o2_p;              /* '<S39>/S-Function1' */
  real_T Inc2Pos_e;                    /* '<S33>/Inc2Pos' */
  real_T AbsPosition_i;                /* '<S33>/AbsPosition' */
  real_T Abs_j;                        /* '<S41>/Abs' */
  real_T TransferFcn_o;                /* '<S41>/Transfer Fcn' */
  real_T mm2m_p;                       /* '<S33>/mm2m' */
  real_T Saturation_l;                 /* '<S32>/Saturation' */
  real_T Current2V_f;                  /* '<S32>/Current2V' */
  real_T DSPscale_o;                   /* '<S32>/DSPscale' */
  real_T SFunction1_o1_p;              /* '<S24>/S-Function1' */
  real_T SFunction1_o2_d;              /* '<S24>/S-Function1' */
  real_T Inc2Pos_p;                    /* '<S18>/Inc2Pos' */
  real_T AbsPosition_m;                /* '<S18>/AbsPosition' */
  real_T Abs_h;                        /* '<S26>/Abs' */
  real_T TransferFcn_f;                /* '<S26>/Transfer Fcn' */
  real_T mm2m_po;                      /* '<S18>/mm2m' */
  real_T Saturation_a;                 /* '<S17>/Saturation' */
  real_T Current2V_h;                  /* '<S17>/Current2V' */
  real_T DSPscale_m;                   /* '<S17>/DSPscale' */
  real_T pos1;                         /* '<S3>/AngleToPos ' */
  real_T pos2;                         /* '<S3>/AngleToPos ' */
  real_T pos3;                         /* '<S3>/AngleToPos ' */
  real_T x;                            /* '<S2>/MATLAB Function' */
  real_T y;                            /* '<S2>/MATLAB Function' */
  real_T z;                            /* '<S2>/MATLAB Function' */
  uint32_T SFunction1_o4;              /* '<S6>/S-Function1' */
  uint32_T SFunction1_o1_m[3];         /* '<S5>/S-Function1' */
  uint16_T SFunction1_o6;              /* '<S6>/S-Function1' */
  uint8_T SFunction1_o1_c[12];         /* '<S6>/S-Function1' */
  uint8_T SFunction1_o5[4];            /* '<S6>/S-Function1' */
  uint8_T SFunction1_o2_f[4];          /* '<S5>/S-Function1' */
  boolean_T DataTypeConversion;        /* '<S2>/Data Type Conversion' */
  boolean_T RelationalOperator1;       /* '<S58>/Relational Operator1' */
  boolean_T RelationalOperator;        /* '<S58>/Relational Operator' */
  boolean_T LogicalOperator2;          /* '<S58>/Logical Operator2' */
  boolean_T RelationalOperator1_c;     /* '<S41>/Relational Operator1' */
  boolean_T RelationalOperator_n;      /* '<S41>/Relational Operator' */
  boolean_T LogicalOperator2_l;        /* '<S41>/Logical Operator2' */
  boolean_T RelationalOperator1_f;     /* '<S26>/Relational Operator1' */
  boolean_T RelationalOperator_a;      /* '<S26>/Relational Operator' */
  boolean_T LogicalOperator2_a;        /* '<S26>/Logical Operator2' */
  B_EnabledSubsystem1_Ball_and__T EnabledSubsystem1_b;/* '<S58>/Enabled Subsystem1' */
  B_EnabledSubsystem_Ball_and_P_T EnabledSubsystem_n;/* '<S58>/Enabled Subsystem' */
  B_MATLABFunction_Ball_and_P_m_T sf_MATLABFunction_hg;/* '<S47>/MATLAB Function' */
  B_MATLABFunction_Ball_and_P_m_T sf_MATLABFunction_g;/* '<S46>/MATLAB Function' */
  B_MATLABFunction1_Ball_and_Pl_T sf_MATLABFunction1_h;/* '<S12>/MATLAB Function1' */
  B_MATLABFunction2_Ball_and_Pl_T sf_MATLABFunction2_e;/* '<S44>/MATLAB Function2' */
  B_MATLABFunction_Ball_and_Pla_T sf_MATLABFunction_h;/* '<S44>/MATLAB Function' */
  B_EnabledSubsystem1_Ball_and__T EnabledSubsystem1_f;/* '<S41>/Enabled Subsystem1' */
  B_EnabledSubsystem_Ball_and_P_T EnabledSubsystem_a;/* '<S41>/Enabled Subsystem' */
  B_MATLABFunction_Ball_and_P_m_T sf_MATLABFunction_ko;/* '<S31>/MATLAB Function' */
  B_MATLABFunction1_Ball_and_Pl_T sf_MATLABFunction2_d;/* '<S11>/MATLAB Function2' */
  B_MATLABFunction2_Ball_and_Pl_T sf_MATLABFunction2_m;/* '<S29>/MATLAB Function2' */
  B_MATLABFunction_Ball_and_Pla_T sf_MATLABFunction_k;/* '<S29>/MATLAB Function' */
  B_EnabledSubsystem1_Ball_and__T EnabledSubsystem1;/* '<S26>/Enabled Subsystem1' */
  B_EnabledSubsystem_Ball_and_P_T EnabledSubsystem;/* '<S26>/Enabled Subsystem' */
  B_MATLABFunction_Ball_and_P_m_T sf_MATLABFunction_e;/* '<S16>/MATLAB Function' */
  B_MATLABFunction1_Ball_and_Pl_T sf_MATLABFunction1;/* '<S10>/MATLAB Function1' */
  B_MATLABFunction2_Ball_and_Pl_T sf_MATLABFunction2;/* '<S14>/MATLAB Function2' */
  B_MATLABFunction_Ball_and_Pla_T sf_MATLABFunction_i;/* '<S14>/MATLAB Function' */
} B_Ball_and_Plate_MicroLabBox_student_T;

/* Block states (default storage) for system '<Root>' */
typedef struct {
  real_T UD_DSTATE;                    /* '<S1>/UD' */
  real_T Observer_DSTATE[4];           /* '<Root>/Observer ' */
  volatile real_T RateTransition3_Buffer0;/* '<Root>/Rate Transition3' */
  struct {
    real_T RECEIVED_FRAMES;
  } SFunction1_RWORK;                  /* '<S6>/S-Function1' */

  struct {
    real_T RX_DROPPED_FRAMES[2];
  } SFunction1_RWORK_l;                /* '<S7>/S-Function1' */

  real_T Dctleadlag2_RWORK[2];         /* '<S19>/Dctleadlag2' */
  real_T Dct1lowpass3_RWORK[2];        /* '<S19>/Dct1lowpass3' */
  real_T Dctleadlag2_RWORK_o[2];       /* '<S34>/Dctleadlag2' */
  real_T Dct1lowpass3_RWORK_b[2];      /* '<S34>/Dct1lowpass3' */
  real_T Dctleadlag2_RWORK_p[2];       /* '<S50>/Dctleadlag2' */
  real_T Dct1lowpass3_RWORK_d[2];      /* '<S50>/Dct1lowpass3' */
  volatile int8_T RateTransition3_semaphoreTaken;/* '<Root>/Rate Transition3' */
  DW_EnabledSubsystem1_Ball_and_T EnabledSubsystem1_b;/* '<S58>/Enabled Subsystem1' */
  DW_EnabledSubsystem_Ball_and__T EnabledSubsystem_n;/* '<S58>/Enabled Subsystem' */
  DW_MATLABFunction_Ball_and__j_T sf_MATLABFunction_hg;/* '<S47>/MATLAB Function' */
  DW_MATLABFunction_Ball_and__j_T sf_MATLABFunction_g;/* '<S46>/MATLAB Function' */
  DW_MATLABFunction1_Ball_and_P_T sf_MATLABFunction1_h;/* '<S12>/MATLAB Function1' */
  DW_MATLABFunction_Ball_and_Pl_T sf_MATLABFunction_h;/* '<S44>/MATLAB Function' */
  DW_EnabledSubsystem1_Ball_and_T EnabledSubsystem1_f;/* '<S41>/Enabled Subsystem1' */
  DW_EnabledSubsystem_Ball_and__T EnabledSubsystem_a;/* '<S41>/Enabled Subsystem' */
  DW_MATLABFunction_Ball_and__j_T sf_MATLABFunction_ko;/* '<S31>/MATLAB Function' */
  DW_MATLABFunction1_Ball_and_P_T sf_MATLABFunction2_d;/* '<S11>/MATLAB Function2' */
  DW_MATLABFunction_Ball_and_Pl_T sf_MATLABFunction_k;/* '<S29>/MATLAB Function' */
  DW_EnabledSubsystem1_Ball_and_T EnabledSubsystem1;/* '<S26>/Enabled Subsystem1' */
  DW_EnabledSubsystem_Ball_and__T EnabledSubsystem;/* '<S26>/Enabled Subsystem' */
  DW_MATLABFunction_Ball_and__j_T sf_MATLABFunction_e;/* '<S16>/MATLAB Function' */
  DW_MATLABFunction1_Ball_and_P_T sf_MATLABFunction1;/* '<S10>/MATLAB Function1' */
  DW_MATLABFunction_Ball_and_Pl_T sf_MATLABFunction_i;/* '<S14>/MATLAB Function' */
} DW_Ball_and_Plate_MicroLabBox_student_T;

/* Continuous states (default storage) */
typedef struct {
  real_T TransferFcn_CSTATE;           /* '<S58>/Transfer Fcn' */
  real_T TransferFcn_CSTATE_h;         /* '<S41>/Transfer Fcn' */
  real_T TransferFcn_CSTATE_l;         /* '<S26>/Transfer Fcn' */
} X_Ball_and_Plate_MicroLabBox_student_T;

/* State derivatives (default storage) */
typedef struct {
  real_T TransferFcn_CSTATE;           /* '<S58>/Transfer Fcn' */
  real_T TransferFcn_CSTATE_h;         /* '<S41>/Transfer Fcn' */
  real_T TransferFcn_CSTATE_l;         /* '<S26>/Transfer Fcn' */
} XDot_Ball_and_Plate_MicroLabBox_student_T;

/* State disabled  */
typedef struct {
  boolean_T TransferFcn_CSTATE;        /* '<S58>/Transfer Fcn' */
  boolean_T TransferFcn_CSTATE_h;      /* '<S41>/Transfer Fcn' */
  boolean_T TransferFcn_CSTATE_l;      /* '<S26>/Transfer Fcn' */
} XDis_Ball_and_Plate_MicroLabBox_student_T;

/* Zero-crossing (trigger) state */
typedef struct {
  ZCSigState EMC_ENCODER_POS_SET_BL1_Trig_ZC;/* '<S49>/EMC_ENCODER_POS_SET_BL1' */
  ZCSigState EMC_ENCODER_POS_SET_BL1_Trig__f;/* '<S33>/EMC_ENCODER_POS_SET_BL1' */
  ZCSigState EMC_ENCODER_POS_SET_BL1_Trig_fh;/* '<S18>/EMC_ENCODER_POS_SET_BL1' */
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

/* Parameters for system: '<S26>/Enabled Subsystem' */
struct P_EnabledSubsystem_Ball_and_P_T_ {
  real_T Out1_Y0;                      /* Expression: [0]
                                        * Referenced by: '<S27>/Out1'
                                        */
  real_T Constant_Value;               /* Expression: -1
                                        * Referenced by: '<S27>/Constant'
                                        */
};

/* Parameters for system: '<S26>/Enabled Subsystem1' */
struct P_EnabledSubsystem1_Ball_and__T_ {
  real_T Out1_Y0;                      /* Expression: [0]
                                        * Referenced by: '<S28>/Out1'
                                        */
};

/* Parameters (default storage) */
struct P_Ball_and_Plate_MicroLabBox_student_T_ {
  real_T A_obs[16];                    /* Variable: A_obs
                                        * Referenced by: '<Root>/Observer '
                                        */
  real_T B_obs[16];                    /* Variable: B_obs
                                        * Referenced by: '<Root>/Observer '
                                        */
  real_T C_obs[16];                    /* Variable: C_obs
                                        * Referenced by: '<Root>/Observer '
                                        */
  real_T D_obs[16];                    /* Variable: D_obs
                                        * Referenced by: '<Root>/Observer '
                                        */
  real_T Ts_Outer;                     /* Variable: Ts_Outer
                                        * Referenced by: '<Root>/Gain'
                                        */
  real_T path[3001];                   /* Variable: path
                                        * Referenced by:
                                        *   '<S14>/Constant4'
                                        *   '<S29>/Constant4'
                                        *   '<S44>/Constant4'
                                        */
  real_T uA[200000];                   /* Variable: uA
                                        * Referenced by:
                                        *   '<S16>/Constant'
                                        *   '<S46>/Constant'
                                        */
  real_T uB[200000];                   /* Variable: uB
                                        * Referenced by: '<S31>/Constant'
                                        */
  real_T uC[200000];                   /* Variable: uC
                                        * Referenced by: '<S47>/Constant'
                                        */
  real_T Difference_ICPrevInput;       /* Mask Parameter: Difference_ICPrevInput
                                        * Referenced by: '<S1>/UD'
                                        */
  real_T Constant_Value;               /* Expression: 0
                                        * Referenced by: '<S10>/Constant'
                                        */
  real_T Constant3_Value;              /* Expression: -0.0289
                                        * Referenced by: '<S10>/Constant3'
                                        */
  real_T Constant3_Value_b;            /* Expression: 0
                                        * Referenced by: '<S14>/Constant3'
                                        */
  real_T Saturation_UpperSat;          /* Expression: 3
                                        * Referenced by: '<S17>/Saturation'
                                        */
  real_T Saturation_LowerSat;          /* Expression: -3
                                        * Referenced by: '<S17>/Saturation'
                                        */
  real_T Current2V_Gain;               /* Expression: 10/3
                                        * Referenced by: '<S17>/Current2V'
                                        */
  real_T DSPscale_Gain;                /* Expression: 1/10
                                        * Referenced by: '<S17>/DSPscale'
                                        */
  real_T Constant_Value_j;             /* Expression: 10
                                        * Referenced by: '<S18>/Constant'
                                        */
  real_T ZP_Value;                     /* Expression: 29
                                        * Referenced by: '<S18>/ZP'
                                        */
  real_T Inc2Pos_Gain;                 /* Expression: 8e-3
                                        * Referenced by: '<S18>/Inc2Pos'
                                        */
  real_T Pos_offset_Value;             /* Expression: 0
                                        * Referenced by: '<S18>/Pos_offset'
                                        */
  real_T Const_Value;                  /* Expression: 0.01
                                        * Referenced by: '<S26>/Const'
                                        */
  real_T Const1_Value;                 /* Expression: -0.98
                                        * Referenced by: '<S26>/Const1'
                                        */
  real_T TransferFcn_A;                /* Computed Parameter: TransferFcn_A
                                        * Referenced by: '<S26>/Transfer Fcn'
                                        */
  real_T TransferFcn_C;                /* Computed Parameter: TransferFcn_C
                                        * Referenced by: '<S26>/Transfer Fcn'
                                        */
  real_T mm2m_Gain;                    /* Expression: 0.001
                                        * Referenced by: '<S18>/mm2m'
                                        */
  real_T Constant1_Value;              /* Expression: 0
                                        * Referenced by: '<S11>/Constant1'
                                        */
  real_T Constant5_Value;              /* Expression: -0.0289
                                        * Referenced by: '<S11>/Constant5'
                                        */
  real_T Constant3_Value_i;            /* Expression: 0
                                        * Referenced by: '<S29>/Constant3'
                                        */
  real_T Saturation_UpperSat_e;        /* Expression: 3
                                        * Referenced by: '<S32>/Saturation'
                                        */
  real_T Saturation_LowerSat_f;        /* Expression: -3
                                        * Referenced by: '<S32>/Saturation'
                                        */
  real_T Current2V_Gain_f;             /* Expression: 10/3
                                        * Referenced by: '<S32>/Current2V'
                                        */
  real_T DSPscale_Gain_d;              /* Expression: 1/10
                                        * Referenced by: '<S32>/DSPscale'
                                        */
  real_T Constant_Value_j0;            /* Expression: 10
                                        * Referenced by: '<S33>/Constant'
                                        */
  real_T ZP_Value_m;                   /* Expression: 29
                                        * Referenced by: '<S33>/ZP'
                                        */
  real_T Inc2Pos_Gain_k;               /* Expression: 8e-3
                                        * Referenced by: '<S33>/Inc2Pos'
                                        */
  real_T Pos_offset_Value_k;           /* Expression: 0
                                        * Referenced by: '<S33>/Pos_offset'
                                        */
  real_T Const_Value_g;                /* Expression: 0.01
                                        * Referenced by: '<S41>/Const'
                                        */
  real_T Const1_Value_a;               /* Expression: -0.98
                                        * Referenced by: '<S41>/Const1'
                                        */
  real_T TransferFcn_A_g;              /* Computed Parameter: TransferFcn_A_g
                                        * Referenced by: '<S41>/Transfer Fcn'
                                        */
  real_T TransferFcn_C_e;              /* Computed Parameter: TransferFcn_C_e
                                        * Referenced by: '<S41>/Transfer Fcn'
                                        */
  real_T mm2m_Gain_f;                  /* Expression: 0.001
                                        * Referenced by: '<S33>/mm2m'
                                        */
  real_T Constant_Value_e;             /* Expression: 0
                                        * Referenced by: '<S12>/Constant'
                                        */
  real_T Constant3_Value_n;            /* Expression: -0.0289
                                        * Referenced by: '<S12>/Constant3'
                                        */
  real_T Constant3_Value_p;            /* Expression: 0
                                        * Referenced by: '<S44>/Constant3'
                                        */
  real_T Saturation_UpperSat_h;        /* Expression: 3
                                        * Referenced by: '<S48>/Saturation'
                                        */
  real_T Saturation_LowerSat_p;        /* Expression: -3
                                        * Referenced by: '<S48>/Saturation'
                                        */
  real_T Current2V_Gain_p;             /* Expression: 10/3
                                        * Referenced by: '<S48>/Current2V'
                                        */
  real_T DSPscale_Gain_h;              /* Expression: 1/10
                                        * Referenced by: '<S48>/DSPscale'
                                        */
  real_T Constant_Value_f;             /* Expression: 10
                                        * Referenced by: '<S49>/Constant'
                                        */
  real_T ZP_Value_p;                   /* Expression: 29
                                        * Referenced by: '<S49>/ZP'
                                        */
  real_T Inc2Pos_Gain_h;               /* Expression: 8e-3
                                        * Referenced by: '<S49>/Inc2Pos'
                                        */
  real_T Pos_offset_Value_f;           /* Expression: 0
                                        * Referenced by: '<S49>/Pos_offset'
                                        */
  real_T Const_Value_m;                /* Expression: 0.01
                                        * Referenced by: '<S58>/Const'
                                        */
  real_T Const1_Value_p;               /* Expression: -0.98
                                        * Referenced by: '<S58>/Const1'
                                        */
  real_T TransferFcn_A_m;              /* Computed Parameter: TransferFcn_A_m
                                        * Referenced by: '<S58>/Transfer Fcn'
                                        */
  real_T TransferFcn_C_c;              /* Computed Parameter: TransferFcn_C_c
                                        * Referenced by: '<S58>/Transfer Fcn'
                                        */
  real_T mm2m_Gain_o;                  /* Expression: 0.001
                                        * Referenced by: '<S49>/mm2m'
                                        */
  real_T Constant_Value_i;             /* Expression: 1
                                        * Referenced by: '<S2>/Constant'
                                        */
  real_T Observer_InitialCondition;    /* Expression: 0
                                        * Referenced by: '<Root>/Observer '
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
  real_T enable_quintic_Value;         /* Expression: 0
                                        * Referenced by: '<S3>/enable_quintic  '
                                        */
  real_T quintic_ref_Value;            /* Expression: 0
                                        * Referenced by: '<S3>/quintic_ref '
                                        */
  real_T Switch_Threshold;             /* Expression: 0.5
                                        * Referenced by: '<S10>/Switch'
                                        */
  real_T Constant2_Value_j;            /* Expression: 1
                                        * Referenced by: '<S10>/Constant2'
                                        */
  real_T Constant1_Value_na;           /* Expression: 0.001
                                        * Referenced by: '<S10>/Constant1'
                                        */
  real_T enable_ref_Value;             /* Expression: 1
                                        * Referenced by: '<S3>/enable_ref '
                                        */
  real_T enable_ref_Threshold;         /* Expression: 0
                                        * Referenced by: '<S10>/enable_ref '
                                        */
  real_T Gain1_Gain;                   /* Expression: 1400
                                        * Referenced by: '<S19>/Gain1'
                                        */
  real_T Dctleadlag2_P1_Size[2];      /* Computed Parameter: Dctleadlag2_P1_Size
                                       * Referenced by: '<S19>/Dctleadlag2'
                                       */
  real_T Dctleadlag2_P1;               /* Expression: f_num
                                        * Referenced by: '<S19>/Dctleadlag2'
                                        */
  real_T Dctleadlag2_P2_Size[2];      /* Computed Parameter: Dctleadlag2_P2_Size
                                       * Referenced by: '<S19>/Dctleadlag2'
                                       */
  real_T Dctleadlag2_P2;               /* Expression: f_den
                                        * Referenced by: '<S19>/Dctleadlag2'
                                        */
  real_T Dctleadlag2_P3_Size[2];      /* Computed Parameter: Dctleadlag2_P3_Size
                                       * Referenced by: '<S19>/Dctleadlag2'
                                       */
  real_T Dctleadlag2_P3;               /* Expression: 0.001
                                        * Referenced by: '<S19>/Dctleadlag2'
                                        */
  real_T Dct1lowpass3_P1_Size[2];    /* Computed Parameter: Dct1lowpass3_P1_Size
                                      * Referenced by: '<S19>/Dct1lowpass3'
                                      */
  real_T Dct1lowpass3_P1;              /* Expression: f_den
                                        * Referenced by: '<S19>/Dct1lowpass3'
                                        */
  real_T Dct1lowpass3_P2_Size[2];    /* Computed Parameter: Dct1lowpass3_P2_Size
                                      * Referenced by: '<S19>/Dct1lowpass3'
                                      */
  real_T Dct1lowpass3_P2;              /* Expression: 0.001
                                        * Referenced by: '<S19>/Dct1lowpass3'
                                        */
  real_T Switch2_Threshold;            /* Expression: 0.5
                                        * Referenced by: '<S10>/Switch2'
                                        */
  real_T enable_ID_A_Value;            /* Expression: 0
                                        * Referenced by: '<S10>/enable_ID_A '
                                        */
  real_T Switch3_Threshold;            /* Expression: 0.5
                                        * Referenced by: '<S10>/Switch3'
                                        */
  real_T u_no_0_init_motion_Value;     /* Expression: 1
                                        * Referenced by: '<S10>/1_no_0_init_motion'
                                        */
  real_T Constant4_Value;              /* Expression: -0.0289
                                        * Referenced by: '<S10>/Constant4'
                                        */
  real_T Switch_Threshold_c;           /* Expression: 0.5
                                        * Referenced by: '<S14>/Switch'
                                        */
  real_T Constant1_Value_i;            /* Expression: 3
                                        * Referenced by: '<S14>/Constant1'
                                        */
  real_T Constant2_Value_e;            /* Expression: 0.001
                                        * Referenced by: '<S14>/Constant2'
                                        */
  real_T Switch1_Threshold;            /* Expression: 0.5
                                        * Referenced by: '<S11>/Switch1'
                                        */
  real_T Constant3_Value_k;            /* Expression: 1
                                        * Referenced by: '<S11>/Constant3'
                                        */
  real_T Constant2_Value_c;            /* Expression: 0.001
                                        * Referenced by: '<S11>/Constant2'
                                        */
  real_T Switch_Threshold_a;           /* Expression: 0
                                        * Referenced by: '<S11>/Switch'
                                        */
  real_T Gain1_Gain_l;                 /* Expression: 1300
                                        * Referenced by: '<S34>/Gain1'
                                        */
  real_T Dctleadlag2_P1_Size_o[2];  /* Computed Parameter: Dctleadlag2_P1_Size_o
                                     * Referenced by: '<S34>/Dctleadlag2'
                                     */
  real_T Dctleadlag2_P1_g;             /* Expression: f_num
                                        * Referenced by: '<S34>/Dctleadlag2'
                                        */
  real_T Dctleadlag2_P2_Size_b[2];  /* Computed Parameter: Dctleadlag2_P2_Size_b
                                     * Referenced by: '<S34>/Dctleadlag2'
                                     */
  real_T Dctleadlag2_P2_b;             /* Expression: f_den
                                        * Referenced by: '<S34>/Dctleadlag2'
                                        */
  real_T Dctleadlag2_P3_Size_f[2];  /* Computed Parameter: Dctleadlag2_P3_Size_f
                                     * Referenced by: '<S34>/Dctleadlag2'
                                     */
  real_T Dctleadlag2_P3_j;             /* Expression: 0.001
                                        * Referenced by: '<S34>/Dctleadlag2'
                                        */
  real_T Dct1lowpass3_P1_Size_m[2];/* Computed Parameter: Dct1lowpass3_P1_Size_m
                                    * Referenced by: '<S34>/Dct1lowpass3'
                                    */
  real_T Dct1lowpass3_P1_a;            /* Expression: f_den
                                        * Referenced by: '<S34>/Dct1lowpass3'
                                        */
  real_T Dct1lowpass3_P2_Size_i[2];/* Computed Parameter: Dct1lowpass3_P2_Size_i
                                    * Referenced by: '<S34>/Dct1lowpass3'
                                    */
  real_T Dct1lowpass3_P2_h;            /* Expression: 0.001
                                        * Referenced by: '<S34>/Dct1lowpass3'
                                        */
  real_T Switch2_Threshold_k;          /* Expression: 0.5
                                        * Referenced by: '<S11>/Switch2'
                                        */
  real_T enable_ID_B_Value;            /* Expression: 0
                                        * Referenced by: '<S11>/enable_ID_B '
                                        */
  real_T Switch3_Threshold_m;          /* Expression: 0.5
                                        * Referenced by: '<S11>/Switch3'
                                        */
  real_T u_no_0_init_motion_Value_e;   /* Expression: 1
                                        * Referenced by: '<S11>/1_no_0_init_motion'
                                        */
  real_T Constant4_Value_b;            /* Expression: -0.0289
                                        * Referenced by: '<S11>/Constant4'
                                        */
  real_T Switch_Threshold_k;           /* Expression: 0.5
                                        * Referenced by: '<S29>/Switch'
                                        */
  real_T Constant1_Value_p;            /* Expression: 3
                                        * Referenced by: '<S29>/Constant1'
                                        */
  real_T Constant2_Value_g;            /* Expression: 0.001
                                        * Referenced by: '<S29>/Constant2'
                                        */
  real_T Switch1_Threshold_j;          /* Expression: 0.5
                                        * Referenced by: '<S12>/Switch1'
                                        */
  real_T Constant2_Value_b;            /* Expression: 1
                                        * Referenced by: '<S12>/Constant2'
                                        */
  real_T Constant1_Value_c;            /* Expression: 0.001
                                        * Referenced by: '<S12>/Constant1'
                                        */
  real_T Switch_Threshold_e;           /* Expression: 0
                                        * Referenced by: '<S12>/Switch'
                                        */
  real_T Gain1_Gain_p;                 /* Expression: 450
                                        * Referenced by: '<S50>/Gain1'
                                        */
  real_T Dctleadlag2_P1_Size_f[2];  /* Computed Parameter: Dctleadlag2_P1_Size_f
                                     * Referenced by: '<S50>/Dctleadlag2'
                                     */
  real_T Dctleadlag2_P1_h;             /* Expression: f_num
                                        * Referenced by: '<S50>/Dctleadlag2'
                                        */
  real_T Dctleadlag2_P2_Size_d[2];  /* Computed Parameter: Dctleadlag2_P2_Size_d
                                     * Referenced by: '<S50>/Dctleadlag2'
                                     */
  real_T Dctleadlag2_P2_l;             /* Expression: f_den
                                        * Referenced by: '<S50>/Dctleadlag2'
                                        */
  real_T Dctleadlag2_P3_Size_p[2];  /* Computed Parameter: Dctleadlag2_P3_Size_p
                                     * Referenced by: '<S50>/Dctleadlag2'
                                     */
  real_T Dctleadlag2_P3_l;             /* Expression: 0.001
                                        * Referenced by: '<S50>/Dctleadlag2'
                                        */
  real_T Dct1lowpass3_P1_Size_d[2];/* Computed Parameter: Dct1lowpass3_P1_Size_d
                                    * Referenced by: '<S50>/Dct1lowpass3'
                                    */
  real_T Dct1lowpass3_P1_g;            /* Expression: f_den
                                        * Referenced by: '<S50>/Dct1lowpass3'
                                        */
  real_T Dct1lowpass3_P2_Size_k[2];/* Computed Parameter: Dct1lowpass3_P2_Size_k
                                    * Referenced by: '<S50>/Dct1lowpass3'
                                    */
  real_T Dct1lowpass3_P2_f;            /* Expression: 0.001
                                        * Referenced by: '<S50>/Dct1lowpass3'
                                        */
  real_T Switch2_Threshold_b;          /* Expression: 0.5
                                        * Referenced by: '<S12>/Switch2'
                                        */
  real_T Enable_ID_C_Value;            /* Expression: 0
                                        * Referenced by: '<S12>/Enable_ID_C '
                                        */
  real_T Switch3_Threshold_g;          /* Expression: 0.5
                                        * Referenced by: '<S12>/Switch3'
                                        */
  real_T u_no_0_init_motion_Value_n;   /* Expression: 1
                                        * Referenced by: '<S12>/1_no_0_init_motion'
                                        */
  real_T Constant4_Value_c;            /* Expression: -0.0289
                                        * Referenced by: '<S12>/Constant4'
                                        */
  real_T Switch_Threshold_l;           /* Expression: 0.5
                                        * Referenced by: '<S44>/Switch'
                                        */
  real_T Constant1_Value_f;            /* Expression: 3
                                        * Referenced by: '<S44>/Constant1'
                                        */
  real_T Constant2_Value_i;            /* Expression: 0.001
                                        * Referenced by: '<S44>/Constant2'
                                        */
  real_T u_to_enable_id_Value;         /* Expression: 0
                                        * Referenced by: '<S12>/1_to_enable_id'
                                        */
  real_T reser_integrator_Value;       /* Expression: 0
                                        * Referenced by: '<S3>/reser_integrator'
                                        */
  real_T Beta_ref_Value;               /* Expression: 0
                                        * Referenced by: '<Root>/Beta_ref '
                                        */
  P_EnabledSubsystem1_Ball_and__T EnabledSubsystem1_b;/* '<S58>/Enabled Subsystem1' */
  P_EnabledSubsystem_Ball_and_P_T EnabledSubsystem_n;/* '<S58>/Enabled Subsystem' */
  P_EnabledSubsystem1_Ball_and__T EnabledSubsystem1_f;/* '<S41>/Enabled Subsystem1' */
  P_EnabledSubsystem_Ball_and_P_T EnabledSubsystem_a;/* '<S41>/Enabled Subsystem' */
  P_EnabledSubsystem1_Ball_and__T EnabledSubsystem1;/* '<S26>/Enabled Subsystem1' */
  P_EnabledSubsystem_Ball_and_P_T EnabledSubsystem;/* '<S26>/Enabled Subsystem' */
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
    SimStruct childSFunctions[6];
    SimStruct *childSFunctionPtrs[6];
    struct _ssBlkInfo2 blkInfo2[6];
    struct _ssSFcnModelMethods2 methods2[6];
    struct _ssSFcnModelMethods3 methods3[6];
    struct _ssSFcnModelMethods4 methods4[6];
    struct _ssStatesInfo2 statesInfo2[6];
    ssPeriodicStatesInfo periodicStatesInfo[6];
    struct _ssPortInfo2 inputOutputPortInfo2[6];
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
 * '<S5>'   : 'Ball_and_Plate_MicroLabBox_student/Ethernet communication/ETHERNET_SETUP_BL1'
 * '<S6>'   : 'Ball_and_Plate_MicroLabBox_student/Ethernet communication/ETHERNET_UDP_RX_BL1'
 * '<S7>'   : 'Ball_and_Plate_MicroLabBox_student/Ethernet communication/ETHERNET_UDP_SETUP_BL1'
 * '<S8>'   : 'Ball_and_Plate_MicroLabBox_student/Ethernet communication/MATLAB Function'
 * '<S9>'   : 'Ball_and_Plate_MicroLabBox_student/Innerloop_Actuator/AngleToPos '
 * '<S10>'  : 'Ball_and_Plate_MicroLabBox_student/Innerloop_Actuator/Motor_A'
 * '<S11>'  : 'Ball_and_Plate_MicroLabBox_student/Innerloop_Actuator/Motor_B'
 * '<S12>'  : 'Ball_and_Plate_MicroLabBox_student/Innerloop_Actuator/Motor_C'
 * '<S13>'  : 'Ball_and_Plate_MicroLabBox_student/Innerloop_Actuator/PosToAngle '
 * '<S14>'  : 'Ball_and_Plate_MicroLabBox_student/Innerloop_Actuator/Motor_A/Init_motion'
 * '<S15>'  : 'Ball_and_Plate_MicroLabBox_student/Innerloop_Actuator/Motor_A/MATLAB Function1'
 * '<S16>'  : 'Ball_and_Plate_MicroLabBox_student/Innerloop_Actuator/Motor_A/Multisine'
 * '<S17>'  : 'Ball_and_Plate_MicroLabBox_student/Innerloop_Actuator/Motor_A/Outputs to Amplifier'
 * '<S18>'  : 'Ball_and_Plate_MicroLabBox_student/Innerloop_Actuator/Motor_A/Position Measurement'
 * '<S19>'  : 'Ball_and_Plate_MicroLabBox_student/Innerloop_Actuator/Motor_A/motorA '
 * '<S20>'  : 'Ball_and_Plate_MicroLabBox_student/Innerloop_Actuator/Motor_A/Init_motion/MATLAB Function'
 * '<S21>'  : 'Ball_and_Plate_MicroLabBox_student/Innerloop_Actuator/Motor_A/Init_motion/MATLAB Function2'
 * '<S22>'  : 'Ball_and_Plate_MicroLabBox_student/Innerloop_Actuator/Motor_A/Multisine/MATLAB Function'
 * '<S23>'  : 'Ball_and_Plate_MicroLabBox_student/Innerloop_Actuator/Motor_A/Outputs to Amplifier/DAC_CLASS1_BL1'
 * '<S24>'  : 'Ball_and_Plate_MicroLabBox_student/Innerloop_Actuator/Motor_A/Position Measurement/EMC_ENCODER_BL1'
 * '<S25>'  : 'Ball_and_Plate_MicroLabBox_student/Innerloop_Actuator/Motor_A/Position Measurement/EMC_ENCODER_POS_SET_BL1'
 * '<S26>'  : 'Ball_and_Plate_MicroLabBox_student/Innerloop_Actuator/Motor_A/Position Measurement/Initial'
 * '<S27>'  : 'Ball_and_Plate_MicroLabBox_student/Innerloop_Actuator/Motor_A/Position Measurement/Initial/Enabled Subsystem'
 * '<S28>'  : 'Ball_and_Plate_MicroLabBox_student/Innerloop_Actuator/Motor_A/Position Measurement/Initial/Enabled Subsystem1'
 * '<S29>'  : 'Ball_and_Plate_MicroLabBox_student/Innerloop_Actuator/Motor_B/Init_motion'
 * '<S30>'  : 'Ball_and_Plate_MicroLabBox_student/Innerloop_Actuator/Motor_B/MATLAB Function2'
 * '<S31>'  : 'Ball_and_Plate_MicroLabBox_student/Innerloop_Actuator/Motor_B/Multisine'
 * '<S32>'  : 'Ball_and_Plate_MicroLabBox_student/Innerloop_Actuator/Motor_B/Outputs to Amplifier'
 * '<S33>'  : 'Ball_and_Plate_MicroLabBox_student/Innerloop_Actuator/Motor_B/Position Measurement'
 * '<S34>'  : 'Ball_and_Plate_MicroLabBox_student/Innerloop_Actuator/Motor_B/motorB'
 * '<S35>'  : 'Ball_and_Plate_MicroLabBox_student/Innerloop_Actuator/Motor_B/Init_motion/MATLAB Function'
 * '<S36>'  : 'Ball_and_Plate_MicroLabBox_student/Innerloop_Actuator/Motor_B/Init_motion/MATLAB Function2'
 * '<S37>'  : 'Ball_and_Plate_MicroLabBox_student/Innerloop_Actuator/Motor_B/Multisine/MATLAB Function'
 * '<S38>'  : 'Ball_and_Plate_MicroLabBox_student/Innerloop_Actuator/Motor_B/Outputs to Amplifier/DAC_CLASS1_BL1'
 * '<S39>'  : 'Ball_and_Plate_MicroLabBox_student/Innerloop_Actuator/Motor_B/Position Measurement/EMC_ENCODER_BL1'
 * '<S40>'  : 'Ball_and_Plate_MicroLabBox_student/Innerloop_Actuator/Motor_B/Position Measurement/EMC_ENCODER_POS_SET_BL1'
 * '<S41>'  : 'Ball_and_Plate_MicroLabBox_student/Innerloop_Actuator/Motor_B/Position Measurement/Initial'
 * '<S42>'  : 'Ball_and_Plate_MicroLabBox_student/Innerloop_Actuator/Motor_B/Position Measurement/Initial/Enabled Subsystem'
 * '<S43>'  : 'Ball_and_Plate_MicroLabBox_student/Innerloop_Actuator/Motor_B/Position Measurement/Initial/Enabled Subsystem1'
 * '<S44>'  : 'Ball_and_Plate_MicroLabBox_student/Innerloop_Actuator/Motor_C/Init_motion'
 * '<S45>'  : 'Ball_and_Plate_MicroLabBox_student/Innerloop_Actuator/Motor_C/MATLAB Function1'
 * '<S46>'  : 'Ball_and_Plate_MicroLabBox_student/Innerloop_Actuator/Motor_C/Multisine'
 * '<S47>'  : 'Ball_and_Plate_MicroLabBox_student/Innerloop_Actuator/Motor_C/Multisine1'
 * '<S48>'  : 'Ball_and_Plate_MicroLabBox_student/Innerloop_Actuator/Motor_C/Outputs to Amplifier'
 * '<S49>'  : 'Ball_and_Plate_MicroLabBox_student/Innerloop_Actuator/Motor_C/Position Measurement'
 * '<S50>'  : 'Ball_and_Plate_MicroLabBox_student/Innerloop_Actuator/Motor_C/motorC'
 * '<S51>'  : 'Ball_and_Plate_MicroLabBox_student/Innerloop_Actuator/Motor_C/Init_motion/MATLAB Function'
 * '<S52>'  : 'Ball_and_Plate_MicroLabBox_student/Innerloop_Actuator/Motor_C/Init_motion/MATLAB Function2'
 * '<S53>'  : 'Ball_and_Plate_MicroLabBox_student/Innerloop_Actuator/Motor_C/Multisine/MATLAB Function'
 * '<S54>'  : 'Ball_and_Plate_MicroLabBox_student/Innerloop_Actuator/Motor_C/Multisine1/MATLAB Function'
 * '<S55>'  : 'Ball_and_Plate_MicroLabBox_student/Innerloop_Actuator/Motor_C/Outputs to Amplifier/DAC_CLASS1_BL1'
 * '<S56>'  : 'Ball_and_Plate_MicroLabBox_student/Innerloop_Actuator/Motor_C/Position Measurement/EMC_ENCODER_BL1'
 * '<S57>'  : 'Ball_and_Plate_MicroLabBox_student/Innerloop_Actuator/Motor_C/Position Measurement/EMC_ENCODER_POS_SET_BL1'
 * '<S58>'  : 'Ball_and_Plate_MicroLabBox_student/Innerloop_Actuator/Motor_C/Position Measurement/Initial'
 * '<S59>'  : 'Ball_and_Plate_MicroLabBox_student/Innerloop_Actuator/Motor_C/Position Measurement/Initial/Enabled Subsystem'
 * '<S60>'  : 'Ball_and_Plate_MicroLabBox_student/Innerloop_Actuator/Motor_C/Position Measurement/Initial/Enabled Subsystem1'
 * '<S61>'  : 'Ball_and_Plate_MicroLabBox_student/RTI Data/RTI Data Store'
 * '<S62>'  : 'Ball_and_Plate_MicroLabBox_student/RTI Data/RTI Data Store/RTI Data Store'
 * '<S63>'  : 'Ball_and_Plate_MicroLabBox_student/RTI Data/RTI Data Store/RTI Data Store/RTI Data Store'
 */
#endif                    /* RTW_HEADER_Ball_and_Plate_MicroLabBox_student_h_ */
