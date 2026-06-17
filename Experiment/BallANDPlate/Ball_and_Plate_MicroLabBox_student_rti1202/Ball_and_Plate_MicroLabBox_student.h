/*
 * Ball_and_Plate_MicroLabBox_student.h
 *
 * Academic License - for use in teaching, academic research, and meeting
 * course requirements at degree granting institutions only.  Not for
 * government, commercial, or other organizational use.
 *
 * Code generation for model "Ball_and_Plate_MicroLabBox_student".
 *
 * Model version              : 1.100
 * Simulink Coder version : 9.2 (R2019b) 18-Jul-2019
 * C source code generated on : Tue Jun 16 17:38:20 2026
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
#include "rtGetInf.h"
#include "rt_nonfinite.h"
#include "rt_zcfcn.h"
#include "rtGetNaN.h"

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
  real_T FirstOrderHold;               /* '<Root>/First Order Hold' */
  real_T FirstOrderHold1;              /* '<Root>/First Order Hold1' */
  real_T Switch[4];                    /* '<Root>/Switch' */
  real_T Add[4];                       /* '<Root>/Add' */
  real_T Optimalcontroller[2];         /* '<Root>/Optimal controller' */
  real_T Gain7;                        /* '<Root>/Gain7' */
  real_T TSamp;                        /* '<S2>/TSamp' */
  real_T Uk1_h;                        /* '<S2>/UD' */
  real_T Diff_e;                       /* '<S2>/Diff' */
  real_T Switch1;                      /* '<Root>/Switch1' */
  real_T Add1;                         /* '<Root>/Add1' */
  real_T TSamp_p;                      /* '<S3>/TSamp' */
  real_T Uk1_d;                        /* '<S3>/UD' */
  real_T Diff_e2;                      /* '<S3>/Diff' */
  real_T Switch2;                      /* '<Root>/Switch2' */
  real_T Add2;                         /* '<Root>/Add2' */
  real_T Gain4;                        /* '<Root>/Gain4' */
  real_T Alpha_sat;                    /* '<Root>/Alpha_sat ' */
  real_T Gain5;                        /* '<Root>/Gain5' */
  real_T Beta_sat;                     /* '<Root>/Beta_sat ' */
  real_T SFunction1[2];                /* '<S14>/S-Function1' */
  real_T Gain2;                        /* '<Root>/Gain2' */
  real_T Gain3;                        /* '<Root>/Gain3' */
  real_T Gain6[4];                     /* '<Root>/Gain6' */
  real_T Switch_h;                     /* '<S17>/Switch' */
  real_T enable_ref;                   /* '<S17>/enable_ref ' */
  real_T Sum1;                         /* '<S17>/Sum1' */
  real_T Gain1;                        /* '<S26>/Gain1' */
  real_T Dctleadlag2;                  /* '<S26>/Dctleadlag2' */
  real_T Dct1lowpass3;                 /* '<S26>/Dct1lowpass3' */
  real_T Switch2_k;                    /* '<S17>/Switch2' */
  real_T Sum;                          /* '<S17>/Sum' */
  real_T Switch3;                      /* '<S17>/Switch3' */
  real_T Switch_f;                     /* '<S21>/Switch' */
  real_T Constant1;                    /* '<S21>/Constant1' */
  real_T Constant2;                    /* '<S21>/Constant2' */
  real_T Switch1_j;                    /* '<S18>/Switch1' */
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
  real_T Gain1_m;                      /* '<S57>/Gain1' */
  real_T Dctleadlag2_a;                /* '<S57>/Dctleadlag2' */
  real_T Dct1lowpass3_e;               /* '<S57>/Dct1lowpass3' */
  real_T Switch2_h;                    /* '<S19>/Switch2' */
  real_T Sum_h;                        /* '<S19>/Sum' */
  real_T Switch3_l;                    /* '<S19>/Switch3' */
  real_T Switch_e;                     /* '<S51>/Switch' */
  real_T Constant1_h;                  /* '<S51>/Constant1' */
  real_T Constant2_h;                  /* '<S51>/Constant2' */
  real_T reser_integrator;             /* '<S5>/reser_integrator' */
  real_T RateTransition4;              /* '<Root>/Rate Transition4' */
  real_T RateTransition5;              /* '<Root>/Rate Transition5' */
  real_T Alpha_sine;                   /* '<Root>/Alpha_sine ' */
  real_T Beta_sine;                    /* '<Root>/Beta_sine' */
  real_T DataTypeConversion1[4];       /* '<S68>/Data Type Conversion1' */
  real_T DataTypeConversion10[2];      /* '<S68>/Data Type Conversion10' */
  real_T DataTypeConversion11;         /* '<S68>/Data Type Conversion11' */
  real_T DataTypeConversion12[2];      /* '<S68>/Data Type Conversion12' */
  real_T DataTypeConversion13[2];      /* '<S68>/Data Type Conversion13' */
  real_T DataTypeConversion14;         /* '<S68>/Data Type Conversion14' */
  real_T DataTypeConversion15;         /* '<S68>/Data Type Conversion15' */
  real_T DataTypeConversion2;          /* '<S68>/Data Type Conversion2' */
  real_T DataTypeConversion3[2];       /* '<S68>/Data Type Conversion3' */
  real_T DataTypeConversion4[2];       /* '<S68>/Data Type Conversion4' */
  real_T DataTypeConversion5[2];       /* '<S68>/Data Type Conversion5' */
  real_T DataTypeConversion6[4];       /* '<S68>/Data Type Conversion6' */
  real_T DataTypeConversion7[4];       /* '<S68>/Data Type Conversion7' */
  real_T DataTypeConversion8;          /* '<S68>/Data Type Conversion8' */
  real_T DataTypeConversion9[4];       /* '<S68>/Data Type Conversion9' */
  real_T EConversion[2];               /* '<S68>/E Conversion' */
  real_T FConversion[4];               /* '<S68>/F Conversion' */
  real_T GConversion;                  /* '<S68>/G Conversion' */
  real_T MathFunction[4];              /* '<S68>/Math Function' */
  real_T MathFunction1[2];             /* '<S68>/Math Function1' */
  real_T MathFunction2[2];             /* '<S68>/Math Function2' */
  real_T umin_scale4[2];               /* '<S68>/umin_scale4' */
  real_T Reshape[2];                   /* '<S68>/Reshape' */
  real_T ymin_scale1[4];               /* '<S68>/ymin_scale1' */
  real_T Reshape1[4];                  /* '<S68>/Reshape1' */
  real_T SConversion;                  /* '<S68>/S Conversion' */
  real_T ymin_scale2;                  /* '<S68>/ymin_scale2' */
  real_T Reshape2;                     /* '<S68>/Reshape2' */
  real_T Reshape3[4];                  /* '<S68>/Reshape3' */
  real_T Reshape4[2];                  /* '<S68>/Reshape4' */
  real_T Reshape5[2];                  /* '<S68>/Reshape5' */
  real_T extmv_scale[2];               /* '<S68>/ext.mv_scale' */
  real_T extmv_scale1[2];              /* '<S68>/ext.mv_scale1' */
  real_T last_mv[2];                   /* '<S68>/last_mv' */
  real_T last_x[4];                    /* '<S68>/last_x' */
  real_T moorxConversion[4];           /* '<S68>/mo or x Conversion' */
  real_T umin_scale[2];                /* '<S68>/umin_scale' */
  real_T umax_scale[2];                /* '<S68>/umax_scale' */
  real_T ymin_scale[4];                /* '<S68>/ymin_scale' */
  real_T ymax_scale[4];                /* '<S68>/ymax_scale' */
  real_T umin_scale1[2];               /* '<S68>/umin_scale1' */
  real_T umin_scale3[52];              /* '<S68>/umin_scale3' */
  real_T umin_scale5[104];             /* '<S68>/umin_scale5' */
  real_T umin_scale2[2];               /* '<S68>/umin_scale2' */
  real_T Constant5;                    /* '<Root>/Constant5' */
  real_T Constant6;                    /* '<Root>/Constant6' */
  real_T Gain1_k;                      /* '<S11>/Gain1' */
  real_T Dct1lowpass2;                 /* '<S11>/Dct1lowpass2' */
  real_T Gain1_p;                      /* '<S10>/Gain1' */
  real_T Dct1lowpass2_i;               /* '<S10>/Dct1lowpass2' */
  real_T xk1[4];                       /* '<S88>/optimizer' */
  real_T u[2];                         /* '<S88>/optimizer' */
  real_T cost;                         /* '<S88>/optimizer' */
  real_T useq[52];                     /* '<S88>/optimizer' */
  real_T xseq[104];                    /* '<S88>/optimizer' */
  real_T yseq[104];                    /* '<S88>/optimizer' */
  real_T status;                       /* '<S88>/optimizer' */
  real_T xest[4];                      /* '<S88>/optimizer' */
  real_T r[4];                         /* '<Root>/MATLAB Function3' */
  real_T TmpSignalConversionAtSFunctionI[2];/* '<Root>/MATLAB Function' */
  real_T TmpSignalConversionAtSFunctio_e[2];/* '<Root>/MATLAB Function' */
  real_T x_est[4];                     /* '<Root>/MATLAB Function' */
  real_T alpha;                        /* '<S5>/PosToAngle ' */
  real_T beta;                         /* '<S5>/PosToAngle ' */
  real_T psi;                          /* '<S5>/PosToAngle ' */
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
  real_T Add3;                         /* '<S5>/Add3' */
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
  real_T Add2_h;                       /* '<S5>/Add2' */
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
  real_T Add1_m;                       /* '<S5>/Add1' */
  real_T pos1;                         /* '<S5>/AngleToPos ' */
  real_T pos2;                         /* '<S5>/AngleToPos ' */
  real_T pos3;                         /* '<S5>/AngleToPos ' */
  real_T x;                            /* '<S4>/MATLAB Function' */
  real_T y;                            /* '<S4>/MATLAB Function' */
  real_T z;                            /* '<S4>/MATLAB Function' */
  real_T Gain8;                        /* '<Root>/Gain8' */
  real_T Gain9;                        /* '<Root>/Gain9' */
  uint32_T SFunction1_o4;              /* '<S13>/S-Function1' */
  uint32_T SFunction1_o1_m[3];         /* '<S12>/S-Function1' */
  uint16_T SFunction1_o6;              /* '<S13>/S-Function1' */
  uint8_T SFunction1_o1_c[12];         /* '<S13>/S-Function1' */
  uint8_T SFunction1_o5[4];            /* '<S13>/S-Function1' */
  uint8_T SFunction1_o2_f[4];          /* '<S12>/S-Function1' */
  uint8_T flag;                        /* '<S4>/MATLAB Function' */
  boolean_T DataTypeConversion;        /* '<S4>/Data Type Conversion' */
  boolean_T Memory[212];               /* '<S68>/Memory' */
  boolean_T iAout[212];                /* '<S88>/optimizer' */
  boolean_T RelationalOperator1;       /* '<S65>/Relational Operator1' */
  boolean_T RelationalOperator;        /* '<S65>/Relational Operator' */
  boolean_T LogicalOperator2;          /* '<S65>/Logical Operator2' */
  boolean_T RelationalOperator1_c;     /* '<S48>/Relational Operator1' */
  boolean_T RelationalOperator_n;      /* '<S48>/Relational Operator' */
  boolean_T LogicalOperator2_l;        /* '<S48>/Logical Operator2' */
  boolean_T RelationalOperator1_f;     /* '<S33>/Relational Operator1' */
  boolean_T RelationalOperator_a;      /* '<S33>/Relational Operator' */
  boolean_T LogicalOperator2_a;        /* '<S33>/Logical Operator2' */
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
  real_T Optimalcontroller_DSTATE[16]; /* '<Root>/Optimal controller' */
  real_T UD_DSTATE_c;                  /* '<S2>/UD' */
  real_T UD_DSTATE_o;                  /* '<S3>/UD' */
  real_T last_mv_DSTATE[2];            /* '<S68>/last_mv' */
  volatile real_T RateTransition3_Buffer0;/* '<Root>/Rate Transition3' */
  real_T Tk;                           /* '<Root>/First Order Hold' */
  real_T Ck;                           /* '<Root>/First Order Hold' */
  real_T Mk;                           /* '<Root>/First Order Hold' */
  real_T Uk;                           /* '<Root>/First Order Hold' */
  real_T Tk_c;                         /* '<Root>/First Order Hold1' */
  real_T Ck_h;                         /* '<Root>/First Order Hold1' */
  real_T Mk_g;                         /* '<Root>/First Order Hold1' */
  real_T Uk_i;                         /* '<Root>/First Order Hold1' */
  volatile real_T RateTransition4_Buffer0;/* '<Root>/Rate Transition4' */
  volatile real_T RateTransition5_Buffer0;/* '<Root>/Rate Transition5' */
  real_T last_x_PreviousInput[4];      /* '<S68>/last_x' */
  real_T theta;                        /* '<Root>/MATLAB Function3' */
  real_T current_R;                    /* '<Root>/MATLAB Function3' */
  real_T x_hat[4];                     /* '<Root>/MATLAB Function' */
  real_T P[16];                        /* '<Root>/MATLAB Function' */
  struct {
    real_T RECEIVED_FRAMES;
  } SFunction1_RWORK;                  /* '<S13>/S-Function1' */

  struct {
    real_T RX_DROPPED_FRAMES[2];
  } SFunction1_RWORK_l;                /* '<S14>/S-Function1' */

  real_T Dctleadlag2_RWORK[2];         /* '<S26>/Dctleadlag2' */
  real_T Dct1lowpass3_RWORK[2];        /* '<S26>/Dct1lowpass3' */
  real_T Dctleadlag2_RWORK_o[2];       /* '<S41>/Dctleadlag2' */
  real_T Dct1lowpass3_RWORK_b[2];      /* '<S41>/Dct1lowpass3' */
  real_T Dctleadlag2_RWORK_p[2];       /* '<S57>/Dctleadlag2' */
  real_T Dct1lowpass3_RWORK_d[2];      /* '<S57>/Dct1lowpass3' */
  real_T Dct1lowpass2_RWORK[2];        /* '<S11>/Dct1lowpass2' */
  real_T Dct1lowpass2_RWORK_a[2];      /* '<S10>/Dct1lowpass2' */
  volatile int8_T RateTransition3_semaphoreTaken;/* '<Root>/Rate Transition3' */
  volatile int8_T RateTransition4_semaphoreTaken;/* '<Root>/Rate Transition4' */
  volatile int8_T RateTransition5_semaphoreTaken;/* '<Root>/Rate Transition5' */
  boolean_T Memory_PreviousInput[212]; /* '<S68>/Memory' */
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
  real_T A_robust[256];                /* Variable: A_robust
                                        * Referenced by: '<Root>/Optimal controller'
                                        */
  real_T B_robust[64];                 /* Variable: B_robust
                                        * Referenced by: '<Root>/Optimal controller'
                                        */
  real_T C_robust[32];                 /* Variable: C_robust
                                        * Referenced by: '<Root>/Optimal controller'
                                        */
  real_T D_robust[8];                  /* Variable: D_robust
                                        * Referenced by: '<Root>/Optimal controller'
                                        */
  real_T Q_kf[16];                     /* Variable: Q_kf
                                        * Referenced by: '<Root>/Constant'
                                        */
  real_T R_kf[4];                      /* Variable: R_kf
                                        * Referenced by: '<Root>/Constant1'
                                        */
  real_T Ts_Inner;                     /* Variable: Ts_Inner
                                        * Referenced by:
                                        *   '<Root>/Constant2'
                                        *   '<Root>/Constant4'
                                        */
  real_T Ts_Outer;                     /* Variable: Ts_Outer
                                        * Referenced by:
                                        *   '<Root>/Constant5'
                                        *   '<Root>/Gain'
                                        */
  real_T Ts_fast;                      /* Variable: Ts_fast
                                        * Referenced by: '<Root>/Constant6'
                                        */
  real_T ball_pos_sat;                 /* Variable: ball_pos_sat
                                        * Referenced by:
                                        *   '<Root>/Saturation2'
                                        *   '<Root>/Saturation3'
                                        */
  real_T ball_vel_sat;                 /* Variable: ball_vel_sat
                                        * Referenced by:
                                        *   '<Root>/Saturation'
                                        *   '<Root>/Saturation1'
                                        */
  real_T g;                            /* Variable: g
                                        * Referenced by:
                                        *   '<Root>/Gain8'
                                        *   '<Root>/Gain9'
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
  real_T Difference_ICPrevInput;       /* Mask Parameter: Difference_ICPrevInput
                                        * Referenced by: '<S1>/UD'
                                        */
  real_T DiscreteDerivative_ICPrevScaled;
                              /* Mask Parameter: DiscreteDerivative_ICPrevScaled
                               * Referenced by: '<S2>/UD'
                               */
  real_T DiscreteDerivative1_ICPrevScale;
                              /* Mask Parameter: DiscreteDerivative1_ICPrevScale
                               * Referenced by: '<S3>/UD'
                               */
  real_T Constant8_Value;              /* Expression: 0
                                        * Referenced by: '<Root>/Constant8'
                                        */
  real_T Constant7_Value;              /* Expression: 0
                                        * Referenced by: '<Root>/Constant7'
                                        */
  real_T Constant3_Value[4];           /* Expression: [0;0;0;0]
                                        * Referenced by: '<Root>/Constant3'
                                        */
  real_T Constant_Value;               /* Expression: 0.32
                                        * Referenced by: '<S5>/Constant'
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
                                        * Referenced by: '<S5>/Constant1'
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
                                        * Referenced by: '<S5>/Constant2'
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
                                        * Referenced by: '<S4>/Constant'
                                        */
  real_T FirstOrderHold_IniOut;        /* Expression: 0
                                        * Referenced by: '<Root>/First Order Hold'
                                        */
  real_T FirstOrderHold_ErrTol;        /* Expression: inf
                                        * Referenced by: '<Root>/First Order Hold'
                                        */
  real_T FirstOrderHold1_IniOut;       /* Expression: 0
                                        * Referenced by: '<Root>/First Order Hold1'
                                        */
  real_T FirstOrderHold1_ErrTol;       /* Expression: inf
                                        * Referenced by: '<Root>/First Order Hold1'
                                        */
  real_T Init_Value;                   /* Expression: 0
                                        * Referenced by: '<S5>/Init  '
                                        */
  real_T Outer_loop_enable_Value;      /* Expression: 0
                                        * Referenced by: '<Root>/Outer_loop_enable'
                                        */
  real_T Switch_Threshold;             /* Expression: 0
                                        * Referenced by: '<Root>/Switch'
                                        */
  real_T Constant9_Value[4];           /* Expression: [0;0;0;0]
                                        * Referenced by: '<Root>/Constant9'
                                        */
  real_T Optimalcontroller_InitialCondit;/* Expression: 0
                                          * Referenced by: '<Root>/Optimal controller'
                                          */
  real_T radius_circ_Value;            /* Expression: 0.1
                                        * Referenced by: '<Root>/radius_circ'
                                        */
  real_T ball_freq_Value;              /* Expression: 1/10
                                        * Referenced by: '<Root>/ball_freq'
                                        */
  real_T Gain7_Gain;                   /* Expression: 2*pi
                                        * Referenced by: '<Root>/Gain7'
                                        */
  real_T enable_circ_Value;            /* Expression: 0
                                        * Referenced by: '<Root>/enable_circ'
                                        */
  real_T TSamp_WtEt;                   /* Computed Parameter: TSamp_WtEt
                                        * Referenced by: '<S2>/TSamp'
                                        */
  real_T ff_enable_Value;              /* Expression: 0
                                        * Referenced by: '<Root>/ff_enable'
                                        */
  real_T Switch1_Threshold;            /* Expression: 0
                                        * Referenced by: '<Root>/Switch1'
                                        */
  real_T TSamp_WtEt_b;                 /* Computed Parameter: TSamp_WtEt_b
                                        * Referenced by: '<S3>/TSamp'
                                        */
  real_T Switch2_Threshold;            /* Expression: 0
                                        * Referenced by: '<Root>/Switch2'
                                        */
  real_T Gain4_Gain;                   /* Expression: -1
                                        * Referenced by: '<Root>/Gain4'
                                        */
  real_T Gain5_Gain;                   /* Expression: -1
                                        * Referenced by: '<Root>/Gain5'
                                        */
  real_T Gain2_Gain;                   /* Expression: 1/1000
                                        * Referenced by: '<Root>/Gain2'
                                        */
  real_T Gain3_Gain;                   /* Expression: 1/1000
                                        * Referenced by: '<Root>/Gain3'
                                        */
  real_T Gain6_Gain;                   /* Expression: -1
                                        * Referenced by: '<Root>/Gain6'
                                        */
  real_T Psi_ref_Value;                /* Expression: 0
                                        * Referenced by: '<Root>/Psi_ref '
                                        */
  real_T CloseLoop_disable_Value;      /* Expression: 1
                                        * Referenced by: '<S5>/CloseLoop_disable'
                                        */
  real_T controller_disable_Value;     /* Expression: 1
                                        * Referenced by: '<S5>/controller_disable'
                                        */
  real_T enable_quintic_Value;         /* Expression: 1
                                        * Referenced by: '<S5>/enable_quintic  '
                                        */
  real_T quintic_ref_Value;            /* Expression: 0
                                        * Referenced by: '<S5>/quintic_ref '
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
                                        * Referenced by: '<S5>/enable_ref '
                                        */
  real_T enable_ref_Threshold;         /* Expression: 0
                                        * Referenced by: '<S17>/enable_ref '
                                        */
  real_T Gain1_Gain;                   /* Expression: 1300
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
  real_T Switch2_Threshold_g;          /* Expression: 0.5
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
  real_T Switch1_Threshold_j;          /* Expression: 0.5
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
  real_T Gain1_Gain_l;                 /* Expression: 1200
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
  real_T Switch1_Threshold_jm;         /* Expression: 0.5
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
  real_T Gain1_Gain_p;                 /* Expression: 350
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
                                        * Referenced by: '<S5>/reser_integrator'
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
  real_T Constant10_Value[4];          /* Expression: [0;0;0;0]
                                        * Referenced by: '<Root>/Constant10'
                                        */
  real_T Constant11_Value[4];          /* Expression: [0;0;0;0]
                                        * Referenced by: '<Root>/Constant11'
                                        */
  real_T E_zero_Value[2];              /* Expression: zeros(1,2)
                                        * Referenced by: '<S8>/E_zero'
                                        */
  real_T F_zero_Value[4];              /* Expression: zeros(1,4)
                                        * Referenced by: '<S8>/F_zero'
                                        */
  real_T G_zero_Value;                 /* Expression: zeros(1,1)
                                        * Referenced by: '<S8>/G_zero'
                                        */
  real_T uwt_zero_Value[2];            /* Expression: zeros(2,1)
                                        * Referenced by: '<S8>/u.wt_zero'
                                        */
  real_T ecrwt_zero_Value;             /* Expression: zeros(1,1)
                                        * Referenced by: '<S8>/ecr.wt_zero'
                                        */
  real_T duwt_zero_Value[2];           /* Expression: zeros(2,1)
                                        * Referenced by: '<S8>/du.wt_zero'
                                        */
  real_T mvtarget_zero_Value[2];       /* Expression: zeros(2,1)
                                        * Referenced by: '<S8>/mv.target_zero'
                                        */
  real_T p_zero_Value;                 /* Expression: zeros(1,1)
                                        * Referenced by: '<S8>/p_zero'
                                        */
  real_T m_zero_Value;                 /* Expression: zeros(1,1)
                                        * Referenced by: '<S8>/m_zero'
                                        */
  real_T md_zero_Value;                /* Expression: zeros(1,1)
                                        * Referenced by: '<S8>/md_zero'
                                        */
  real_T extmv_zero_Value[2];          /* Expression: zeros(2,1)
                                        * Referenced by: '<S8>/ext.mv_zero'
                                        */
  real_T umin_zero_Value[2];           /* Expression: zeros(2,1)
                                        * Referenced by: '<S8>/umin_zero'
                                        */
  real_T umax_zero_Value[2];           /* Expression: zeros(2,1)
                                        * Referenced by: '<S8>/umax_zero'
                                        */
  real_T ymin_zero_Value[4];           /* Expression: zeros(4,1)
                                        * Referenced by: '<S8>/ymin_zero'
                                        */
  real_T ymax_zero_Value[4];           /* Expression: zeros(4,1)
                                        * Referenced by: '<S8>/ymax_zero'
                                        */
  real_T switch_zero_Value;            /* Expression: zeros(1,1)
                                        * Referenced by: '<S8>/switch_zero'
                                        */
  real_T ywt_zero_Value[4];            /* Expression: zeros(4,1)
                                        * Referenced by: '<S8>/y.wt_zero'
                                        */
  real_T umin_scale4_Gain[2];      /* Expression: MVscale(:,ones(1,max(nCC,1)))'
                                    * Referenced by: '<S68>/umin_scale4'
                                    */
  real_T ymin_scale1_Gain[4];       /* Expression: Yscale(:,ones(1,max(nCC,1)))'
                                     * Referenced by: '<S68>/ymin_scale1'
                                     */
  real_T S_zero_Value;                 /* Expression: zeros(1,1)
                                        * Referenced by: '<S8>/S_zero'
                                        */
  real_T ymin_scale2_Gain;         /* Expression: MDscale(:,ones(1,max(nCC,1)))'
                                    * Referenced by: '<S68>/ymin_scale2'
                                    */
  real_T extmv_scale_Gain[2];          /* Expression: RMVscale
                                        * Referenced by: '<S68>/ext.mv_scale'
                                        */
  real_T extmv_scale1_Gain[2];         /* Expression: RMVscale
                                        * Referenced by: '<S68>/ext.mv_scale1'
                                        */
  real_T last_mv_InitialCondition[2];  /* Expression: lastu+uoff
                                        * Referenced by: '<S68>/last_mv'
                                        */
  real_T last_x_InitialCondition[4];   /* Expression: lastx+xoff
                                        * Referenced by: '<S68>/last_x'
                                        */
  real_T ym_zero_Value[4];             /* Expression: zeros(nym,1)
                                        * Referenced by: '<S68>/ym_zero'
                                        */
  real_T umin_scale_Gain[2];           /* Expression: RMVscale
                                        * Referenced by: '<S68>/umin_scale'
                                        */
  real_T umax_scale_Gain[2];           /* Expression: RMVscale
                                        * Referenced by: '<S68>/umax_scale'
                                        */
  real_T ymin_scale_Gain[4];           /* Expression: RYscale
                                        * Referenced by: '<S68>/ymin_scale'
                                        */
  real_T ymax_scale_Gain[4];           /* Expression: RYscale
                                        * Referenced by: '<S68>/ymax_scale'
                                        */
  real_T umin_scale1_Gain[2];          /* Expression: MVscale
                                        * Referenced by: '<S68>/umin_scale1'
                                        */
  real_T umin_scale3_Gain[52];         /* Expression: MVscale(:,ones(1,p+1))'
                                        * Referenced by: '<S68>/umin_scale3'
                                        */
  real_T umin_scale5_Gain[104];        /* Expression: Yscale(:,ones(1,p+1))'
                                        * Referenced by: '<S68>/umin_scale5'
                                        */
  real_T constant_Value[2];            /* Expression: lastu+uoff
                                        * Referenced by: '<S68>/constant'
                                        */
  real_T umin_scale2_Gain[2];          /* Expression: MVscale
                                        * Referenced by: '<S68>/umin_scale2'
                                        */
  real_T Gain1_Gain_i;                 /* Expression: 1
                                        * Referenced by: '<S11>/Gain1'
                                        */
  real_T Dct1lowpass2_P1_Size[2];    /* Computed Parameter: Dct1lowpass2_P1_Size
                                      * Referenced by: '<S11>/Dct1lowpass2'
                                      */
  real_T Dct1lowpass2_P1;              /* Expression: f_den
                                        * Referenced by: '<S11>/Dct1lowpass2'
                                        */
  real_T Dct1lowpass2_P2_Size[2];    /* Computed Parameter: Dct1lowpass2_P2_Size
                                      * Referenced by: '<S11>/Dct1lowpass2'
                                      */
  real_T Dct1lowpass2_P2;              /* Expression: 0.001
                                        * Referenced by: '<S11>/Dct1lowpass2'
                                        */
  real_T Gain1_Gain_a;                 /* Expression: 1
                                        * Referenced by: '<S10>/Gain1'
                                        */
  real_T Dct1lowpass2_P1_Size_f[2];/* Computed Parameter: Dct1lowpass2_P1_Size_f
                                    * Referenced by: '<S10>/Dct1lowpass2'
                                    */
  real_T Dct1lowpass2_P1_j;            /* Expression: f_den
                                        * Referenced by: '<S10>/Dct1lowpass2'
                                        */
  real_T Dct1lowpass2_P2_Size_l[2];/* Computed Parameter: Dct1lowpass2_P2_Size_l
                                    * Referenced by: '<S10>/Dct1lowpass2'
                                    */
  real_T Dct1lowpass2_P2_j;            /* Expression: 0.001
                                        * Referenced by: '<S10>/Dct1lowpass2'
                                        */
  uint16_T MatrixDimensionCheck_P1[49];
                                  /* Computed Parameter: MatrixDimensionCheck_P1
                                   * Referenced by: '<S69>/Matrix Dimension Check'
                                   */
  uint16_T MatrixDimensionCheck_P2[18];
                                  /* Computed Parameter: MatrixDimensionCheck_P2
                                   * Referenced by: '<S69>/Matrix Dimension Check'
                                   */
  uint16_T MatrixDimensionCheck_P3;    /* Expression: nrow
                                        * Referenced by: '<S69>/Matrix Dimension Check'
                                        */
  uint16_T MatrixDimensionCheck_P4;    /* Expression: ncol
                                        * Referenced by: '<S69>/Matrix Dimension Check'
                                        */
  uint16_T MatrixDimensionCheck_P5;    /* Expression: nsteps
                                        * Referenced by: '<S69>/Matrix Dimension Check'
                                        */
  uint16_T MatrixDimensionCheck_P6;    /* Expression: isltv
                                        * Referenced by: '<S69>/Matrix Dimension Check'
                                        */
  uint16_T MatrixDimensionCheck_P1_o[49];
                                /* Computed Parameter: MatrixDimensionCheck_P1_o
                                 * Referenced by: '<S70>/Matrix Dimension Check'
                                 */
  uint16_T MatrixDimensionCheck_P2_b[18];
                                /* Computed Parameter: MatrixDimensionCheck_P2_b
                                 * Referenced by: '<S70>/Matrix Dimension Check'
                                 */
  uint16_T MatrixDimensionCheck_P3_i;  /* Expression: nrow
                                        * Referenced by: '<S70>/Matrix Dimension Check'
                                        */
  uint16_T MatrixDimensionCheck_P4_p;  /* Expression: ncol
                                        * Referenced by: '<S70>/Matrix Dimension Check'
                                        */
  uint16_T MatrixDimensionCheck_P5_b;  /* Expression: nsteps
                                        * Referenced by: '<S70>/Matrix Dimension Check'
                                        */
  uint16_T MatrixDimensionCheck_P6_p;  /* Expression: isltv
                                        * Referenced by: '<S70>/Matrix Dimension Check'
                                        */
  uint16_T MatrixDimensionCheck_P1_e[49];
                                /* Computed Parameter: MatrixDimensionCheck_P1_e
                                 * Referenced by: '<S71>/Matrix Dimension Check'
                                 */
  uint16_T MatrixDimensionCheck_P2_o[18];
                                /* Computed Parameter: MatrixDimensionCheck_P2_o
                                 * Referenced by: '<S71>/Matrix Dimension Check'
                                 */
  uint16_T MatrixDimensionCheck_P3_f;  /* Expression: nrow
                                        * Referenced by: '<S71>/Matrix Dimension Check'
                                        */
  uint16_T MatrixDimensionCheck_P4_d;  /* Expression: ncol
                                        * Referenced by: '<S71>/Matrix Dimension Check'
                                        */
  uint16_T MatrixDimensionCheck_P5_l;  /* Expression: nsteps
                                        * Referenced by: '<S71>/Matrix Dimension Check'
                                        */
  uint16_T MatrixDimensionCheck_P6_ph; /* Expression: isltv
                                        * Referenced by: '<S71>/Matrix Dimension Check'
                                        */
  uint16_T MatrixDimensionCheck_P1_l[49];
                                /* Computed Parameter: MatrixDimensionCheck_P1_l
                                 * Referenced by: '<S72>/Matrix Dimension Check'
                                 */
  uint16_T MatrixDimensionCheck_P2_bz[3];
                               /* Computed Parameter: MatrixDimensionCheck_P2_bz
                                * Referenced by: '<S72>/Matrix Dimension Check'
                                */
  uint16_T MatrixDimensionCheck_P3_d;  /* Expression: nrow
                                        * Referenced by: '<S72>/Matrix Dimension Check'
                                        */
  uint16_T MatrixDimensionCheck_P4_h;  /* Expression: ncol
                                        * Referenced by: '<S72>/Matrix Dimension Check'
                                        */
  uint16_T MatrixDimensionCheck_P1_i[49];
                                /* Computed Parameter: MatrixDimensionCheck_P1_i
                                 * Referenced by: '<S73>/Matrix Dimension Check'
                                 */
  uint16_T MatrixDimensionCheck_P2_g[2];
                                /* Computed Parameter: MatrixDimensionCheck_P2_g
                                 * Referenced by: '<S73>/Matrix Dimension Check'
                                 */
  uint16_T MatrixDimensionCheck_P3_a;  /* Expression: nrow
                                        * Referenced by: '<S73>/Matrix Dimension Check'
                                        */
  uint16_T MatrixDimensionCheck_P4_b;  /* Expression: ncol
                                        * Referenced by: '<S73>/Matrix Dimension Check'
                                        */
  uint16_T MatrixDimensionCheck_P1_o1[49];
                               /* Computed Parameter: MatrixDimensionCheck_P1_o1
                                * Referenced by: '<S74>/Matrix Dimension Check'
                                */
  uint16_T MatrixDimensionCheck_P2_n[4];
                                /* Computed Parameter: MatrixDimensionCheck_P2_n
                                 * Referenced by: '<S74>/Matrix Dimension Check'
                                 */
  uint16_T MatrixDimensionCheck_P3_b;  /* Expression: nrow
                                        * Referenced by: '<S74>/Matrix Dimension Check'
                                        */
  uint16_T MatrixDimensionCheck_P4_po; /* Expression: ncol
                                        * Referenced by: '<S74>/Matrix Dimension Check'
                                        */
  uint16_T MatrixDimensionCheck_P1_f[49];
                                /* Computed Parameter: MatrixDimensionCheck_P1_f
                                 * Referenced by: '<S75>/Matrix Dimension Check'
                                 */
  uint16_T MatrixDimensionCheck_P2_k[4];
                                /* Computed Parameter: MatrixDimensionCheck_P2_k
                                 * Referenced by: '<S75>/Matrix Dimension Check'
                                 */
  uint16_T MatrixDimensionCheck_P3_l;  /* Expression: nrow
                                        * Referenced by: '<S75>/Matrix Dimension Check'
                                        */
  uint16_T MatrixDimensionCheck_P4_g;  /* Expression: ncol
                                        * Referenced by: '<S75>/Matrix Dimension Check'
                                        */
  uint16_T MatrixDimensionCheck_P1_n[49];
                                /* Computed Parameter: MatrixDimensionCheck_P1_n
                                 * Referenced by: '<S76>/Matrix Dimension Check'
                                 */
  uint16_T MatrixDimensionCheck_P2_f[5];
                                /* Computed Parameter: MatrixDimensionCheck_P2_f
                                 * Referenced by: '<S76>/Matrix Dimension Check'
                                 */
  uint16_T MatrixDimensionCheck_P3_lv; /* Expression: nrow
                                        * Referenced by: '<S76>/Matrix Dimension Check'
                                        */
  uint16_T MatrixDimensionCheck_P4_a;  /* Expression: ncol
                                        * Referenced by: '<S76>/Matrix Dimension Check'
                                        */
  uint16_T VectorDimensionCheck_P1[49];
                                  /* Computed Parameter: VectorDimensionCheck_P1
                                   * Referenced by: '<S77>/Vector Dimension Check'
                                   */
  uint16_T VectorDimensionCheck_P2[6];
                                  /* Computed Parameter: VectorDimensionCheck_P2
                                   * Referenced by: '<S77>/Vector Dimension Check'
                                   */
  uint16_T VectorDimensionCheck_P1_c[49];
                                /* Computed Parameter: VectorDimensionCheck_P1_c
                                 * Referenced by: '<S78>/Vector Dimension Check'
                                 */
  uint16_T VectorDimensionCheck_P2_i[6];
                                /* Computed Parameter: VectorDimensionCheck_P2_i
                                 * Referenced by: '<S78>/Vector Dimension Check'
                                 */
  uint16_T VectorDimensionCheck_P1_a[49];
                                /* Computed Parameter: VectorDimensionCheck_P1_a
                                 * Referenced by: '<S79>/Vector Dimension Check'
                                 */
  uint16_T VectorDimensionCheck_P2_m;
                                /* Computed Parameter: VectorDimensionCheck_P2_m
                                 * Referenced by: '<S79>/Vector Dimension Check'
                                 */
  uint16_T VectorDimensionCheck_P1_b[49];
                                /* Computed Parameter: VectorDimensionCheck_P1_b
                                 * Referenced by: '<S80>/Vector Dimension Check'
                                 */
  uint16_T VectorDimensionCheck_P2_o[6];
                                /* Computed Parameter: VectorDimensionCheck_P2_o
                                 * Referenced by: '<S80>/Vector Dimension Check'
                                 */
  uint16_T VectorDimensionCheck_P3;    /* Expression: n
                                        * Referenced by: '<S80>/Vector Dimension Check'
                                        */
  uint16_T VectorDimensionCheck_P4;    /* Expression: option
                                        * Referenced by: '<S80>/Vector Dimension Check'
                                        */
  uint16_T VectorDimensionCheck_P1_k[49];
                                /* Computed Parameter: VectorDimensionCheck_P1_k
                                 * Referenced by: '<S81>/Vector Dimension Check'
                                 */
  uint16_T VectorDimensionCheck_P2_n[18];
                                /* Computed Parameter: VectorDimensionCheck_P2_n
                                 * Referenced by: '<S81>/Vector Dimension Check'
                                 */
  uint16_T VectorDimensionCheck_P3_i;  /* Expression: n
                                        * Referenced by: '<S81>/Vector Dimension Check'
                                        */
  uint16_T VectorDimensionCheck_P4_n;  /* Expression: option
                                        * Referenced by: '<S81>/Vector Dimension Check'
                                        */
  uint16_T VectorDimensionCheck_P1_j[49];
                                /* Computed Parameter: VectorDimensionCheck_P1_j
                                 * Referenced by: '<S82>/Vector Dimension Check'
                                 */
  uint16_T VectorDimensionCheck_P2_iv[6];
                               /* Computed Parameter: VectorDimensionCheck_P2_iv
                                * Referenced by: '<S82>/Vector Dimension Check'
                                */
  uint16_T VectorDimensionCheck_P3_d;  /* Expression: n
                                        * Referenced by: '<S82>/Vector Dimension Check'
                                        */
  uint16_T VectorDimensionCheck_P4_e;  /* Expression: option
                                        * Referenced by: '<S82>/Vector Dimension Check'
                                        */
  uint16_T VectorDimensionCheck_P1_ba[49];
                               /* Computed Parameter: VectorDimensionCheck_P1_ba
                                * Referenced by: '<S83>/Vector Dimension Check'
                                */
  uint16_T VectorDimensionCheck_P2_ok[4];
                               /* Computed Parameter: VectorDimensionCheck_P2_ok
                                * Referenced by: '<S83>/Vector Dimension Check'
                                */
  uint16_T VectorDimensionCheck_P3_d5; /* Expression: n
                                        * Referenced by: '<S83>/Vector Dimension Check'
                                        */
  uint16_T VectorDimensionCheck_P4_h;  /* Expression: option
                                        * Referenced by: '<S83>/Vector Dimension Check'
                                        */
  uint16_T VectorDimensionCheck_P1_kq[49];
                               /* Computed Parameter: VectorDimensionCheck_P1_kq
                                * Referenced by: '<S84>/Vector Dimension Check'
                                */
  uint16_T VectorDimensionCheck_P2_a[4];
                                /* Computed Parameter: VectorDimensionCheck_P2_a
                                 * Referenced by: '<S84>/Vector Dimension Check'
                                 */
  uint16_T VectorDimensionCheck_P3_c;  /* Expression: n
                                        * Referenced by: '<S84>/Vector Dimension Check'
                                        */
  uint16_T VectorDimensionCheck_P4_a;  /* Expression: option
                                        * Referenced by: '<S84>/Vector Dimension Check'
                                        */
  uint16_T VectorDimensionCheck_P1_h[49];
                                /* Computed Parameter: VectorDimensionCheck_P1_h
                                 * Referenced by: '<S85>/Vector Dimension Check'
                                 */
  uint16_T VectorDimensionCheck_P2_oy[4];
                               /* Computed Parameter: VectorDimensionCheck_P2_oy
                                * Referenced by: '<S85>/Vector Dimension Check'
                                */
  uint16_T VectorDimensionCheck_P3_e;  /* Expression: n
                                        * Referenced by: '<S85>/Vector Dimension Check'
                                        */
  uint16_T VectorDimensionCheck_P4_p;  /* Expression: option
                                        * Referenced by: '<S85>/Vector Dimension Check'
                                        */
  uint16_T VectorDimensionCheck_P1_d[49];
                                /* Computed Parameter: VectorDimensionCheck_P1_d
                                 * Referenced by: '<S86>/Vector Dimension Check'
                                 */
  uint16_T VectorDimensionCheck_P2_as[4];
                               /* Computed Parameter: VectorDimensionCheck_P2_as
                                * Referenced by: '<S86>/Vector Dimension Check'
                                */
  uint16_T VectorDimensionCheck_P3_m;  /* Expression: n
                                        * Referenced by: '<S86>/Vector Dimension Check'
                                        */
  uint16_T VectorDimensionCheck_P4_j;  /* Expression: option
                                        * Referenced by: '<S86>/Vector Dimension Check'
                                        */
  uint16_T VectorDimensionCheck_P1_bz[49];
                               /* Computed Parameter: VectorDimensionCheck_P1_bz
                                * Referenced by: '<S87>/Vector Dimension Check'
                                */
  uint16_T VectorDimensionCheck_P2_am[9];
                               /* Computed Parameter: VectorDimensionCheck_P2_am
                                * Referenced by: '<S87>/Vector Dimension Check'
                                */
  uint16_T VectorDimensionCheck_P3_cy; /* Expression: n
                                        * Referenced by: '<S87>/Vector Dimension Check'
                                        */
  uint16_T VectorDimensionCheck_P4_et; /* Expression: option
                                        * Referenced by: '<S87>/Vector Dimension Check'
                                        */
  boolean_T Memory_InitialCondition[212];/* Expression: iA
                                          * Referenced by: '<S68>/Memory'
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
 * '<S2>'   : 'Ball_and_Plate_MicroLabBox_student/Discrete Derivative'
 * '<S3>'   : 'Ball_and_Plate_MicroLabBox_student/Discrete Derivative1'
 * '<S4>'   : 'Ball_and_Plate_MicroLabBox_student/Ethernet communication'
 * '<S5>'   : 'Ball_and_Plate_MicroLabBox_student/Innerloop_Actuator'
 * '<S6>'   : 'Ball_and_Plate_MicroLabBox_student/MATLAB Function'
 * '<S7>'   : 'Ball_and_Plate_MicroLabBox_student/MATLAB Function3'
 * '<S8>'   : 'Ball_and_Plate_MicroLabBox_student/MPC Controller'
 * '<S9>'   : 'Ball_and_Plate_MicroLabBox_student/RTI Data'
 * '<S10>'  : 'Ball_and_Plate_MicroLabBox_student/lowpass '
 * '<S11>'  : 'Ball_and_Plate_MicroLabBox_student/lowpass 6'
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
 * '<S68>'  : 'Ball_and_Plate_MicroLabBox_student/MPC Controller/MPC'
 * '<S69>'  : 'Ball_and_Plate_MicroLabBox_student/MPC Controller/MPC/MPC Matrix Signal Check'
 * '<S70>'  : 'Ball_and_Plate_MicroLabBox_student/MPC Controller/MPC/MPC Matrix Signal Check1'
 * '<S71>'  : 'Ball_and_Plate_MicroLabBox_student/MPC Controller/MPC/MPC Matrix Signal Check2'
 * '<S72>'  : 'Ball_and_Plate_MicroLabBox_student/MPC Controller/MPC/MPC Preview Signal Check'
 * '<S73>'  : 'Ball_and_Plate_MicroLabBox_student/MPC Controller/MPC/MPC Preview Signal Check1'
 * '<S74>'  : 'Ball_and_Plate_MicroLabBox_student/MPC Controller/MPC/MPC Preview Signal Check2'
 * '<S75>'  : 'Ball_and_Plate_MicroLabBox_student/MPC Controller/MPC/MPC Preview Signal Check3'
 * '<S76>'  : 'Ball_and_Plate_MicroLabBox_student/MPC Controller/MPC/MPC Preview Signal Check4'
 * '<S77>'  : 'Ball_and_Plate_MicroLabBox_student/MPC Controller/MPC/MPC Scalar Signal Check'
 * '<S78>'  : 'Ball_and_Plate_MicroLabBox_student/MPC Controller/MPC/MPC Scalar Signal Check1'
 * '<S79>'  : 'Ball_and_Plate_MicroLabBox_student/MPC Controller/MPC/MPC Scalar Signal Check2'
 * '<S80>'  : 'Ball_and_Plate_MicroLabBox_student/MPC Controller/MPC/MPC Vector Signal Check'
 * '<S81>'  : 'Ball_and_Plate_MicroLabBox_student/MPC Controller/MPC/MPC Vector Signal Check1'
 * '<S82>'  : 'Ball_and_Plate_MicroLabBox_student/MPC Controller/MPC/MPC Vector Signal Check11'
 * '<S83>'  : 'Ball_and_Plate_MicroLabBox_student/MPC Controller/MPC/MPC Vector Signal Check2'
 * '<S84>'  : 'Ball_and_Plate_MicroLabBox_student/MPC Controller/MPC/MPC Vector Signal Check3'
 * '<S85>'  : 'Ball_and_Plate_MicroLabBox_student/MPC Controller/MPC/MPC Vector Signal Check4'
 * '<S86>'  : 'Ball_and_Plate_MicroLabBox_student/MPC Controller/MPC/MPC Vector Signal Check5'
 * '<S87>'  : 'Ball_and_Plate_MicroLabBox_student/MPC Controller/MPC/MPC Vector Signal Check6'
 * '<S88>'  : 'Ball_and_Plate_MicroLabBox_student/MPC Controller/MPC/optimizer'
 * '<S89>'  : 'Ball_and_Plate_MicroLabBox_student/MPC Controller/MPC/optimizer/optimizer'
 * '<S90>'  : 'Ball_and_Plate_MicroLabBox_student/RTI Data/RTI Data Store'
 * '<S91>'  : 'Ball_and_Plate_MicroLabBox_student/RTI Data/RTI Data Store/RTI Data Store'
 * '<S92>'  : 'Ball_and_Plate_MicroLabBox_student/RTI Data/RTI Data Store/RTI Data Store/RTI Data Store'
 */
#endif                    /* RTW_HEADER_Ball_and_Plate_MicroLabBox_student_h_ */
