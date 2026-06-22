/*
 * Ball_and_Plate_MicroLabBox_student.h
 *
 * Academic License - for use in teaching, academic research, and meeting
 * course requirements at degree granting institutions only.  Not for
 * government, commercial, or other organizational use.
 *
 * Code generation for model "Ball_and_Plate_MicroLabBox_student".
 *
 * Model version              : 1.131
 * Simulink Coder version : 9.2 (R2019b) 18-Jul-2019
 * C source code generated on : Mon Jun 22 18:00:50 2026
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
#include "rtGetNaN.h"
#include "rt_zcfcn.h"

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

/* Block signals for system '<S18>/MATLAB Function1' */
typedef struct {
  real_T path;                         /* '<S18>/MATLAB Function1' */
} B_MATLABFunction1_Ball_and_Pl_T;

/* Block states (default storage) for system '<S18>/MATLAB Function1' */
typedef struct {
  real_T coeffs[6];                    /* '<S18>/MATLAB Function1' */
  real_T t_elapsed;                    /* '<S18>/MATLAB Function1' */
  real_T prev_ref_end;                 /* '<S18>/MATLAB Function1' */
  real_T tf_internal;                  /* '<S18>/MATLAB Function1' */
  boolean_T prev_ref_end_not_empty;    /* '<S18>/MATLAB Function1' */
} DW_MATLABFunction1_Ball_and_P_T;

/* Block signals for system '<S23>/MATLAB Function' */
typedef struct {
  real_T y;                            /* '<S23>/MATLAB Function' */
} B_MATLABFunction_Ball_and_Pla_T;

/* Block states (default storage) for system '<S23>/MATLAB Function' */
typedef struct {
  real_T index;                        /* '<S23>/MATLAB Function' */
  real_T previous_enable;              /* '<S23>/MATLAB Function' */
} DW_MATLABFunction_Ball_and_Pl_T;

/* Block signals for system '<S31>/Enabled Subsystem' */
typedef struct {
  real_T OutportBufferForOut1;         /* '<S32>/Constant' */
} B_EnabledSubsystem_Ball_and_P_T;

/* Block states (default storage) for system '<S31>/Enabled Subsystem' */
typedef struct {
  boolean_T EnabledSubsystem_MODE;     /* '<S31>/Enabled Subsystem' */
} DW_EnabledSubsystem_Ball_and__T;

/* Block signals for system '<S31>/Enabled Subsystem1' */
typedef struct {
  real_T In1;                          /* '<S33>/In1' */
} B_EnabledSubsystem1_Ball_and__T;

/* Block states (default storage) for system '<S31>/Enabled Subsystem1' */
typedef struct {
  boolean_T EnabledSubsystem1_MODE;    /* '<S31>/Enabled Subsystem1' */
} DW_EnabledSubsystem1_Ball_and_T;

/* Block signals (default storage) */
typedef struct {
  real_T RateTransition3;              /* '<Root>/Rate Transition3' */
  real_T Uk1;                          /* '<S1>/UD' */
  real_T Diff;                         /* '<S1>/Diff' */
  real_T Gain;                         /* '<Root>/Gain' */
  real_T FirstOrderHold;               /* '<Root>/First Order Hold' */
  real_T FirstOrderHold1;              /* '<Root>/First Order Hold1' */
  real_T Delay[2];                     /* '<Root>/Delay' */
  real_T Switch[4];                    /* '<Root>/Switch' */
  real_T Gain7;                        /* '<Root>/Gain7' */
  real_T Sum[4];                       /* '<S10>/Sum' */
  real_T RateLimiter[2];               /* '<S10>/Rate Limiter' */
  real_T moorxConversion[4];           /* '<S63>/mo or x Conversion' */
  real_T last_mv[2];                   /* '<S63>/last_mv' */
  real_T DataTypeConversion1[4];       /* '<S63>/Data Type Conversion1' */
  real_T DataTypeConversion2;          /* '<S63>/Data Type Conversion2' */
  real_T DataTypeConversion4[2];       /* '<S63>/Data Type Conversion4' */
  real_T umin_scale[2];                /* '<S63>/umin_scale' */
  real_T DataTypeConversion5[2];       /* '<S63>/Data Type Conversion5' */
  real_T umax_scale[2];                /* '<S63>/umax_scale' */
  real_T DataTypeConversion6[4];       /* '<S63>/Data Type Conversion6' */
  real_T ymin_scale[4];                /* '<S63>/ymin_scale' */
  real_T DataTypeConversion7[4];       /* '<S63>/Data Type Conversion7' */
  real_T ymax_scale[4];                /* '<S63>/ymax_scale' */
  real_T EConversion[2];               /* '<S63>/E Conversion' */
  real_T umin_scale4[2];               /* '<S63>/umin_scale4' */
  real_T Reshape[2];                   /* '<S63>/Reshape' */
  real_T FConversion[4];               /* '<S63>/F Conversion' */
  real_T ymin_scale1[4];               /* '<S63>/ymin_scale1' */
  real_T Reshape1[4];                  /* '<S63>/Reshape1' */
  real_T GConversion;                  /* '<S63>/G Conversion' */
  real_T SConversion;                  /* '<S63>/S Conversion' */
  real_T ymin_scale2;                  /* '<S63>/ymin_scale2' */
  real_T Reshape2;                     /* '<S63>/Reshape2' */
  real_T DataTypeConversion8;          /* '<S63>/Data Type Conversion8' */
  real_T DataTypeConversion3[2];       /* '<S63>/Data Type Conversion3' */
  real_T extmv_scale[2];               /* '<S63>/ext.mv_scale' */
  real_T DataTypeConversion13[2];      /* '<S63>/Data Type Conversion13' */
  real_T extmv_scale1[2];              /* '<S63>/ext.mv_scale1' */
  real_T DataTypeConversion9[4];       /* '<S63>/Data Type Conversion9' */
  real_T MathFunction[4];              /* '<S63>/Math Function' */
  real_T Reshape3[4];                  /* '<S63>/Reshape3' */
  real_T DataTypeConversion10[2];      /* '<S63>/Data Type Conversion10' */
  real_T MathFunction1[2];             /* '<S63>/Math Function1' */
  real_T Reshape4[2];                  /* '<S63>/Reshape4' */
  real_T DataTypeConversion12[2];      /* '<S63>/Data Type Conversion12' */
  real_T MathFunction2[2];             /* '<S63>/Math Function2' */
  real_T Reshape5[2];                  /* '<S63>/Reshape5' */
  real_T DataTypeConversion11;         /* '<S63>/Data Type Conversion11' */
  real_T umin_scale1[2];               /* '<S63>/umin_scale1' */
  real_T Add3[4];                      /* '<Root>/Add3' */
  real_T ProportionalGain;             /* '<S119>/Proportional Gain' */
  real_T Integrator;                   /* '<S114>/Integrator' */
  real_T DerivativeGain;               /* '<S108>/Derivative Gain' */
  real_T Filter;                       /* '<S109>/Filter' */
  real_T SumD;                         /* '<S109>/SumD' */
  real_T FilterCoefficient;            /* '<S117>/Filter Coefficient' */
  real_T ProportionalGain_b;           /* '<S163>/Proportional Gain' */
  real_T Integrator_c;                 /* '<S158>/Integrator' */
  real_T DerivativeGain_i;             /* '<S152>/Derivative Gain' */
  real_T Filter_c;                     /* '<S153>/Filter' */
  real_T SumD_c;                       /* '<S153>/SumD' */
  real_T FilterCoefficient_g;          /* '<S161>/Filter Coefficient' */
  real_T MultiportSwitch[2];           /* '<Root>/Multiport Switch' */
  real_T Switch1;                      /* '<Root>/Switch1' */
  real_T Add1;                         /* '<Root>/Add1' */
  real_T Switch2;                      /* '<Root>/Switch2' */
  real_T Add2;                         /* '<Root>/Add2' */
  real_T Gain4;                        /* '<Root>/Gain4' */
  real_T Gain1;                        /* '<S11>/Gain1' */
  real_T Dct1lowpass2;                 /* '<S11>/Dct1lowpass2' */
  real_T Alpha_sat;                    /* '<Root>/Alpha_sat ' */
  real_T Gain5;                        /* '<Root>/Gain5' */
  real_T Gain1_d;                      /* '<S12>/Gain1' */
  real_T Dct1lowpass2_g;               /* '<S12>/Dct1lowpass2' */
  real_T Beta_sat;                     /* '<Root>/Beta_sat ' */
  real_T CastToDouble;                 /* '<Root>/Cast To Double' */
  real_T CastToDouble1;                /* '<Root>/Cast To Double1' */
  real_T SFunction1[2];                /* '<S16>/S-Function1' */
  real_T Switch_h;                     /* '<S18>/Switch' */
  real_T enable_ref;                   /* '<S18>/enable_ref ' */
  real_T Sum1;                         /* '<S18>/Sum1' */
  real_T Gain2;                        /* '<S26>/Gain2' */
  real_T Dctleadlag1;                  /* '<S26>/Dctleadlag1' */
  real_T Dct1lowpass1;                 /* '<S26>/Dct1lowpass1' */
  real_T DiscreteTimeIntegrator;       /* '<S26>/Discrete-Time Integrator' */
  real_T Switch2_k;                    /* '<S18>/Switch2' */
  real_T Sum_p;                        /* '<S18>/Sum' */
  real_T Switch3;                      /* '<S18>/Switch3' */
  real_T Switch1_j;                    /* '<S19>/Switch1' */
  real_T Switch_j;                     /* '<S19>/Switch' */
  real_T Sum1_f;                       /* '<S19>/Sum1' */
  real_T Gain2_o;                      /* '<S39>/Gain2' */
  real_T Dctleadlag1_d;                /* '<S39>/Dctleadlag1' */
  real_T Dct1lowpass1_c;               /* '<S39>/Dct1lowpass1' */
  real_T DiscreteTimeIntegrator_m;     /* '<S39>/Discrete-Time Integrator' */
  real_T Switch2_p;                    /* '<S19>/Switch2' */
  real_T Sum_g;                        /* '<S19>/Sum' */
  real_T Switch3_d;                    /* '<S19>/Switch3' */
  real_T Switch_n;                     /* '<S34>/Switch' */
  real_T Constant1;                    /* '<S34>/Constant1' */
  real_T Constant2;                    /* '<S34>/Constant2' */
  real_T Switch1_k;                    /* '<S20>/Switch1' */
  real_T Switch_jo;                    /* '<S20>/Switch' */
  real_T Sum1_h;                       /* '<S20>/Sum1' */
  real_T Gain1_a;                      /* '<S54>/Gain1' */
  real_T Dctleadlag2;                  /* '<S54>/Dctleadlag2' */
  real_T Dct1lowpass2_p;               /* '<S54>/Dct1lowpass2' */
  real_T DiscreteTimeIntegrator_h;     /* '<S54>/Discrete-Time Integrator' */
  real_T Switch2_h;                    /* '<S20>/Switch2' */
  real_T Sum_h;                        /* '<S20>/Sum' */
  real_T Switch3_l;                    /* '<S20>/Switch3' */
  real_T reser_integrator;             /* '<S3>/reser_integrator' */
  real_T DataTypeConversion14;         /* '<S63>/Data Type Conversion14' */
  real_T DataTypeConversion15;         /* '<S63>/Data Type Conversion15' */
  real_T last_x[4];                    /* '<S63>/last_x' */
  real_T umin_scale3[52];              /* '<S63>/umin_scale3' */
  real_T umin_scale5[104];             /* '<S63>/umin_scale5' */
  real_T umin_scale2[2];               /* '<S63>/umin_scale2' */
  real_T IntegralGain;                 /* '<S111>/Integral Gain' */
  real_T IntegralGain_f;               /* '<S155>/Integral Gain' */
  real_T Gain1_k;                      /* '<S13>/Gain1' */
  real_T Dct1lowpass2_k;               /* '<S13>/Dct1lowpass2' */
  real_T u_cmd[2];                     /* '<S10>/MATLAB Function1' */
  real_T u_raw[2];                     /* '<S10>/MATLAB Function1' */
  real_T u_box[2];                     /* '<S10>/MATLAB Function1' */
  real_T xK_norm;                      /* '<S10>/MATLAB Function1' */
  real_T u_total[2];                   /* '<S10>/MATLAB Function' */
  real_T xk1[4];                       /* '<S83>/optimizer' */
  real_T u[2];                         /* '<S83>/optimizer' */
  real_T cost;                         /* '<S83>/optimizer' */
  real_T useq[52];                     /* '<S83>/optimizer' */
  real_T xseq[104];                    /* '<S83>/optimizer' */
  real_T yseq[104];                    /* '<S83>/optimizer' */
  real_T status;                       /* '<S83>/optimizer' */
  real_T xest[4];                      /* '<S83>/optimizer' */
  real_T r[4];                         /* '<Root>/MATLAB Function3' */
  real_T a[2];                         /* '<Root>/MATLAB Function3' */
  real_T TmpSignalConversionAtSFunctionI[2];/* '<Root>/MATLAB Function' */
  real_T x_est[4];                     /* '<Root>/MATLAB Function' */
  real_T alpha;                        /* '<S3>/PosToAngle ' */
  real_T beta;                         /* '<S3>/PosToAngle ' */
  real_T psi;                          /* '<S3>/PosToAngle ' */
  real_T SFunction1_o1;                /* '<S58>/S-Function1' */
  real_T SFunction1_o2;                /* '<S58>/S-Function1' */
  real_T Inc2Pos;                      /* '<S53>/Inc2Pos' */
  real_T AbsPosition;                  /* '<S53>/AbsPosition' */
  real_T Abs;                          /* '<S60>/Abs' */
  real_T TransferFcn;                  /* '<S60>/Transfer Fcn' */
  real_T mm2m;                         /* '<S53>/mm2m' */
  real_T Saturation_i;                 /* '<S52>/Saturation' */
  real_T Current2V;                    /* '<S52>/Current2V' */
  real_T DSPscale;                     /* '<S52>/DSPscale' */
  real_T Add3_n;                       /* '<S3>/Add3' */
  real_T Add;                          /* '<S54>/Add' */
  real_T SFunction1_o1_i;              /* '<S44>/S-Function1' */
  real_T SFunction1_o2_p;              /* '<S44>/S-Function1' */
  real_T Inc2Pos_e;                    /* '<S38>/Inc2Pos' */
  real_T AbsPosition_i;                /* '<S38>/AbsPosition' */
  real_T Abs_j;                        /* '<S46>/Abs' */
  real_T TransferFcn_o;                /* '<S46>/Transfer Fcn' */
  real_T mm2m_p;                       /* '<S38>/mm2m' */
  real_T Saturation_l;                 /* '<S37>/Saturation' */
  real_T Current2V_f;                  /* '<S37>/Current2V' */
  real_T DSPscale_o;                   /* '<S37>/DSPscale' */
  real_T path;                         /* '<S34>/MATLAB Function2' */
  real_T y;                            /* '<S34>/MATLAB Function' */
  real_T Add2_h;                       /* '<S3>/Add2' */
  real_T Add_f;                        /* '<S39>/Add' */
  real_T SFunction1_o1_p;              /* '<S29>/S-Function1' */
  real_T SFunction1_o2_d;              /* '<S29>/S-Function1' */
  real_T Inc2Pos_p;                    /* '<S25>/Inc2Pos' */
  real_T AbsPosition_m;                /* '<S25>/AbsPosition' */
  real_T Abs_h;                        /* '<S31>/Abs' */
  real_T TransferFcn_f;                /* '<S31>/Transfer Fcn' */
  real_T mm2m_po;                      /* '<S25>/mm2m' */
  real_T Saturation_a;                 /* '<S24>/Saturation' */
  real_T Current2V_h;                  /* '<S24>/Current2V' */
  real_T DSPscale_m;                   /* '<S24>/DSPscale' */
  real_T Add_h;                        /* '<S26>/Add' */
  real_T Add1_m;                       /* '<S3>/Add1' */
  real_T pos1;                         /* '<S3>/AngleToPos ' */
  real_T pos2;                         /* '<S3>/AngleToPos ' */
  real_T pos3;                         /* '<S3>/AngleToPos ' */
  real_T Add_n[4];                     /* '<Root>/Add' */
  real_T Gain1_j[2];                   /* '<Root>/Gain1' */
  real_T Sum_n;                        /* '<S123>/Sum' */
  real_T Sum_f;                        /* '<S167>/Sum' */
  real_T Gain_ff_x;                    /* '<Root>/Gain_ff_x' */
  real_T Gain_ff_y;                    /* '<Root>/Gain_ff_y ' */
  uint32_T SFunction1_o4;              /* '<S15>/S-Function1' */
  uint32_T SFunction1_o1_m[3];         /* '<S14>/S-Function1' */
  real32_T ByteUnpacking_o1;           /* '<S2>/Byte Unpacking ' */
  real32_T ByteUnpacking_o2;           /* '<S2>/Byte Unpacking ' */
  real32_T ByteUnpacking_o3;           /* '<S2>/Byte Unpacking ' */
  real32_T Gain2_j;                    /* '<Root>/Gain2' */
  real32_T RateTransition4;            /* '<Root>/Rate Transition4' */
  real32_T Gain3;                      /* '<Root>/Gain3' */
  real32_T RateTransition5;            /* '<Root>/Rate Transition5' */
  uint16_T SFunction1_o6;              /* '<S15>/S-Function1' */
  uint8_T SFunction1_o1_c[32];         /* '<S15>/S-Function1' */
  uint8_T SFunction1_o5[4];            /* '<S15>/S-Function1' */
  uint8_T SFunction1_o2_f[4];          /* '<S14>/S-Function1' */
  int8_T ByteUnpacking_o4;             /* '<S2>/Byte Unpacking ' */
  int8_T ByteUnpacking_o5[19];         /* '<S2>/Byte Unpacking ' */
  boolean_T DataTypeConversion;        /* '<S2>/Data Type Conversion' */
  boolean_T Compare;                   /* '<S179>/Compare' */
  boolean_T Uk1_j;                     /* '<S176>/Delay Input1' */
  boolean_T FixPtRelationalOperator;   /* '<S176>/FixPt Relational Operator' */
  boolean_T Memory[212];               /* '<S63>/Memory' */
  boolean_T sat_flag;                  /* '<S10>/MATLAB Function1' */
  boolean_T iAout[212];                /* '<S83>/optimizer' */
  boolean_T RelationalOperator1;       /* '<S60>/Relational Operator1' */
  boolean_T RelationalOperator;        /* '<S60>/Relational Operator' */
  boolean_T LogicalOperator2;          /* '<S60>/Logical Operator2' */
  boolean_T RelationalOperator1_c;     /* '<S46>/Relational Operator1' */
  boolean_T RelationalOperator_n;      /* '<S46>/Relational Operator' */
  boolean_T LogicalOperator2_l;        /* '<S46>/Logical Operator2' */
  boolean_T RelationalOperator1_f;     /* '<S31>/Relational Operator1' */
  boolean_T RelationalOperator_a;      /* '<S31>/Relational Operator' */
  boolean_T LogicalOperator2_a;        /* '<S31>/Logical Operator2' */
  B_EnabledSubsystem1_Ball_and__T EnabledSubsystem1_b;/* '<S60>/Enabled Subsystem1' */
  B_EnabledSubsystem_Ball_and_P_T EnabledSubsystem_n;/* '<S60>/Enabled Subsystem' */
  B_MATLABFunction_Ball_and_Pla_T sf_MATLABFunction_h;/* '<S51>/MATLAB Function' */
  B_MATLABFunction_Ball_and_Pla_T sf_MATLABFunction_g;/* '<S50>/MATLAB Function' */
  B_MATLABFunction1_Ball_and_Pl_T sf_MATLABFunction1_h;/* '<S20>/MATLAB Function1' */
  B_EnabledSubsystem1_Ball_and__T EnabledSubsystem1_f;/* '<S46>/Enabled Subsystem1' */
  B_EnabledSubsystem_Ball_and_P_T EnabledSubsystem_a;/* '<S46>/Enabled Subsystem' */
  B_MATLABFunction_Ball_and_Pla_T sf_MATLABFunction_ko;/* '<S36>/MATLAB Function' */
  B_MATLABFunction1_Ball_and_Pl_T sf_MATLABFunction2_d;/* '<S19>/MATLAB Function2' */
  B_EnabledSubsystem1_Ball_and__T EnabledSubsystem1;/* '<S31>/Enabled Subsystem1' */
  B_EnabledSubsystem_Ball_and_P_T EnabledSubsystem;/* '<S31>/Enabled Subsystem' */
  B_MATLABFunction_Ball_and_Pla_T sf_MATLABFunction;/* '<S23>/MATLAB Function' */
  B_MATLABFunction1_Ball_and_Pl_T sf_MATLABFunction1;/* '<S18>/MATLAB Function1' */
} B_Ball_and_Plate_MicroLabBox_student_T;

/* Block states (default storage) for system '<Root>' */
typedef struct {
  real_T UD_DSTATE;                    /* '<S1>/UD' */
  real_T Delay_DSTATE[2];              /* '<Root>/Delay' */
  real_T last_mv_DSTATE[2];            /* '<S63>/last_mv' */
  real_T Integrator_DSTATE;            /* '<S114>/Integrator' */
  real_T Filter_DSTATE;                /* '<S109>/Filter' */
  real_T Integrator_DSTATE_a;          /* '<S158>/Integrator' */
  real_T Filter_DSTATE_e;              /* '<S153>/Filter' */
  real_T DiscreteTimeIntegrator_DSTATE;/* '<S26>/Discrete-Time Integrator' */
  real_T DiscreteTimeIntegrator_DSTATE_p;/* '<S39>/Discrete-Time Integrator' */
  real_T DiscreteTimeIntegrator_DSTATE_o;/* '<S54>/Discrete-Time Integrator' */
  volatile real_T RateTransition3_Buffer0;/* '<Root>/Rate Transition3' */
  real_T Tk;                           /* '<Root>/First Order Hold' */
  real_T Ck;                           /* '<Root>/First Order Hold' */
  real_T Mk;                           /* '<Root>/First Order Hold' */
  real_T Uk;                           /* '<Root>/First Order Hold' */
  real_T Tk_c;                         /* '<Root>/First Order Hold1' */
  real_T Ck_h;                         /* '<Root>/First Order Hold1' */
  real_T Mk_g;                         /* '<Root>/First Order Hold1' */
  real_T Uk_i;                         /* '<Root>/First Order Hold1' */
  real_T PrevY[2];                     /* '<S10>/Rate Limiter' */
  real_T last_x_PreviousInput[4];      /* '<S63>/last_x' */
  real_T xK[16];                       /* '<S10>/MATLAB Function1' */
  real_T u_prev[2];                    /* '<S10>/MATLAB Function1' */
  real_T alpha_I;                      /* '<S10>/MATLAB Function' */
  real_T beta_I;                       /* '<S10>/MATLAB Function' */
  real_T theta;                        /* '<Root>/MATLAB Function3' */
  real_T current_R;                    /* '<Root>/MATLAB Function3' */
  real_T x_hat[4];                     /* '<Root>/MATLAB Function' */
  real_T P[16];                        /* '<Root>/MATLAB Function' */
  real_T index;                        /* '<S34>/MATLAB Function' */
  real_T previous_enable;              /* '<S34>/MATLAB Function' */
  struct {
    real_T RECEIVED_FRAMES;
  } SFunction1_RWORK;                  /* '<S15>/S-Function1' */

  real_T Dct1lowpass2_RWORK[2];        /* '<S11>/Dct1lowpass2' */
  real_T Dct1lowpass2_RWORK_g[2];      /* '<S12>/Dct1lowpass2' */
  struct {
    real_T RX_DROPPED_FRAMES[2];
  } SFunction1_RWORK_l;                /* '<S16>/S-Function1' */

  real_T Dctleadlag1_RWORK[2];         /* '<S26>/Dctleadlag1' */
  real_T Dct1lowpass1_RWORK[2];        /* '<S26>/Dct1lowpass1' */
  real_T Dctleadlag1_RWORK_l[2];       /* '<S39>/Dctleadlag1' */
  real_T Dct1lowpass1_RWORK_p[2];      /* '<S39>/Dct1lowpass1' */
  real_T Dctleadlag2_RWORK[2];         /* '<S54>/Dctleadlag2' */
  real_T Dct1lowpass2_RWORK_f[2];      /* '<S54>/Dct1lowpass2' */
  real_T Dct1lowpass2_RWORK_j[2];      /* '<S13>/Dct1lowpass2' */
  int_T ByteUnpacking_IWORK[10];       /* '<S2>/Byte Unpacking ' */
  boolean_T DelayInput1_DSTATE;        /* '<S176>/Delay Input1' */
  volatile int8_T RateTransition3_semaphoreTaken;/* '<Root>/Rate Transition3' */
  int8_T Integrator_PrevResetState;    /* '<S114>/Integrator' */
  int8_T Filter_PrevResetState;        /* '<S109>/Filter' */
  int8_T Integrator_PrevResetState_d;  /* '<S158>/Integrator' */
  int8_T Filter_PrevResetState_k;      /* '<S153>/Filter' */
  int8_T DiscreteTimeIntegrator_PrevRese;/* '<S26>/Discrete-Time Integrator' */
  int8_T DiscreteTimeIntegrator_PrevRe_h;/* '<S39>/Discrete-Time Integrator' */
  int8_T DiscreteTimeIntegrator_PrevRe_e;/* '<S54>/Discrete-Time Integrator' */
  boolean_T Memory_PreviousInput[212]; /* '<S63>/Memory' */
  boolean_T initialized_not_empty;     /* '<S10>/MATLAB Function' */
  DW_EnabledSubsystem1_Ball_and_T EnabledSubsystem1_b;/* '<S60>/Enabled Subsystem1' */
  DW_EnabledSubsystem_Ball_and__T EnabledSubsystem_n;/* '<S60>/Enabled Subsystem' */
  DW_MATLABFunction_Ball_and_Pl_T sf_MATLABFunction_h;/* '<S51>/MATLAB Function' */
  DW_MATLABFunction_Ball_and_Pl_T sf_MATLABFunction_g;/* '<S50>/MATLAB Function' */
  DW_MATLABFunction1_Ball_and_P_T sf_MATLABFunction1_h;/* '<S20>/MATLAB Function1' */
  DW_EnabledSubsystem1_Ball_and_T EnabledSubsystem1_f;/* '<S46>/Enabled Subsystem1' */
  DW_EnabledSubsystem_Ball_and__T EnabledSubsystem_a;/* '<S46>/Enabled Subsystem' */
  DW_MATLABFunction_Ball_and_Pl_T sf_MATLABFunction_ko;/* '<S36>/MATLAB Function' */
  DW_MATLABFunction1_Ball_and_P_T sf_MATLABFunction2_d;/* '<S19>/MATLAB Function2' */
  DW_EnabledSubsystem1_Ball_and_T EnabledSubsystem1;/* '<S31>/Enabled Subsystem1' */
  DW_EnabledSubsystem_Ball_and__T EnabledSubsystem;/* '<S31>/Enabled Subsystem' */
  DW_MATLABFunction_Ball_and_Pl_T sf_MATLABFunction;/* '<S23>/MATLAB Function' */
  DW_MATLABFunction1_Ball_and_P_T sf_MATLABFunction1;/* '<S18>/MATLAB Function1' */
} DW_Ball_and_Plate_MicroLabBox_student_T;

/* Continuous states (default storage) */
typedef struct {
  real_T TransferFcn_CSTATE;           /* '<S60>/Transfer Fcn' */
  real_T TransferFcn_CSTATE_h;         /* '<S46>/Transfer Fcn' */
  real_T TransferFcn_CSTATE_l;         /* '<S31>/Transfer Fcn' */
} X_Ball_and_Plate_MicroLabBox_student_T;

/* State derivatives (default storage) */
typedef struct {
  real_T TransferFcn_CSTATE;           /* '<S60>/Transfer Fcn' */
  real_T TransferFcn_CSTATE_h;         /* '<S46>/Transfer Fcn' */
  real_T TransferFcn_CSTATE_l;         /* '<S31>/Transfer Fcn' */
} XDot_Ball_and_Plate_MicroLabBox_student_T;

/* State disabled  */
typedef struct {
  boolean_T TransferFcn_CSTATE;        /* '<S60>/Transfer Fcn' */
  boolean_T TransferFcn_CSTATE_h;      /* '<S46>/Transfer Fcn' */
  boolean_T TransferFcn_CSTATE_l;      /* '<S31>/Transfer Fcn' */
} XDis_Ball_and_Plate_MicroLabBox_student_T;

/* Zero-crossing (trigger) state */
typedef struct {
  ZCSigState EMC_ENCODER_POS_SET_BL1_Trig_ZC;/* '<S53>/EMC_ENCODER_POS_SET_BL1' */
  ZCSigState EMC_ENCODER_POS_SET_BL1_Trig__f;/* '<S38>/EMC_ENCODER_POS_SET_BL1' */
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

/* Parameters for system: '<S31>/Enabled Subsystem' */
struct P_EnabledSubsystem_Ball_and_P_T_ {
  real_T Out1_Y0;                      /* Expression: [0]
                                        * Referenced by: '<S32>/Out1'
                                        */
  real_T Constant_Value;               /* Expression: -1
                                        * Referenced by: '<S32>/Constant'
                                        */
};

/* Parameters for system: '<S31>/Enabled Subsystem1' */
struct P_EnabledSubsystem1_Ball_and__T_ {
  real_T Out1_Y0;                      /* Expression: [0]
                                        * Referenced by: '<S33>/Out1'
                                        */
};

/* Parameters (default storage) */
struct P_Ball_and_Plate_MicroLabBox_student_T_ {
  real_T A_robust[256];                /* Variable: A_robust
                                        * Referenced by: '<S10>/Constant'
                                        */
  real_T B_robust[64];                 /* Variable: B_robust
                                        * Referenced by: '<S10>/Constant1'
                                        */
  real_T Baw_robust[32];               /* Variable: Baw_robust
                                        * Referenced by: '<S10>/Constant4'
                                        */
  real_T C_robust[32];                 /* Variable: C_robust
                                        * Referenced by: '<S10>/Constant2'
                                        */
  real_T D_robust[8];                  /* Variable: D_robust
                                        * Referenced by: '<S10>/Constant3'
                                        */
  real_T K_lqr[8];                     /* Variable: K_lqr
                                        * Referenced by: '<Root>/Gain1'
                                        */
  real_T Kaw_robust;                   /* Variable: Kaw_robust
                                        * Referenced by: '<S10>/Constant5'
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
                                        * Referenced by: '<Root>/Gain'
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
  real_T inner_I_sat;                  /* Variable: inner_I_sat
                                        * Referenced by:
                                        *   '<S26>/Discrete-Time Integrator'
                                        *   '<S39>/Discrete-Time Integrator'
                                        *   '<S54>/Discrete-Time Integrator'
                                        */
  real_T path[3001];                   /* Variable: path
                                        * Referenced by: '<S34>/Constant4'
                                        */
  real_T plate_angle_sat;              /* Variable: plate_angle_sat
                                        * Referenced by:
                                        *   '<Root>/Alpha_sat '
                                        *   '<Root>/Beta_sat '
                                        */
  real_T uA[200000];                   /* Variable: uA
                                        * Referenced by:
                                        *   '<S23>/Constant'
                                        *   '<S50>/Constant'
                                        */
  real_T uB[200000];                   /* Variable: uB
                                        * Referenced by: '<S36>/Constant'
                                        */
  real_T uC[200000];                   /* Variable: uC
                                        * Referenced by: '<S51>/Constant'
                                        */
  real_T u_max_robust[2];              /* Variable: u_max_robust
                                        * Referenced by: '<S10>/Constant7'
                                        */
  real_T u_min_robust[2];              /* Variable: u_min_robust
                                        * Referenced by: '<S10>/Constant6'
                                        */
  real_T PIDController_D;              /* Mask Parameter: PIDController_D
                                        * Referenced by: '<S108>/Derivative Gain'
                                        */
  real_T PIDController1_D;             /* Mask Parameter: PIDController1_D
                                        * Referenced by: '<S152>/Derivative Gain'
                                        */
  real_T PIDController_I;              /* Mask Parameter: PIDController_I
                                        * Referenced by: '<S111>/Integral Gain'
                                        */
  real_T PIDController1_I;             /* Mask Parameter: PIDController1_I
                                        * Referenced by: '<S155>/Integral Gain'
                                        */
  real_T Difference_ICPrevInput;       /* Mask Parameter: Difference_ICPrevInput
                                        * Referenced by: '<S1>/UD'
                                        */
  real_T PIDController_InitialConditionF;
                              /* Mask Parameter: PIDController_InitialConditionF
                               * Referenced by: '<S109>/Filter'
                               */
  real_T PIDController1_InitialCondition;
                              /* Mask Parameter: PIDController1_InitialCondition
                               * Referenced by: '<S153>/Filter'
                               */
  real_T PIDController_InitialConditio_l;
                              /* Mask Parameter: PIDController_InitialConditio_l
                               * Referenced by: '<S114>/Integrator'
                               */
  real_T PIDController1_InitialConditi_b;
                              /* Mask Parameter: PIDController1_InitialConditi_b
                               * Referenced by: '<S158>/Integrator'
                               */
  real_T PIDController_N;              /* Mask Parameter: PIDController_N
                                        * Referenced by: '<S117>/Filter Coefficient'
                                        */
  real_T PIDController1_N;             /* Mask Parameter: PIDController1_N
                                        * Referenced by: '<S161>/Filter Coefficient'
                                        */
  real_T PIDController_P;              /* Mask Parameter: PIDController_P
                                        * Referenced by: '<S119>/Proportional Gain'
                                        */
  real_T PIDController1_P;             /* Mask Parameter: PIDController1_P
                                        * Referenced by: '<S163>/Proportional Gain'
                                        */
  boolean_T DetectRisePositive_vinit;/* Mask Parameter: DetectRisePositive_vinit
                                      * Referenced by: '<S176>/Delay Input1'
                                      */
  real_T Constant8_Value;              /* Expression: 0
                                        * Referenced by: '<Root>/Constant8'
                                        */
  real_T Gain_ff_y_Gain;               /* Expression: 1
                                        * Referenced by: '<Root>/Gain_ff_y '
                                        */
  real_T Constant7_Value;              /* Expression: 0
                                        * Referenced by: '<Root>/Constant7'
                                        */
  real_T Gain_ff_x_Gain;               /* Expression: 1
                                        * Referenced by: '<Root>/Gain_ff_x'
                                        */
  real_T Constant3_Value[4];           /* Expression: [0;0;0;0]
                                        * Referenced by: '<Root>/Constant3'
                                        */
  real_T Constant_Value;               /* Expression: 0.32
                                        * Referenced by: '<S3>/Constant'
                                        */
  real_T Constant_Value_h;             /* Expression: 0
                                        * Referenced by: '<S18>/Constant'
                                        */
  real_T Constant3_Value_d;            /* Expression: -0.0289
                                        * Referenced by: '<S18>/Constant3'
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
                                        * Referenced by: '<S31>/Const'
                                        */
  real_T Const1_Value;                 /* Expression: -0.98
                                        * Referenced by: '<S31>/Const1'
                                        */
  real_T TransferFcn_A;                /* Computed Parameter: TransferFcn_A
                                        * Referenced by: '<S31>/Transfer Fcn'
                                        */
  real_T TransferFcn_C;                /* Computed Parameter: TransferFcn_C
                                        * Referenced by: '<S31>/Transfer Fcn'
                                        */
  real_T mm2m_Gain;                    /* Expression: 0.001
                                        * Referenced by: '<S25>/mm2m'
                                        */
  real_T Constant1_Value;              /* Expression: 0
                                        * Referenced by: '<S19>/Constant1'
                                        */
  real_T Constant5_Value;              /* Expression: -0.0289
                                        * Referenced by: '<S19>/Constant5'
                                        */
  real_T Constant1_Value_n;            /* Expression: 0.32
                                        * Referenced by: '<S3>/Constant1'
                                        */
  real_T Constant3_Value_i;            /* Expression: 0
                                        * Referenced by: '<S34>/Constant3'
                                        */
  real_T Saturation_UpperSat_e;        /* Expression: 3
                                        * Referenced by: '<S37>/Saturation'
                                        */
  real_T Saturation_LowerSat_f;        /* Expression: -3
                                        * Referenced by: '<S37>/Saturation'
                                        */
  real_T Current2V_Gain_f;             /* Expression: 10/3
                                        * Referenced by: '<S37>/Current2V'
                                        */
  real_T DSPscale_Gain_d;              /* Expression: 1/10
                                        * Referenced by: '<S37>/DSPscale'
                                        */
  real_T Constant_Value_j0;            /* Expression: 10
                                        * Referenced by: '<S38>/Constant'
                                        */
  real_T ZP_Value_m;                   /* Expression: 29
                                        * Referenced by: '<S38>/ZP'
                                        */
  real_T Inc2Pos_Gain_k;               /* Expression: 8e-3
                                        * Referenced by: '<S38>/Inc2Pos'
                                        */
  real_T Pos_offset_Value_k;           /* Expression: 0
                                        * Referenced by: '<S38>/Pos_offset'
                                        */
  real_T Const_Value_g;                /* Expression: 0.01
                                        * Referenced by: '<S46>/Const'
                                        */
  real_T Const1_Value_a;               /* Expression: -0.98
                                        * Referenced by: '<S46>/Const1'
                                        */
  real_T TransferFcn_A_g;              /* Computed Parameter: TransferFcn_A_g
                                        * Referenced by: '<S46>/Transfer Fcn'
                                        */
  real_T TransferFcn_C_e;              /* Computed Parameter: TransferFcn_C_e
                                        * Referenced by: '<S46>/Transfer Fcn'
                                        */
  real_T mm2m_Gain_f;                  /* Expression: 0.001
                                        * Referenced by: '<S38>/mm2m'
                                        */
  real_T Constant_Value_e;             /* Expression: 0
                                        * Referenced by: '<S20>/Constant'
                                        */
  real_T Constant3_Value_n;            /* Expression: -0.0289
                                        * Referenced by: '<S20>/Constant3'
                                        */
  real_T Constant2_Value;              /* Expression: 0.32
                                        * Referenced by: '<S3>/Constant2'
                                        */
  real_T Saturation_UpperSat_h;        /* Expression: 3
                                        * Referenced by: '<S52>/Saturation'
                                        */
  real_T Saturation_LowerSat_p;        /* Expression: -3
                                        * Referenced by: '<S52>/Saturation'
                                        */
  real_T Current2V_Gain_p;             /* Expression: 10/3
                                        * Referenced by: '<S52>/Current2V'
                                        */
  real_T DSPscale_Gain_h;              /* Expression: 1/10
                                        * Referenced by: '<S52>/DSPscale'
                                        */
  real_T Constant_Value_f;             /* Expression: 10
                                        * Referenced by: '<S53>/Constant'
                                        */
  real_T ZP_Value_p;                   /* Expression: 29
                                        * Referenced by: '<S53>/ZP'
                                        */
  real_T Inc2Pos_Gain_h;               /* Expression: 8e-3
                                        * Referenced by: '<S53>/Inc2Pos'
                                        */
  real_T Pos_offset_Value_f;           /* Expression: 0
                                        * Referenced by: '<S53>/Pos_offset'
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
                                        * Referenced by: '<S53>/mm2m'
                                        */
  real_T Constant_Value_hi;            /* Expression: 0
                                        * Referenced by: '<S179>/Constant'
                                        */
  real_T Constant_Value_i;             /* Expression: 1
                                        * Referenced by: '<S2>/Constant'
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
  real_T Delay_InitialCondition;       /* Expression: 0.0
                                        * Referenced by: '<Root>/Delay'
                                        */
  real_T enable_outer_controller_Value;/* Expression: 0
                                        * Referenced by: '<Root>/enable_outer_controller'
                                        */
  real_T Switch_Threshold;             /* Expression: 0
                                        * Referenced by: '<Root>/Switch'
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
  real_T Controller_select_Value;      /* Expression: 2
                                        * Referenced by: '<Root>/Controller_select'
                                        */
  real_T RateLimiter_RisingLim;     /* Computed Parameter: RateLimiter_RisingLim
                                     * Referenced by: '<S10>/Rate Limiter'
                                     */
  real_T RateLimiter_FallingLim;   /* Computed Parameter: RateLimiter_FallingLim
                                    * Referenced by: '<S10>/Rate Limiter'
                                    */
  real_T RateLimiter_IC;               /* Expression: 0
                                        * Referenced by: '<S10>/Rate Limiter'
                                        */
  real_T last_mv_InitialCondition[2];  /* Expression: lastu+uoff
                                        * Referenced by: '<S63>/last_mv'
                                        */
  real_T ym_zero_Value[4];             /* Expression: zeros(nym,1)
                                        * Referenced by: '<S63>/ym_zero'
                                        */
  real_T md_zero_Value;                /* Expression: zeros(1,1)
                                        * Referenced by: '<S6>/md_zero'
                                        */
  real_T umin_zero_Value[2];           /* Expression: zeros(2,1)
                                        * Referenced by: '<S6>/umin_zero'
                                        */
  real_T umin_scale_Gain[2];           /* Expression: RMVscale
                                        * Referenced by: '<S63>/umin_scale'
                                        */
  real_T umax_zero_Value[2];           /* Expression: zeros(2,1)
                                        * Referenced by: '<S6>/umax_zero'
                                        */
  real_T umax_scale_Gain[2];           /* Expression: RMVscale
                                        * Referenced by: '<S63>/umax_scale'
                                        */
  real_T ymin_zero_Value[4];           /* Expression: zeros(4,1)
                                        * Referenced by: '<S6>/ymin_zero'
                                        */
  real_T ymin_scale_Gain[4];           /* Expression: RYscale
                                        * Referenced by: '<S63>/ymin_scale'
                                        */
  real_T ymax_zero_Value[4];           /* Expression: zeros(4,1)
                                        * Referenced by: '<S6>/ymax_zero'
                                        */
  real_T ymax_scale_Gain[4];           /* Expression: RYscale
                                        * Referenced by: '<S63>/ymax_scale'
                                        */
  real_T E_zero_Value[2];              /* Expression: zeros(1,2)
                                        * Referenced by: '<S6>/E_zero'
                                        */
  real_T umin_scale4_Gain[2];      /* Expression: MVscale(:,ones(1,max(nCC,1)))'
                                    * Referenced by: '<S63>/umin_scale4'
                                    */
  real_T F_zero_Value[4];              /* Expression: zeros(1,4)
                                        * Referenced by: '<S6>/F_zero'
                                        */
  real_T ymin_scale1_Gain[4];       /* Expression: Yscale(:,ones(1,max(nCC,1)))'
                                     * Referenced by: '<S63>/ymin_scale1'
                                     */
  real_T G_zero_Value;                 /* Expression: zeros(1,1)
                                        * Referenced by: '<S6>/G_zero'
                                        */
  real_T S_zero_Value;                 /* Expression: zeros(1,1)
                                        * Referenced by: '<S6>/S_zero'
                                        */
  real_T ymin_scale2_Gain;         /* Expression: MDscale(:,ones(1,max(nCC,1)))'
                                    * Referenced by: '<S63>/ymin_scale2'
                                    */
  real_T switch_zero_Value;            /* Expression: zeros(1,1)
                                        * Referenced by: '<S6>/switch_zero'
                                        */
  real_T extmv_zero_Value[2];          /* Expression: zeros(2,1)
                                        * Referenced by: '<S6>/ext.mv_zero'
                                        */
  real_T extmv_scale_Gain[2];          /* Expression: RMVscale
                                        * Referenced by: '<S63>/ext.mv_scale'
                                        */
  real_T mvtarget_zero_Value[2];       /* Expression: zeros(2,1)
                                        * Referenced by: '<S6>/mv.target_zero'
                                        */
  real_T extmv_scale1_Gain[2];         /* Expression: RMVscale
                                        * Referenced by: '<S63>/ext.mv_scale1'
                                        */
  real_T ywt_zero_Value[4];            /* Expression: zeros(4,1)
                                        * Referenced by: '<S6>/y.wt_zero'
                                        */
  real_T uwt_zero_Value[2];            /* Expression: zeros(2,1)
                                        * Referenced by: '<S6>/u.wt_zero'
                                        */
  real_T duwt_zero_Value[2];           /* Expression: zeros(2,1)
                                        * Referenced by: '<S6>/du.wt_zero'
                                        */
  real_T ecrwt_zero_Value;             /* Expression: zeros(1,1)
                                        * Referenced by: '<S6>/ecr.wt_zero'
                                        */
  real_T umin_scale1_Gain[2];          /* Expression: MVscale
                                        * Referenced by: '<S63>/umin_scale1'
                                        */
  real_T Integrator_gainval;           /* Computed Parameter: Integrator_gainval
                                        * Referenced by: '<S114>/Integrator'
                                        */
  real_T Filter_gainval;               /* Computed Parameter: Filter_gainval
                                        * Referenced by: '<S109>/Filter'
                                        */
  real_T Integrator_gainval_j;       /* Computed Parameter: Integrator_gainval_j
                                      * Referenced by: '<S158>/Integrator'
                                      */
  real_T Filter_gainval_a;             /* Computed Parameter: Filter_gainval_a
                                        * Referenced by: '<S153>/Filter'
                                        */
  real_T ff_enable_Value;              /* Expression: 0
                                        * Referenced by: '<Root>/ff_enable'
                                        */
  real_T Switch1_Threshold;            /* Expression: 0
                                        * Referenced by: '<Root>/Switch1'
                                        */
  real_T Switch2_Threshold;            /* Expression: 0
                                        * Referenced by: '<Root>/Switch2'
                                        */
  real_T Gain4_Gain;                   /* Expression: -1
                                        * Referenced by: '<Root>/Gain4'
                                        */
  real_T Gain1_Gain;                   /* Expression: 1
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
  real_T Gain5_Gain;                   /* Expression: -1
                                        * Referenced by: '<Root>/Gain5'
                                        */
  real_T Gain1_Gain_b;                 /* Expression: 1
                                        * Referenced by: '<S12>/Gain1'
                                        */
  real_T Dct1lowpass2_P1_Size_p[2];/* Computed Parameter: Dct1lowpass2_P1_Size_p
                                    * Referenced by: '<S12>/Dct1lowpass2'
                                    */
  real_T Dct1lowpass2_P1_f;            /* Expression: f_den
                                        * Referenced by: '<S12>/Dct1lowpass2'
                                        */
  real_T Dct1lowpass2_P2_Size_c[2];/* Computed Parameter: Dct1lowpass2_P2_Size_c
                                    * Referenced by: '<S12>/Dct1lowpass2'
                                    */
  real_T Dct1lowpass2_P2_b;            /* Expression: 0.001
                                        * Referenced by: '<S12>/Dct1lowpass2'
                                        */
  real_T Psi_ref_Value;                /* Expression: 0
                                        * Referenced by: '<Root>/Psi_ref '
                                        */
  real_T Init_Value;                   /* Expression: 0
                                        * Referenced by: '<S3>/Init  '
                                        */
  real_T enable_outerloop_Value;       /* Expression: 0
                                        * Referenced by: '<S3>/enable_outerloop '
                                        */
  real_T quintic_ref_Value;            /* Expression: 0
                                        * Referenced by: '<S3>/quintic_ref '
                                        */
  real_T enable_quintic_Value;         /* Expression: 0
                                        * Referenced by: '<S3>/enable_quintic  '
                                        */
  real_T Switch_Threshold_j;           /* Expression: 0.5
                                        * Referenced by: '<S18>/Switch'
                                        */
  real_T end_time_A_Value;             /* Expression: 1
                                        * Referenced by: '<S18>/end_time_A'
                                        */
  real_T Constant1_Value_na;           /* Expression: 0.001
                                        * Referenced by: '<S18>/Constant1'
                                        */
  real_T enable_ref_Threshold;         /* Expression: 0
                                        * Referenced by: '<S18>/enable_ref '
                                        */
  real_T Gain2_Gain;                   /* Expression: 350
                                        * Referenced by: '<S26>/Gain2'
                                        */
  real_T Dctleadlag1_P1_Size[2];      /* Computed Parameter: Dctleadlag1_P1_Size
                                       * Referenced by: '<S26>/Dctleadlag1'
                                       */
  real_T Dctleadlag1_P1;               /* Expression: f_num
                                        * Referenced by: '<S26>/Dctleadlag1'
                                        */
  real_T Dctleadlag1_P2_Size[2];      /* Computed Parameter: Dctleadlag1_P2_Size
                                       * Referenced by: '<S26>/Dctleadlag1'
                                       */
  real_T Dctleadlag1_P2;               /* Expression: f_den
                                        * Referenced by: '<S26>/Dctleadlag1'
                                        */
  real_T Dctleadlag1_P3_Size[2];      /* Computed Parameter: Dctleadlag1_P3_Size
                                       * Referenced by: '<S26>/Dctleadlag1'
                                       */
  real_T Dctleadlag1_P3;               /* Expression: 0.001
                                        * Referenced by: '<S26>/Dctleadlag1'
                                        */
  real_T Dct1lowpass1_P1_Size[2];    /* Computed Parameter: Dct1lowpass1_P1_Size
                                      * Referenced by: '<S26>/Dct1lowpass1'
                                      */
  real_T Dct1lowpass1_P1;              /* Expression: f_den
                                        * Referenced by: '<S26>/Dct1lowpass1'
                                        */
  real_T Dct1lowpass1_P2_Size[2];    /* Computed Parameter: Dct1lowpass1_P2_Size
                                      * Referenced by: '<S26>/Dct1lowpass1'
                                      */
  real_T Dct1lowpass1_P2;              /* Expression: 0.001
                                        * Referenced by: '<S26>/Dct1lowpass1'
                                        */
  real_T enable_inner_controller_Value;/* Expression: 0
                                        * Referenced by: '<S3>/enable_inner_controller'
                                        */
  real_T DiscreteTimeIntegrator_gainval;
                           /* Computed Parameter: DiscreteTimeIntegrator_gainval
                            * Referenced by: '<S26>/Discrete-Time Integrator'
                            */
  real_T DiscreteTimeIntegrator_IC;    /* Expression: 0
                                        * Referenced by: '<S26>/Discrete-Time Integrator'
                                        */
  real_T Switch2_Threshold_g;          /* Expression: 0.5
                                        * Referenced by: '<S18>/Switch2'
                                        */
  real_T enable_ID_A_Value;            /* Expression: 0
                                        * Referenced by: '<S18>/enable_ID_A '
                                        */
  real_T enable_inner_closeLoop_Value; /* Expression: 0
                                        * Referenced by: '<S3>/enable_inner_closeLoop'
                                        */
  real_T Switch3_Threshold;            /* Expression: 0.5
                                        * Referenced by: '<S18>/Switch3'
                                        */
  real_T Switch1_Threshold_j;          /* Expression: 0.5
                                        * Referenced by: '<S19>/Switch1'
                                        */
  real_T end_time_b_Value;             /* Expression: 1
                                        * Referenced by: '<S19>/end_time_b '
                                        */
  real_T Constant2_Value_c;            /* Expression: 0.001
                                        * Referenced by: '<S19>/Constant2'
                                        */
  real_T Switch_Threshold_a;           /* Expression: 0
                                        * Referenced by: '<S19>/Switch'
                                        */
  real_T Gain2_Gain_l;                 /* Expression: 450
                                        * Referenced by: '<S39>/Gain2'
                                        */
  real_T Dctleadlag1_P1_Size_n[2];  /* Computed Parameter: Dctleadlag1_P1_Size_n
                                     * Referenced by: '<S39>/Dctleadlag1'
                                     */
  real_T Dctleadlag1_P1_k;             /* Expression: f_num
                                        * Referenced by: '<S39>/Dctleadlag1'
                                        */
  real_T Dctleadlag1_P2_Size_c[2];  /* Computed Parameter: Dctleadlag1_P2_Size_c
                                     * Referenced by: '<S39>/Dctleadlag1'
                                     */
  real_T Dctleadlag1_P2_o;             /* Expression: f_den
                                        * Referenced by: '<S39>/Dctleadlag1'
                                        */
  real_T Dctleadlag1_P3_Size_o[2];  /* Computed Parameter: Dctleadlag1_P3_Size_o
                                     * Referenced by: '<S39>/Dctleadlag1'
                                     */
  real_T Dctleadlag1_P3_d;             /* Expression: 0.001
                                        * Referenced by: '<S39>/Dctleadlag1'
                                        */
  real_T Dct1lowpass1_P1_Size_h[2];/* Computed Parameter: Dct1lowpass1_P1_Size_h
                                    * Referenced by: '<S39>/Dct1lowpass1'
                                    */
  real_T Dct1lowpass1_P1_d;            /* Expression: f_den
                                        * Referenced by: '<S39>/Dct1lowpass1'
                                        */
  real_T Dct1lowpass1_P2_Size_h[2];/* Computed Parameter: Dct1lowpass1_P2_Size_h
                                    * Referenced by: '<S39>/Dct1lowpass1'
                                    */
  real_T Dct1lowpass1_P2_n;            /* Expression: 0.001
                                        * Referenced by: '<S39>/Dct1lowpass1'
                                        */
  real_T DiscreteTimeIntegrator_gainva_m;
                          /* Computed Parameter: DiscreteTimeIntegrator_gainva_m
                           * Referenced by: '<S39>/Discrete-Time Integrator'
                           */
  real_T DiscreteTimeIntegrator_IC_m;  /* Expression: 0
                                        * Referenced by: '<S39>/Discrete-Time Integrator'
                                        */
  real_T Switch2_Threshold_k;          /* Expression: 0.5
                                        * Referenced by: '<S19>/Switch2'
                                        */
  real_T enable_ID_B_Value;            /* Expression: 0
                                        * Referenced by: '<S19>/enable_ID_B '
                                        */
  real_T Switch3_Threshold_m;          /* Expression: 0.5
                                        * Referenced by: '<S19>/Switch3'
                                        */
  real_T u_no_0_init_motion_Value;     /* Expression: 1
                                        * Referenced by: '<S19>/1_no_0_init_motion'
                                        */
  real_T Constant4_Value;              /* Expression: -0.0289
                                        * Referenced by: '<S19>/Constant4'
                                        */
  real_T Switch_Threshold_k;           /* Expression: 0.5
                                        * Referenced by: '<S34>/Switch'
                                        */
  real_T Constant1_Value_p;            /* Expression: 3
                                        * Referenced by: '<S34>/Constant1'
                                        */
  real_T Constant2_Value_g;            /* Expression: 0.001
                                        * Referenced by: '<S34>/Constant2'
                                        */
  real_T Switch1_Threshold_jm;         /* Expression: 0.5
                                        * Referenced by: '<S20>/Switch1'
                                        */
  real_T end_time_c_Value;             /* Expression: 1
                                        * Referenced by: '<S20>/end_time_c '
                                        */
  real_T Constant1_Value_c;            /* Expression: 0.001
                                        * Referenced by: '<S20>/Constant1'
                                        */
  real_T Switch_Threshold_e;           /* Expression: 0
                                        * Referenced by: '<S20>/Switch'
                                        */
  real_T Gain1_Gain_p;                 /* Expression: 350
                                        * Referenced by: '<S54>/Gain1'
                                        */
  real_T Dctleadlag2_P1_Size[2];      /* Computed Parameter: Dctleadlag2_P1_Size
                                       * Referenced by: '<S54>/Dctleadlag2'
                                       */
  real_T Dctleadlag2_P1;               /* Expression: f_num
                                        * Referenced by: '<S54>/Dctleadlag2'
                                        */
  real_T Dctleadlag2_P2_Size[2];      /* Computed Parameter: Dctleadlag2_P2_Size
                                       * Referenced by: '<S54>/Dctleadlag2'
                                       */
  real_T Dctleadlag2_P2;               /* Expression: f_den
                                        * Referenced by: '<S54>/Dctleadlag2'
                                        */
  real_T Dctleadlag2_P3_Size[2];      /* Computed Parameter: Dctleadlag2_P3_Size
                                       * Referenced by: '<S54>/Dctleadlag2'
                                       */
  real_T Dctleadlag2_P3;               /* Expression: 0.001
                                        * Referenced by: '<S54>/Dctleadlag2'
                                        */
  real_T Dct1lowpass2_P1_Size_i[2];/* Computed Parameter: Dct1lowpass2_P1_Size_i
                                    * Referenced by: '<S54>/Dct1lowpass2'
                                    */
  real_T Dct1lowpass2_P1_m;            /* Expression: f_den
                                        * Referenced by: '<S54>/Dct1lowpass2'
                                        */
  real_T Dct1lowpass2_P2_Size_e[2];/* Computed Parameter: Dct1lowpass2_P2_Size_e
                                    * Referenced by: '<S54>/Dct1lowpass2'
                                    */
  real_T Dct1lowpass2_P2_i;            /* Expression: 0.001
                                        * Referenced by: '<S54>/Dct1lowpass2'
                                        */
  real_T DiscreteTimeIntegrator_gainva_g;
                          /* Computed Parameter: DiscreteTimeIntegrator_gainva_g
                           * Referenced by: '<S54>/Discrete-Time Integrator'
                           */
  real_T DiscreteTimeIntegrator_IC_j;  /* Expression: 0
                                        * Referenced by: '<S54>/Discrete-Time Integrator'
                                        */
  real_T Switch2_Threshold_b;          /* Expression: 0.5
                                        * Referenced by: '<S20>/Switch2'
                                        */
  real_T Enable_ID_C_Value;            /* Expression: 0
                                        * Referenced by: '<S20>/Enable_ID_C '
                                        */
  real_T Switch3_Threshold_g;          /* Expression: 0.5
                                        * Referenced by: '<S20>/Switch3'
                                        */
  real_T u_to_enable_id_Value;         /* Expression: 0
                                        * Referenced by: '<S20>/1_to_enable_id'
                                        */
  real_T reser_integrator_Value;       /* Expression: 0
                                        * Referenced by: '<S3>/reser_integrator'
                                        */
  real_T p_zero_Value;                 /* Expression: zeros(1,1)
                                        * Referenced by: '<S6>/p_zero'
                                        */
  real_T m_zero_Value;                 /* Expression: zeros(1,1)
                                        * Referenced by: '<S6>/m_zero'
                                        */
  real_T last_x_InitialCondition[4];   /* Expression: lastx+xoff
                                        * Referenced by: '<S63>/last_x'
                                        */
  real_T umin_scale3_Gain[52];         /* Expression: MVscale(:,ones(1,p+1))'
                                        * Referenced by: '<S63>/umin_scale3'
                                        */
  real_T umin_scale5_Gain[104];        /* Expression: Yscale(:,ones(1,p+1))'
                                        * Referenced by: '<S63>/umin_scale5'
                                        */
  real_T constant_Value[2];            /* Expression: lastu+uoff
                                        * Referenced by: '<S63>/constant'
                                        */
  real_T umin_scale2_Gain[2];          /* Expression: MVscale
                                        * Referenced by: '<S63>/umin_scale2'
                                        */
  real_T Gain1_Gain_i;                 /* Expression: 1
                                        * Referenced by: '<S13>/Gain1'
                                        */
  real_T Dct1lowpass2_P1_Size_i5[2];
                                  /* Computed Parameter: Dct1lowpass2_P1_Size_i5
                                   * Referenced by: '<S13>/Dct1lowpass2'
                                   */
  real_T Dct1lowpass2_P1_n;            /* Expression: f_den
                                        * Referenced by: '<S13>/Dct1lowpass2'
                                        */
  real_T Dct1lowpass2_P2_Size_a[2];/* Computed Parameter: Dct1lowpass2_P2_Size_a
                                    * Referenced by: '<S13>/Dct1lowpass2'
                                    */
  real_T Dct1lowpass2_P2_p;            /* Expression: 0.001
                                        * Referenced by: '<S13>/Dct1lowpass2'
                                        */
  real32_T Gain2_Gain_p;               /* Computed Parameter: Gain2_Gain_p
                                        * Referenced by: '<Root>/Gain2'
                                        */
  real32_T Gain3_Gain;                 /* Computed Parameter: Gain3_Gain
                                        * Referenced by: '<Root>/Gain3'
                                        */
  uint16_T MatrixDimensionCheck_P1[49];
                                  /* Computed Parameter: MatrixDimensionCheck_P1
                                   * Referenced by: '<S64>/Matrix Dimension Check'
                                   */
  uint16_T MatrixDimensionCheck_P2[18];
                                  /* Computed Parameter: MatrixDimensionCheck_P2
                                   * Referenced by: '<S64>/Matrix Dimension Check'
                                   */
  uint16_T MatrixDimensionCheck_P3;    /* Expression: nrow
                                        * Referenced by: '<S64>/Matrix Dimension Check'
                                        */
  uint16_T MatrixDimensionCheck_P4;    /* Expression: ncol
                                        * Referenced by: '<S64>/Matrix Dimension Check'
                                        */
  uint16_T MatrixDimensionCheck_P5;    /* Expression: nsteps
                                        * Referenced by: '<S64>/Matrix Dimension Check'
                                        */
  uint16_T MatrixDimensionCheck_P6;    /* Expression: isltv
                                        * Referenced by: '<S64>/Matrix Dimension Check'
                                        */
  uint16_T MatrixDimensionCheck_P1_o[49];
                                /* Computed Parameter: MatrixDimensionCheck_P1_o
                                 * Referenced by: '<S65>/Matrix Dimension Check'
                                 */
  uint16_T MatrixDimensionCheck_P2_b[18];
                                /* Computed Parameter: MatrixDimensionCheck_P2_b
                                 * Referenced by: '<S65>/Matrix Dimension Check'
                                 */
  uint16_T MatrixDimensionCheck_P3_i;  /* Expression: nrow
                                        * Referenced by: '<S65>/Matrix Dimension Check'
                                        */
  uint16_T MatrixDimensionCheck_P4_p;  /* Expression: ncol
                                        * Referenced by: '<S65>/Matrix Dimension Check'
                                        */
  uint16_T MatrixDimensionCheck_P5_b;  /* Expression: nsteps
                                        * Referenced by: '<S65>/Matrix Dimension Check'
                                        */
  uint16_T MatrixDimensionCheck_P6_p;  /* Expression: isltv
                                        * Referenced by: '<S65>/Matrix Dimension Check'
                                        */
  uint16_T MatrixDimensionCheck_P1_e[49];
                                /* Computed Parameter: MatrixDimensionCheck_P1_e
                                 * Referenced by: '<S66>/Matrix Dimension Check'
                                 */
  uint16_T MatrixDimensionCheck_P2_o[18];
                                /* Computed Parameter: MatrixDimensionCheck_P2_o
                                 * Referenced by: '<S66>/Matrix Dimension Check'
                                 */
  uint16_T MatrixDimensionCheck_P3_f;  /* Expression: nrow
                                        * Referenced by: '<S66>/Matrix Dimension Check'
                                        */
  uint16_T MatrixDimensionCheck_P4_d;  /* Expression: ncol
                                        * Referenced by: '<S66>/Matrix Dimension Check'
                                        */
  uint16_T MatrixDimensionCheck_P5_l;  /* Expression: nsteps
                                        * Referenced by: '<S66>/Matrix Dimension Check'
                                        */
  uint16_T MatrixDimensionCheck_P6_ph; /* Expression: isltv
                                        * Referenced by: '<S66>/Matrix Dimension Check'
                                        */
  uint16_T MatrixDimensionCheck_P1_l[49];
                                /* Computed Parameter: MatrixDimensionCheck_P1_l
                                 * Referenced by: '<S67>/Matrix Dimension Check'
                                 */
  uint16_T MatrixDimensionCheck_P2_bz[3];
                               /* Computed Parameter: MatrixDimensionCheck_P2_bz
                                * Referenced by: '<S67>/Matrix Dimension Check'
                                */
  uint16_T MatrixDimensionCheck_P3_d;  /* Expression: nrow
                                        * Referenced by: '<S67>/Matrix Dimension Check'
                                        */
  uint16_T MatrixDimensionCheck_P4_h;  /* Expression: ncol
                                        * Referenced by: '<S67>/Matrix Dimension Check'
                                        */
  uint16_T MatrixDimensionCheck_P1_i[49];
                                /* Computed Parameter: MatrixDimensionCheck_P1_i
                                 * Referenced by: '<S68>/Matrix Dimension Check'
                                 */
  uint16_T MatrixDimensionCheck_P2_g[2];
                                /* Computed Parameter: MatrixDimensionCheck_P2_g
                                 * Referenced by: '<S68>/Matrix Dimension Check'
                                 */
  uint16_T MatrixDimensionCheck_P3_a;  /* Expression: nrow
                                        * Referenced by: '<S68>/Matrix Dimension Check'
                                        */
  uint16_T MatrixDimensionCheck_P4_b;  /* Expression: ncol
                                        * Referenced by: '<S68>/Matrix Dimension Check'
                                        */
  uint16_T MatrixDimensionCheck_P1_o1[49];
                               /* Computed Parameter: MatrixDimensionCheck_P1_o1
                                * Referenced by: '<S69>/Matrix Dimension Check'
                                */
  uint16_T MatrixDimensionCheck_P2_n[4];
                                /* Computed Parameter: MatrixDimensionCheck_P2_n
                                 * Referenced by: '<S69>/Matrix Dimension Check'
                                 */
  uint16_T MatrixDimensionCheck_P3_b;  /* Expression: nrow
                                        * Referenced by: '<S69>/Matrix Dimension Check'
                                        */
  uint16_T MatrixDimensionCheck_P4_po; /* Expression: ncol
                                        * Referenced by: '<S69>/Matrix Dimension Check'
                                        */
  uint16_T MatrixDimensionCheck_P1_f[49];
                                /* Computed Parameter: MatrixDimensionCheck_P1_f
                                 * Referenced by: '<S70>/Matrix Dimension Check'
                                 */
  uint16_T MatrixDimensionCheck_P2_k[4];
                                /* Computed Parameter: MatrixDimensionCheck_P2_k
                                 * Referenced by: '<S70>/Matrix Dimension Check'
                                 */
  uint16_T MatrixDimensionCheck_P3_l;  /* Expression: nrow
                                        * Referenced by: '<S70>/Matrix Dimension Check'
                                        */
  uint16_T MatrixDimensionCheck_P4_g;  /* Expression: ncol
                                        * Referenced by: '<S70>/Matrix Dimension Check'
                                        */
  uint16_T MatrixDimensionCheck_P1_n[49];
                                /* Computed Parameter: MatrixDimensionCheck_P1_n
                                 * Referenced by: '<S71>/Matrix Dimension Check'
                                 */
  uint16_T MatrixDimensionCheck_P2_f[5];
                                /* Computed Parameter: MatrixDimensionCheck_P2_f
                                 * Referenced by: '<S71>/Matrix Dimension Check'
                                 */
  uint16_T MatrixDimensionCheck_P3_lv; /* Expression: nrow
                                        * Referenced by: '<S71>/Matrix Dimension Check'
                                        */
  uint16_T MatrixDimensionCheck_P4_a;  /* Expression: ncol
                                        * Referenced by: '<S71>/Matrix Dimension Check'
                                        */
  uint16_T VectorDimensionCheck_P1[49];
                                  /* Computed Parameter: VectorDimensionCheck_P1
                                   * Referenced by: '<S72>/Vector Dimension Check'
                                   */
  uint16_T VectorDimensionCheck_P2[6];
                                  /* Computed Parameter: VectorDimensionCheck_P2
                                   * Referenced by: '<S72>/Vector Dimension Check'
                                   */
  uint16_T VectorDimensionCheck_P1_c[49];
                                /* Computed Parameter: VectorDimensionCheck_P1_c
                                 * Referenced by: '<S73>/Vector Dimension Check'
                                 */
  uint16_T VectorDimensionCheck_P2_i[6];
                                /* Computed Parameter: VectorDimensionCheck_P2_i
                                 * Referenced by: '<S73>/Vector Dimension Check'
                                 */
  uint16_T VectorDimensionCheck_P1_a[49];
                                /* Computed Parameter: VectorDimensionCheck_P1_a
                                 * Referenced by: '<S74>/Vector Dimension Check'
                                 */
  uint16_T VectorDimensionCheck_P2_m;
                                /* Computed Parameter: VectorDimensionCheck_P2_m
                                 * Referenced by: '<S74>/Vector Dimension Check'
                                 */
  uint16_T VectorDimensionCheck_P1_b[49];
                                /* Computed Parameter: VectorDimensionCheck_P1_b
                                 * Referenced by: '<S75>/Vector Dimension Check'
                                 */
  uint16_T VectorDimensionCheck_P2_o[6];
                                /* Computed Parameter: VectorDimensionCheck_P2_o
                                 * Referenced by: '<S75>/Vector Dimension Check'
                                 */
  uint16_T VectorDimensionCheck_P3;    /* Expression: n
                                        * Referenced by: '<S75>/Vector Dimension Check'
                                        */
  uint16_T VectorDimensionCheck_P4;    /* Expression: option
                                        * Referenced by: '<S75>/Vector Dimension Check'
                                        */
  uint16_T VectorDimensionCheck_P1_k[49];
                                /* Computed Parameter: VectorDimensionCheck_P1_k
                                 * Referenced by: '<S76>/Vector Dimension Check'
                                 */
  uint16_T VectorDimensionCheck_P2_n[18];
                                /* Computed Parameter: VectorDimensionCheck_P2_n
                                 * Referenced by: '<S76>/Vector Dimension Check'
                                 */
  uint16_T VectorDimensionCheck_P3_i;  /* Expression: n
                                        * Referenced by: '<S76>/Vector Dimension Check'
                                        */
  uint16_T VectorDimensionCheck_P4_n;  /* Expression: option
                                        * Referenced by: '<S76>/Vector Dimension Check'
                                        */
  uint16_T VectorDimensionCheck_P1_j[49];
                                /* Computed Parameter: VectorDimensionCheck_P1_j
                                 * Referenced by: '<S77>/Vector Dimension Check'
                                 */
  uint16_T VectorDimensionCheck_P2_iv[6];
                               /* Computed Parameter: VectorDimensionCheck_P2_iv
                                * Referenced by: '<S77>/Vector Dimension Check'
                                */
  uint16_T VectorDimensionCheck_P3_d;  /* Expression: n
                                        * Referenced by: '<S77>/Vector Dimension Check'
                                        */
  uint16_T VectorDimensionCheck_P4_e;  /* Expression: option
                                        * Referenced by: '<S77>/Vector Dimension Check'
                                        */
  uint16_T VectorDimensionCheck_P1_ba[49];
                               /* Computed Parameter: VectorDimensionCheck_P1_ba
                                * Referenced by: '<S78>/Vector Dimension Check'
                                */
  uint16_T VectorDimensionCheck_P2_ok[4];
                               /* Computed Parameter: VectorDimensionCheck_P2_ok
                                * Referenced by: '<S78>/Vector Dimension Check'
                                */
  uint16_T VectorDimensionCheck_P3_d5; /* Expression: n
                                        * Referenced by: '<S78>/Vector Dimension Check'
                                        */
  uint16_T VectorDimensionCheck_P4_h;  /* Expression: option
                                        * Referenced by: '<S78>/Vector Dimension Check'
                                        */
  uint16_T VectorDimensionCheck_P1_kq[49];
                               /* Computed Parameter: VectorDimensionCheck_P1_kq
                                * Referenced by: '<S79>/Vector Dimension Check'
                                */
  uint16_T VectorDimensionCheck_P2_a[4];
                                /* Computed Parameter: VectorDimensionCheck_P2_a
                                 * Referenced by: '<S79>/Vector Dimension Check'
                                 */
  uint16_T VectorDimensionCheck_P3_c;  /* Expression: n
                                        * Referenced by: '<S79>/Vector Dimension Check'
                                        */
  uint16_T VectorDimensionCheck_P4_a;  /* Expression: option
                                        * Referenced by: '<S79>/Vector Dimension Check'
                                        */
  uint16_T VectorDimensionCheck_P1_h[49];
                                /* Computed Parameter: VectorDimensionCheck_P1_h
                                 * Referenced by: '<S80>/Vector Dimension Check'
                                 */
  uint16_T VectorDimensionCheck_P2_oy[4];
                               /* Computed Parameter: VectorDimensionCheck_P2_oy
                                * Referenced by: '<S80>/Vector Dimension Check'
                                */
  uint16_T VectorDimensionCheck_P3_e;  /* Expression: n
                                        * Referenced by: '<S80>/Vector Dimension Check'
                                        */
  uint16_T VectorDimensionCheck_P4_p;  /* Expression: option
                                        * Referenced by: '<S80>/Vector Dimension Check'
                                        */
  uint16_T VectorDimensionCheck_P1_d[49];
                                /* Computed Parameter: VectorDimensionCheck_P1_d
                                 * Referenced by: '<S81>/Vector Dimension Check'
                                 */
  uint16_T VectorDimensionCheck_P2_as[4];
                               /* Computed Parameter: VectorDimensionCheck_P2_as
                                * Referenced by: '<S81>/Vector Dimension Check'
                                */
  uint16_T VectorDimensionCheck_P3_m;  /* Expression: n
                                        * Referenced by: '<S81>/Vector Dimension Check'
                                        */
  uint16_T VectorDimensionCheck_P4_j;  /* Expression: option
                                        * Referenced by: '<S81>/Vector Dimension Check'
                                        */
  uint16_T VectorDimensionCheck_P1_bz[49];
                               /* Computed Parameter: VectorDimensionCheck_P1_bz
                                * Referenced by: '<S82>/Vector Dimension Check'
                                */
  uint16_T VectorDimensionCheck_P2_am[9];
                               /* Computed Parameter: VectorDimensionCheck_P2_am
                                * Referenced by: '<S82>/Vector Dimension Check'
                                */
  uint16_T VectorDimensionCheck_P3_cy; /* Expression: n
                                        * Referenced by: '<S82>/Vector Dimension Check'
                                        */
  uint16_T VectorDimensionCheck_P4_et; /* Expression: option
                                        * Referenced by: '<S82>/Vector Dimension Check'
                                        */
  boolean_T Memory_InitialCondition[212];/* Expression: iA
                                          * Referenced by: '<S63>/Memory'
                                          */
  P_EnabledSubsystem1_Ball_and__T EnabledSubsystem1_b;/* '<S60>/Enabled Subsystem1' */
  P_EnabledSubsystem_Ball_and_P_T EnabledSubsystem_n;/* '<S60>/Enabled Subsystem' */
  P_EnabledSubsystem1_Ball_and__T EnabledSubsystem1_f;/* '<S46>/Enabled Subsystem1' */
  P_EnabledSubsystem_Ball_and_P_T EnabledSubsystem_a;/* '<S46>/Enabled Subsystem' */
  P_EnabledSubsystem1_Ball_and__T EnabledSubsystem1;/* '<S31>/Enabled Subsystem1' */
  P_EnabledSubsystem_Ball_and_P_T EnabledSubsystem;/* '<S31>/Enabled Subsystem' */
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
    SimStruct childSFunctions[9];
    SimStruct *childSFunctionPtrs[9];
    struct _ssBlkInfo2 blkInfo2[9];
    struct _ssSFcnModelMethods2 methods2[9];
    struct _ssSFcnModelMethods3 methods3[9];
    struct _ssSFcnModelMethods4 methods4[9];
    struct _ssStatesInfo2 statesInfo2[9];
    ssPeriodicStatesInfo periodicStatesInfo[9];
    struct _ssPortInfo2 inputOutputPortInfo2[9];
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
      uint_T attribs[2];
      mxArray *params[2];
      struct _ssDWorkRecord dWork[1];
      struct _ssDWorkAuxRecord dWorkAux[1];
    } Sfcn8;
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
 * '<S5>'   : 'Ball_and_Plate_MicroLabBox_student/MATLAB Function3'
 * '<S6>'   : 'Ball_and_Plate_MicroLabBox_student/MPC Controller'
 * '<S7>'   : 'Ball_and_Plate_MicroLabBox_student/PID Controller'
 * '<S8>'   : 'Ball_and_Plate_MicroLabBox_student/PID Controller1'
 * '<S9>'   : 'Ball_and_Plate_MicroLabBox_student/RTI Data'
 * '<S10>'  : 'Ball_and_Plate_MicroLabBox_student/Subsystem'
 * '<S11>'  : 'Ball_and_Plate_MicroLabBox_student/lowpass 2'
 * '<S12>'  : 'Ball_and_Plate_MicroLabBox_student/lowpass 3'
 * '<S13>'  : 'Ball_and_Plate_MicroLabBox_student/lowpass 6'
 * '<S14>'  : 'Ball_and_Plate_MicroLabBox_student/Ethernet communication/ETHERNET_SETUP_BL1'
 * '<S15>'  : 'Ball_and_Plate_MicroLabBox_student/Ethernet communication/ETHERNET_UDP_RX_BL1'
 * '<S16>'  : 'Ball_and_Plate_MicroLabBox_student/Ethernet communication/ETHERNET_UDP_SETUP_BL1'
 * '<S17>'  : 'Ball_and_Plate_MicroLabBox_student/Innerloop_Actuator/AngleToPos '
 * '<S18>'  : 'Ball_and_Plate_MicroLabBox_student/Innerloop_Actuator/Motor_A'
 * '<S19>'  : 'Ball_and_Plate_MicroLabBox_student/Innerloop_Actuator/Motor_B'
 * '<S20>'  : 'Ball_and_Plate_MicroLabBox_student/Innerloop_Actuator/Motor_C'
 * '<S21>'  : 'Ball_and_Plate_MicroLabBox_student/Innerloop_Actuator/PosToAngle '
 * '<S22>'  : 'Ball_and_Plate_MicroLabBox_student/Innerloop_Actuator/Motor_A/MATLAB Function1'
 * '<S23>'  : 'Ball_and_Plate_MicroLabBox_student/Innerloop_Actuator/Motor_A/Multisine'
 * '<S24>'  : 'Ball_and_Plate_MicroLabBox_student/Innerloop_Actuator/Motor_A/Outputs to Amplifier'
 * '<S25>'  : 'Ball_and_Plate_MicroLabBox_student/Innerloop_Actuator/Motor_A/Position Measurement'
 * '<S26>'  : 'Ball_and_Plate_MicroLabBox_student/Innerloop_Actuator/Motor_A/motorA '
 * '<S27>'  : 'Ball_and_Plate_MicroLabBox_student/Innerloop_Actuator/Motor_A/Multisine/MATLAB Function'
 * '<S28>'  : 'Ball_and_Plate_MicroLabBox_student/Innerloop_Actuator/Motor_A/Outputs to Amplifier/DAC_CLASS1_BL1'
 * '<S29>'  : 'Ball_and_Plate_MicroLabBox_student/Innerloop_Actuator/Motor_A/Position Measurement/EMC_ENCODER_BL1'
 * '<S30>'  : 'Ball_and_Plate_MicroLabBox_student/Innerloop_Actuator/Motor_A/Position Measurement/EMC_ENCODER_POS_SET_BL1'
 * '<S31>'  : 'Ball_and_Plate_MicroLabBox_student/Innerloop_Actuator/Motor_A/Position Measurement/Initial'
 * '<S32>'  : 'Ball_and_Plate_MicroLabBox_student/Innerloop_Actuator/Motor_A/Position Measurement/Initial/Enabled Subsystem'
 * '<S33>'  : 'Ball_and_Plate_MicroLabBox_student/Innerloop_Actuator/Motor_A/Position Measurement/Initial/Enabled Subsystem1'
 * '<S34>'  : 'Ball_and_Plate_MicroLabBox_student/Innerloop_Actuator/Motor_B/Init_motion'
 * '<S35>'  : 'Ball_and_Plate_MicroLabBox_student/Innerloop_Actuator/Motor_B/MATLAB Function2'
 * '<S36>'  : 'Ball_and_Plate_MicroLabBox_student/Innerloop_Actuator/Motor_B/Multisine'
 * '<S37>'  : 'Ball_and_Plate_MicroLabBox_student/Innerloop_Actuator/Motor_B/Outputs to Amplifier'
 * '<S38>'  : 'Ball_and_Plate_MicroLabBox_student/Innerloop_Actuator/Motor_B/Position Measurement'
 * '<S39>'  : 'Ball_and_Plate_MicroLabBox_student/Innerloop_Actuator/Motor_B/motorB'
 * '<S40>'  : 'Ball_and_Plate_MicroLabBox_student/Innerloop_Actuator/Motor_B/Init_motion/MATLAB Function'
 * '<S41>'  : 'Ball_and_Plate_MicroLabBox_student/Innerloop_Actuator/Motor_B/Init_motion/MATLAB Function2'
 * '<S42>'  : 'Ball_and_Plate_MicroLabBox_student/Innerloop_Actuator/Motor_B/Multisine/MATLAB Function'
 * '<S43>'  : 'Ball_and_Plate_MicroLabBox_student/Innerloop_Actuator/Motor_B/Outputs to Amplifier/DAC_CLASS1_BL1'
 * '<S44>'  : 'Ball_and_Plate_MicroLabBox_student/Innerloop_Actuator/Motor_B/Position Measurement/EMC_ENCODER_BL1'
 * '<S45>'  : 'Ball_and_Plate_MicroLabBox_student/Innerloop_Actuator/Motor_B/Position Measurement/EMC_ENCODER_POS_SET_BL1'
 * '<S46>'  : 'Ball_and_Plate_MicroLabBox_student/Innerloop_Actuator/Motor_B/Position Measurement/Initial'
 * '<S47>'  : 'Ball_and_Plate_MicroLabBox_student/Innerloop_Actuator/Motor_B/Position Measurement/Initial/Enabled Subsystem'
 * '<S48>'  : 'Ball_and_Plate_MicroLabBox_student/Innerloop_Actuator/Motor_B/Position Measurement/Initial/Enabled Subsystem1'
 * '<S49>'  : 'Ball_and_Plate_MicroLabBox_student/Innerloop_Actuator/Motor_C/MATLAB Function1'
 * '<S50>'  : 'Ball_and_Plate_MicroLabBox_student/Innerloop_Actuator/Motor_C/Multisine'
 * '<S51>'  : 'Ball_and_Plate_MicroLabBox_student/Innerloop_Actuator/Motor_C/Multisine1'
 * '<S52>'  : 'Ball_and_Plate_MicroLabBox_student/Innerloop_Actuator/Motor_C/Outputs to Amplifier'
 * '<S53>'  : 'Ball_and_Plate_MicroLabBox_student/Innerloop_Actuator/Motor_C/Position Measurement'
 * '<S54>'  : 'Ball_and_Plate_MicroLabBox_student/Innerloop_Actuator/Motor_C/motorC'
 * '<S55>'  : 'Ball_and_Plate_MicroLabBox_student/Innerloop_Actuator/Motor_C/Multisine/MATLAB Function'
 * '<S56>'  : 'Ball_and_Plate_MicroLabBox_student/Innerloop_Actuator/Motor_C/Multisine1/MATLAB Function'
 * '<S57>'  : 'Ball_and_Plate_MicroLabBox_student/Innerloop_Actuator/Motor_C/Outputs to Amplifier/DAC_CLASS1_BL1'
 * '<S58>'  : 'Ball_and_Plate_MicroLabBox_student/Innerloop_Actuator/Motor_C/Position Measurement/EMC_ENCODER_BL1'
 * '<S59>'  : 'Ball_and_Plate_MicroLabBox_student/Innerloop_Actuator/Motor_C/Position Measurement/EMC_ENCODER_POS_SET_BL1'
 * '<S60>'  : 'Ball_and_Plate_MicroLabBox_student/Innerloop_Actuator/Motor_C/Position Measurement/Initial'
 * '<S61>'  : 'Ball_and_Plate_MicroLabBox_student/Innerloop_Actuator/Motor_C/Position Measurement/Initial/Enabled Subsystem'
 * '<S62>'  : 'Ball_and_Plate_MicroLabBox_student/Innerloop_Actuator/Motor_C/Position Measurement/Initial/Enabled Subsystem1'
 * '<S63>'  : 'Ball_and_Plate_MicroLabBox_student/MPC Controller/MPC'
 * '<S64>'  : 'Ball_and_Plate_MicroLabBox_student/MPC Controller/MPC/MPC Matrix Signal Check'
 * '<S65>'  : 'Ball_and_Plate_MicroLabBox_student/MPC Controller/MPC/MPC Matrix Signal Check1'
 * '<S66>'  : 'Ball_and_Plate_MicroLabBox_student/MPC Controller/MPC/MPC Matrix Signal Check2'
 * '<S67>'  : 'Ball_and_Plate_MicroLabBox_student/MPC Controller/MPC/MPC Preview Signal Check'
 * '<S68>'  : 'Ball_and_Plate_MicroLabBox_student/MPC Controller/MPC/MPC Preview Signal Check1'
 * '<S69>'  : 'Ball_and_Plate_MicroLabBox_student/MPC Controller/MPC/MPC Preview Signal Check2'
 * '<S70>'  : 'Ball_and_Plate_MicroLabBox_student/MPC Controller/MPC/MPC Preview Signal Check3'
 * '<S71>'  : 'Ball_and_Plate_MicroLabBox_student/MPC Controller/MPC/MPC Preview Signal Check4'
 * '<S72>'  : 'Ball_and_Plate_MicroLabBox_student/MPC Controller/MPC/MPC Scalar Signal Check'
 * '<S73>'  : 'Ball_and_Plate_MicroLabBox_student/MPC Controller/MPC/MPC Scalar Signal Check1'
 * '<S74>'  : 'Ball_and_Plate_MicroLabBox_student/MPC Controller/MPC/MPC Scalar Signal Check2'
 * '<S75>'  : 'Ball_and_Plate_MicroLabBox_student/MPC Controller/MPC/MPC Vector Signal Check'
 * '<S76>'  : 'Ball_and_Plate_MicroLabBox_student/MPC Controller/MPC/MPC Vector Signal Check1'
 * '<S77>'  : 'Ball_and_Plate_MicroLabBox_student/MPC Controller/MPC/MPC Vector Signal Check11'
 * '<S78>'  : 'Ball_and_Plate_MicroLabBox_student/MPC Controller/MPC/MPC Vector Signal Check2'
 * '<S79>'  : 'Ball_and_Plate_MicroLabBox_student/MPC Controller/MPC/MPC Vector Signal Check3'
 * '<S80>'  : 'Ball_and_Plate_MicroLabBox_student/MPC Controller/MPC/MPC Vector Signal Check4'
 * '<S81>'  : 'Ball_and_Plate_MicroLabBox_student/MPC Controller/MPC/MPC Vector Signal Check5'
 * '<S82>'  : 'Ball_and_Plate_MicroLabBox_student/MPC Controller/MPC/MPC Vector Signal Check6'
 * '<S83>'  : 'Ball_and_Plate_MicroLabBox_student/MPC Controller/MPC/optimizer'
 * '<S84>'  : 'Ball_and_Plate_MicroLabBox_student/MPC Controller/MPC/optimizer/optimizer'
 * '<S85>'  : 'Ball_and_Plate_MicroLabBox_student/PID Controller/Anti-windup'
 * '<S86>'  : 'Ball_and_Plate_MicroLabBox_student/PID Controller/D Gain'
 * '<S87>'  : 'Ball_and_Plate_MicroLabBox_student/PID Controller/Filter'
 * '<S88>'  : 'Ball_and_Plate_MicroLabBox_student/PID Controller/Filter ICs'
 * '<S89>'  : 'Ball_and_Plate_MicroLabBox_student/PID Controller/I Gain'
 * '<S90>'  : 'Ball_and_Plate_MicroLabBox_student/PID Controller/Ideal P Gain'
 * '<S91>'  : 'Ball_and_Plate_MicroLabBox_student/PID Controller/Ideal P Gain Fdbk'
 * '<S92>'  : 'Ball_and_Plate_MicroLabBox_student/PID Controller/Integrator'
 * '<S93>'  : 'Ball_and_Plate_MicroLabBox_student/PID Controller/Integrator ICs'
 * '<S94>'  : 'Ball_and_Plate_MicroLabBox_student/PID Controller/N Copy'
 * '<S95>'  : 'Ball_and_Plate_MicroLabBox_student/PID Controller/N Gain'
 * '<S96>'  : 'Ball_and_Plate_MicroLabBox_student/PID Controller/P Copy'
 * '<S97>'  : 'Ball_and_Plate_MicroLabBox_student/PID Controller/Parallel P Gain'
 * '<S98>'  : 'Ball_and_Plate_MicroLabBox_student/PID Controller/Reset Signal'
 * '<S99>'  : 'Ball_and_Plate_MicroLabBox_student/PID Controller/Saturation'
 * '<S100>' : 'Ball_and_Plate_MicroLabBox_student/PID Controller/Saturation Fdbk'
 * '<S101>' : 'Ball_and_Plate_MicroLabBox_student/PID Controller/Sum'
 * '<S102>' : 'Ball_and_Plate_MicroLabBox_student/PID Controller/Sum Fdbk'
 * '<S103>' : 'Ball_and_Plate_MicroLabBox_student/PID Controller/Tracking Mode'
 * '<S104>' : 'Ball_and_Plate_MicroLabBox_student/PID Controller/Tracking Mode Sum'
 * '<S105>' : 'Ball_and_Plate_MicroLabBox_student/PID Controller/postSat Signal'
 * '<S106>' : 'Ball_and_Plate_MicroLabBox_student/PID Controller/preSat Signal'
 * '<S107>' : 'Ball_and_Plate_MicroLabBox_student/PID Controller/Anti-windup/Passthrough'
 * '<S108>' : 'Ball_and_Plate_MicroLabBox_student/PID Controller/D Gain/Internal Parameters'
 * '<S109>' : 'Ball_and_Plate_MicroLabBox_student/PID Controller/Filter/Disc. Forward Euler Filter'
 * '<S110>' : 'Ball_and_Plate_MicroLabBox_student/PID Controller/Filter ICs/Internal IC - Filter'
 * '<S111>' : 'Ball_and_Plate_MicroLabBox_student/PID Controller/I Gain/Internal Parameters'
 * '<S112>' : 'Ball_and_Plate_MicroLabBox_student/PID Controller/Ideal P Gain/Passthrough'
 * '<S113>' : 'Ball_and_Plate_MicroLabBox_student/PID Controller/Ideal P Gain Fdbk/Disabled'
 * '<S114>' : 'Ball_and_Plate_MicroLabBox_student/PID Controller/Integrator/Discrete'
 * '<S115>' : 'Ball_and_Plate_MicroLabBox_student/PID Controller/Integrator ICs/Internal IC'
 * '<S116>' : 'Ball_and_Plate_MicroLabBox_student/PID Controller/N Copy/Disabled'
 * '<S117>' : 'Ball_and_Plate_MicroLabBox_student/PID Controller/N Gain/Internal Parameters'
 * '<S118>' : 'Ball_and_Plate_MicroLabBox_student/PID Controller/P Copy/Disabled'
 * '<S119>' : 'Ball_and_Plate_MicroLabBox_student/PID Controller/Parallel P Gain/Internal Parameters'
 * '<S120>' : 'Ball_and_Plate_MicroLabBox_student/PID Controller/Reset Signal/External Reset'
 * '<S121>' : 'Ball_and_Plate_MicroLabBox_student/PID Controller/Saturation/Passthrough'
 * '<S122>' : 'Ball_and_Plate_MicroLabBox_student/PID Controller/Saturation Fdbk/Disabled'
 * '<S123>' : 'Ball_and_Plate_MicroLabBox_student/PID Controller/Sum/Sum_PID'
 * '<S124>' : 'Ball_and_Plate_MicroLabBox_student/PID Controller/Sum Fdbk/Disabled'
 * '<S125>' : 'Ball_and_Plate_MicroLabBox_student/PID Controller/Tracking Mode/Disabled'
 * '<S126>' : 'Ball_and_Plate_MicroLabBox_student/PID Controller/Tracking Mode Sum/Passthrough'
 * '<S127>' : 'Ball_and_Plate_MicroLabBox_student/PID Controller/postSat Signal/Forward_Path'
 * '<S128>' : 'Ball_and_Plate_MicroLabBox_student/PID Controller/preSat Signal/Forward_Path'
 * '<S129>' : 'Ball_and_Plate_MicroLabBox_student/PID Controller1/Anti-windup'
 * '<S130>' : 'Ball_and_Plate_MicroLabBox_student/PID Controller1/D Gain'
 * '<S131>' : 'Ball_and_Plate_MicroLabBox_student/PID Controller1/Filter'
 * '<S132>' : 'Ball_and_Plate_MicroLabBox_student/PID Controller1/Filter ICs'
 * '<S133>' : 'Ball_and_Plate_MicroLabBox_student/PID Controller1/I Gain'
 * '<S134>' : 'Ball_and_Plate_MicroLabBox_student/PID Controller1/Ideal P Gain'
 * '<S135>' : 'Ball_and_Plate_MicroLabBox_student/PID Controller1/Ideal P Gain Fdbk'
 * '<S136>' : 'Ball_and_Plate_MicroLabBox_student/PID Controller1/Integrator'
 * '<S137>' : 'Ball_and_Plate_MicroLabBox_student/PID Controller1/Integrator ICs'
 * '<S138>' : 'Ball_and_Plate_MicroLabBox_student/PID Controller1/N Copy'
 * '<S139>' : 'Ball_and_Plate_MicroLabBox_student/PID Controller1/N Gain'
 * '<S140>' : 'Ball_and_Plate_MicroLabBox_student/PID Controller1/P Copy'
 * '<S141>' : 'Ball_and_Plate_MicroLabBox_student/PID Controller1/Parallel P Gain'
 * '<S142>' : 'Ball_and_Plate_MicroLabBox_student/PID Controller1/Reset Signal'
 * '<S143>' : 'Ball_and_Plate_MicroLabBox_student/PID Controller1/Saturation'
 * '<S144>' : 'Ball_and_Plate_MicroLabBox_student/PID Controller1/Saturation Fdbk'
 * '<S145>' : 'Ball_and_Plate_MicroLabBox_student/PID Controller1/Sum'
 * '<S146>' : 'Ball_and_Plate_MicroLabBox_student/PID Controller1/Sum Fdbk'
 * '<S147>' : 'Ball_and_Plate_MicroLabBox_student/PID Controller1/Tracking Mode'
 * '<S148>' : 'Ball_and_Plate_MicroLabBox_student/PID Controller1/Tracking Mode Sum'
 * '<S149>' : 'Ball_and_Plate_MicroLabBox_student/PID Controller1/postSat Signal'
 * '<S150>' : 'Ball_and_Plate_MicroLabBox_student/PID Controller1/preSat Signal'
 * '<S151>' : 'Ball_and_Plate_MicroLabBox_student/PID Controller1/Anti-windup/Passthrough'
 * '<S152>' : 'Ball_and_Plate_MicroLabBox_student/PID Controller1/D Gain/Internal Parameters'
 * '<S153>' : 'Ball_and_Plate_MicroLabBox_student/PID Controller1/Filter/Disc. Forward Euler Filter'
 * '<S154>' : 'Ball_and_Plate_MicroLabBox_student/PID Controller1/Filter ICs/Internal IC - Filter'
 * '<S155>' : 'Ball_and_Plate_MicroLabBox_student/PID Controller1/I Gain/Internal Parameters'
 * '<S156>' : 'Ball_and_Plate_MicroLabBox_student/PID Controller1/Ideal P Gain/Passthrough'
 * '<S157>' : 'Ball_and_Plate_MicroLabBox_student/PID Controller1/Ideal P Gain Fdbk/Disabled'
 * '<S158>' : 'Ball_and_Plate_MicroLabBox_student/PID Controller1/Integrator/Discrete'
 * '<S159>' : 'Ball_and_Plate_MicroLabBox_student/PID Controller1/Integrator ICs/Internal IC'
 * '<S160>' : 'Ball_and_Plate_MicroLabBox_student/PID Controller1/N Copy/Disabled'
 * '<S161>' : 'Ball_and_Plate_MicroLabBox_student/PID Controller1/N Gain/Internal Parameters'
 * '<S162>' : 'Ball_and_Plate_MicroLabBox_student/PID Controller1/P Copy/Disabled'
 * '<S163>' : 'Ball_and_Plate_MicroLabBox_student/PID Controller1/Parallel P Gain/Internal Parameters'
 * '<S164>' : 'Ball_and_Plate_MicroLabBox_student/PID Controller1/Reset Signal/External Reset'
 * '<S165>' : 'Ball_and_Plate_MicroLabBox_student/PID Controller1/Saturation/Passthrough'
 * '<S166>' : 'Ball_and_Plate_MicroLabBox_student/PID Controller1/Saturation Fdbk/Disabled'
 * '<S167>' : 'Ball_and_Plate_MicroLabBox_student/PID Controller1/Sum/Sum_PID'
 * '<S168>' : 'Ball_and_Plate_MicroLabBox_student/PID Controller1/Sum Fdbk/Disabled'
 * '<S169>' : 'Ball_and_Plate_MicroLabBox_student/PID Controller1/Tracking Mode/Disabled'
 * '<S170>' : 'Ball_and_Plate_MicroLabBox_student/PID Controller1/Tracking Mode Sum/Passthrough'
 * '<S171>' : 'Ball_and_Plate_MicroLabBox_student/PID Controller1/postSat Signal/Forward_Path'
 * '<S172>' : 'Ball_and_Plate_MicroLabBox_student/PID Controller1/preSat Signal/Forward_Path'
 * '<S173>' : 'Ball_and_Plate_MicroLabBox_student/RTI Data/RTI Data Store'
 * '<S174>' : 'Ball_and_Plate_MicroLabBox_student/RTI Data/RTI Data Store/RTI Data Store'
 * '<S175>' : 'Ball_and_Plate_MicroLabBox_student/RTI Data/RTI Data Store/RTI Data Store/RTI Data Store'
 * '<S176>' : 'Ball_and_Plate_MicroLabBox_student/Subsystem/Detect Rise Positive'
 * '<S177>' : 'Ball_and_Plate_MicroLabBox_student/Subsystem/MATLAB Function'
 * '<S178>' : 'Ball_and_Plate_MicroLabBox_student/Subsystem/MATLAB Function1'
 * '<S179>' : 'Ball_and_Plate_MicroLabBox_student/Subsystem/Detect Rise Positive/Positive'
 */
#endif                    /* RTW_HEADER_Ball_and_Plate_MicroLabBox_student_h_ */
