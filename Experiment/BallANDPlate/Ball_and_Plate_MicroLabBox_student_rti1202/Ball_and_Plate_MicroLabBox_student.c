/*
 * Ball_and_Plate_MicroLabBox_student.c
 *
 * Academic License - for use in teaching, academic research, and meeting
 * course requirements at degree granting institutions only.  Not for
 * government, commercial, or other organizational use.
 *
 * Code generation for model "Ball_and_Plate_MicroLabBox_student".
 *
 * Model version              : 1.12
 * Simulink Coder version : 9.2 (R2019b) 18-Jul-2019
 * C source code generated on : Wed May 13 12:01:12 2026
 *
 * Target selection: rti1202.tlc
 * Note: GRT includes extra infrastructure and instrumentation for prototyping
 * Embedded hardware selection: Custom Processor->Custom
 * Code generation objectives: Unspecified
 * Validation result: Not run
 */

#include "Ball_and_Plate_MicroLabBox_student_trc_ptr.h"
#include "Ball_and_Plate_MicroLabBox_student.h"
#include "Ball_and_Plate_MicroLabBox_student_private.h"

/* Block signals (default storage) */
B_Ball_and_Plate_MicroLabBox_student_T Ball_and_Plate_MicroLabBox_student_B;

/* Continuous states */
X_Ball_and_Plate_MicroLabBox_student_T Ball_and_Plate_MicroLabBox_student_X;

/* Block states (default storage) */
DW_Ball_and_Plate_MicroLabBox_student_T Ball_and_Plate_MicroLabBox_student_DW;

/* Previous zero-crossings (trigger) states */
PrevZCX_Ball_and_Plate_MicroLabBox_student_T
  Ball_and_Plate_MicroLabBox_student_PrevZCX;

/* External outputs (root outports fed by signals with default storage) */
ExtY_Ball_and_Plate_MicroLabBox_student_T Ball_and_Plate_MicroLabBox_student_Y;

/* Real-time model */
RT_MODEL_Ball_and_Plate_MicroLabBox_student_T
  Ball_and_Plate_MicroLabBox_student_M_;
RT_MODEL_Ball_and_Plate_MicroLabBox_student_T *const
  Ball_and_Plate_MicroLabBox_student_M = &Ball_and_Plate_MicroLabBox_student_M_;

/* Forward declaration for local functions */
static real_T Ball_and_Plate_MicroLabBox_norm(const real_T x[3]);
static void Ball_and_Plate_MicroLa_mldivide(const real_T A[36], real_T B[6]);
static void rate_scheduler(void);

/*
 *   This function updates active task flag for each subrate.
 * The function is called at model base rate, hence the
 * generated code self-manages all its subrates.
 */
static void rate_scheduler(void)
{
  /* Compute which subrates run during the next base time step.  Subrates
   * are an integer multiple of the base rate counter.  Therefore, the subtask
   * counter is reset when it reaches its limit (zero means run).
   */
  (Ball_and_Plate_MicroLabBox_student_M->Timing.TaskCounters.TID[2])++;
  if ((Ball_and_Plate_MicroLabBox_student_M->Timing.TaskCounters.TID[2]) > 99) {/* Sample time: [0.1s, 0.0s] */
    Ball_and_Plate_MicroLabBox_student_M->Timing.TaskCounters.TID[2] = 0;
  }

  Ball_and_Plate_MicroLabBox_student_M->Timing.sampleHits[2] =
    (Ball_and_Plate_MicroLabBox_student_M->Timing.TaskCounters.TID[2] == 0);
}

/*
 * This function updates continuous states using the ODE1 fixed-step
 * solver algorithm
 */
static void rt_ertODEUpdateContinuousStates(RTWSolverInfo *si )
{
  time_T tnew = rtsiGetSolverStopTime(si);
  time_T h = rtsiGetStepSize(si);
  real_T *x = rtsiGetContStates(si);
  ODE1_IntgData *id = (ODE1_IntgData *)rtsiGetSolverData(si);
  real_T *f0 = id->f[0];
  int_T i;
  int_T nXc = 3;
  rtsiSetSimTimeStep(si,MINOR_TIME_STEP);
  rtsiSetdX(si, f0);
  Ball_and_Plate_MicroLabBox_student_derivatives();
  rtsiSetT(si, tnew);
  for (i = 0; i < nXc; ++i) {
    x[i] += h * f0[i];
  }

  rtsiSetSimTimeStep(si,MAJOR_TIME_STEP);
}

/*
 * System initialize for atomic system:
 *    '<S14>/MATLAB Function'
 *    '<S29>/MATLAB Function'
 *    '<S43>/MATLAB Function'
 */
void Ball_and_Pl_MATLABFunction_Init(DW_MATLABFunction_Ball_and_Pl_T *localDW)
{
  localDW->index = 1.0;
  localDW->previous_enable = 0.0;
}

/*
 * Output and update for atomic system:
 *    '<S14>/MATLAB Function'
 *    '<S29>/MATLAB Function'
 *    '<S43>/MATLAB Function'
 */
void Ball_and_Plate_M_MATLABFunction(real_T rtu_enable, const real_T rtu_u[3001],
  B_MATLABFunction_Ball_and_Pla_T *localB, DW_MATLABFunction_Ball_and_Pl_T
  *localDW)
{
  /* MATLAB Function 'Innerloop_Actuator/Motor_A/Init_motion/MATLAB Function': '<S19>:1' */
  /* '<S19>:1:14' */
  if ((rtu_enable != 0.0) && (!(localDW->previous_enable != 0.0))) {
    /* '<S19>:1:17' */
    /* '<S19>:1:18' */
    localDW->index = 1.0;
  }

  if (rtu_enable != 0.0) {
    if (localDW->index <= 3001.0) {
      /* '<S19>:1:25' */
      /* '<S19>:1:26' */
      localB->y = rtu_u[(int32_T)localDW->index - 1];

      /* '<S19>:1:27' */
      localDW->index++;
    } else {
      /* '<S19>:1:31' */
      localB->y = 0.0;
    }
  } else {
    /* '<S19>:1:36' */
    localB->y = -0.0289;
  }

  /* '<S19>:1:39' */
  localDW->previous_enable = rtu_enable;
}

/*
 * Output and update for atomic system:
 *    '<S14>/MATLAB Function2'
 *    '<S29>/MATLAB Function2'
 *    '<S43>/MATLAB Function2'
 */
void Ball_and_Plate__MATLABFunction2(real_T rtu_y, real_T rtu_start, real_T
  rtu_init_value, B_MATLABFunction2_Ball_and_Pl_T *localB)
{
  /* MATLAB Function 'Innerloop_Actuator/Motor_A/Init_motion/MATLAB Function2': '<S20>:1' */
  if (rtu_start > 0.5) {
    /* '<S20>:1:3' */
    /* '<S20>:1:4' */
    localB->path = rtu_init_value;
  } else if (rtu_start < 0.5) {
    /* '<S20>:1:5' */
    /* '<S20>:1:6' */
    localB->path = rtu_y;
  } else {
    /* '<S20>:1:8' */
    localB->path = 0.0;
  }
}

/*
 * System initialize for atomic system:
 *    '<S16>/MATLAB Function'
 *    '<S30>/MATLAB Function'
 *    '<S44>/MATLAB Function'
 *    '<S45>/MATLAB Function'
 */
void Ball_and__MATLABFunction_o_Init(DW_MATLABFunction_Ball_and__j_T *localDW)
{
  localDW->index = 1.0;
  localDW->previous_enable = 0.0;
}

/*
 * Output and update for atomic system:
 *    '<S16>/MATLAB Function'
 *    '<S30>/MATLAB Function'
 *    '<S44>/MATLAB Function'
 *    '<S45>/MATLAB Function'
 */
void Ball_and_Plate_MATLABFunction_e(real_T rtu_enable, const real_T rtu_u
  [200000], B_MATLABFunction_Ball_and_P_m_T *localB,
  DW_MATLABFunction_Ball_and__j_T *localDW)
{
  /* MATLAB Function 'Innerloop_Actuator/Motor_A/Multisine/MATLAB Function': '<S21>:1' */
  /* '<S21>:1:14' */
  if ((rtu_enable != 0.0) && (!(localDW->previous_enable != 0.0))) {
    /* '<S21>:1:17' */
    /* '<S21>:1:18' */
    localDW->index = 1.0;
  }

  if (rtu_enable != 0.0) {
    if (localDW->index <= 200000.0) {
      /* '<S21>:1:25' */
      /* '<S21>:1:26' */
      localB->y = rtu_u[(int32_T)localDW->index - 1];

      /* '<S21>:1:27' */
      localDW->index++;
    } else {
      /* '<S21>:1:31' */
      localB->y = 0.0;
    }
  } else {
    /* '<S21>:1:36' */
    localB->y = 0.0;
  }

  /* '<S21>:1:39' */
  localDW->previous_enable = rtu_enable;
}

/*
 * System initialize for enable system:
 *    '<S25>/Enabled Subsystem'
 *    '<S39>/Enabled Subsystem'
 *    '<S55>/Enabled Subsystem'
 */
void Ball_and__EnabledSubsystem_Init(B_EnabledSubsystem_Ball_and_P_T *localB,
  P_EnabledSubsystem_Ball_and_P_T *localP)
{
  /* SystemInitialize for Outport: '<S26>/Out1' */
  localB->OutportBufferForOut1 = localP->Out1_Y0;
}

/*
 * Disable for enable system:
 *    '<S25>/Enabled Subsystem'
 *    '<S39>/Enabled Subsystem'
 *    '<S55>/Enabled Subsystem'
 */
void Ball_a_EnabledSubsystem_Disable(B_EnabledSubsystem_Ball_and_P_T *localB,
  DW_EnabledSubsystem_Ball_and__T *localDW, P_EnabledSubsystem_Ball_and_P_T
  *localP)
{
  /* Disable for Outport: '<S26>/Out1' */
  localB->OutportBufferForOut1 = localP->Out1_Y0;
  localDW->EnabledSubsystem_MODE = false;
}

/*
 * Start for enable system:
 *    '<S25>/Enabled Subsystem'
 *    '<S39>/Enabled Subsystem'
 *    '<S55>/Enabled Subsystem'
 */
void Ball_and_EnabledSubsystem_Start(DW_EnabledSubsystem_Ball_and__T *localDW)
{
  localDW->EnabledSubsystem_MODE = false;
}

/*
 * Output and update for enable system:
 *    '<S25>/Enabled Subsystem'
 *    '<S39>/Enabled Subsystem'
 *    '<S55>/Enabled Subsystem'
 */
void Ball_and_Plate_EnabledSubsystem
  (RT_MODEL_Ball_and_Plate_MicroLabBox_student_T * const
   Ball_and_Plate_MicroLabBox_student_M, real_T rtu_Enable,
   B_EnabledSubsystem_Ball_and_P_T *localB, DW_EnabledSubsystem_Ball_and__T
   *localDW, P_EnabledSubsystem_Ball_and_P_T *localP)
{
  /* Outputs for Enabled SubSystem: '<S25>/Enabled Subsystem' incorporates:
   *  EnablePort: '<S26>/Enable'
   */
  if (rtmIsMajorTimeStep(Ball_and_Plate_MicroLabBox_student_M)) {
    if (rtu_Enable > 0.0) {
      localDW->EnabledSubsystem_MODE = true;
    } else {
      if (localDW->EnabledSubsystem_MODE) {
        Ball_a_EnabledSubsystem_Disable(localB, localDW, localP);
      }
    }
  }

  if (localDW->EnabledSubsystem_MODE) {
    /* SignalConversion generated from: '<S26>/Out1' incorporates:
     *  Constant: '<S26>/Constant'
     */
    localB->OutportBufferForOut1 = localP->Constant_Value;
  }

  /* End of Outputs for SubSystem: '<S25>/Enabled Subsystem' */
}

/*
 * System initialize for enable system:
 *    '<S25>/Enabled Subsystem1'
 *    '<S39>/Enabled Subsystem1'
 *    '<S55>/Enabled Subsystem1'
 */
void Ball_and_EnabledSubsystem1_Init(B_EnabledSubsystem1_Ball_and__T *localB,
  P_EnabledSubsystem1_Ball_and__T *localP)
{
  /* SystemInitialize for Outport: '<S27>/Out1' */
  localB->In1 = localP->Out1_Y0;
}

/*
 * Disable for enable system:
 *    '<S25>/Enabled Subsystem1'
 *    '<S39>/Enabled Subsystem1'
 *    '<S55>/Enabled Subsystem1'
 */
void Ball__EnabledSubsystem1_Disable(DW_EnabledSubsystem1_Ball_and_T *localDW)
{
  localDW->EnabledSubsystem1_MODE = false;
}

/*
 * Start for enable system:
 *    '<S25>/Enabled Subsystem1'
 *    '<S39>/Enabled Subsystem1'
 *    '<S55>/Enabled Subsystem1'
 */
void Ball_an_EnabledSubsystem1_Start(DW_EnabledSubsystem1_Ball_and_T *localDW)
{
  localDW->EnabledSubsystem1_MODE = false;
}

/*
 * Output and update for enable system:
 *    '<S25>/Enabled Subsystem1'
 *    '<S39>/Enabled Subsystem1'
 *    '<S55>/Enabled Subsystem1'
 */
void Ball_and_Plat_EnabledSubsystem1
  (RT_MODEL_Ball_and_Plate_MicroLabBox_student_T * const
   Ball_and_Plate_MicroLabBox_student_M, boolean_T rtu_Enable, real_T rtu_In1,
   B_EnabledSubsystem1_Ball_and__T *localB, DW_EnabledSubsystem1_Ball_and_T
   *localDW)
{
  /* Outputs for Enabled SubSystem: '<S25>/Enabled Subsystem1' incorporates:
   *  EnablePort: '<S27>/Enable'
   */
  if (rtmIsMajorTimeStep(Ball_and_Plate_MicroLabBox_student_M)) {
    if (rtu_Enable) {
      localDW->EnabledSubsystem1_MODE = true;
    } else {
      if (localDW->EnabledSubsystem1_MODE) {
        Ball__EnabledSubsystem1_Disable(localDW);
      }
    }
  }

  if (localDW->EnabledSubsystem1_MODE && (rtmIsMajorTimeStep
       (Ball_and_Plate_MicroLabBox_student_M) &&
       Ball_and_Plate_MicroLabBox_student_M->Timing.TaskCounters.TID[1] == 0)) {
    /* Inport: '<S27>/In1' */
    localB->In1 = rtu_In1;
  }

  /* End of Outputs for SubSystem: '<S25>/Enabled Subsystem1' */
}

/* Function for MATLAB Function: '<S3>/AngleToPos ' */
static real_T Ball_and_Plate_MicroLabBox_norm(const real_T x[3])
{
  real_T y;
  real_T scale;
  real_T absxk;
  real_T t;
  scale = 3.3121686421112381E-170;
  absxk = fabs(x[0]);
  if (absxk > 3.3121686421112381E-170) {
    y = 1.0;
    scale = absxk;
  } else {
    t = absxk / 3.3121686421112381E-170;
    y = t * t;
  }

  absxk = fabs(x[1]);
  if (absxk > scale) {
    t = scale / absxk;
    y = y * t * t + 1.0;
    scale = absxk;
  } else {
    t = absxk / scale;
    y += t * t;
  }

  absxk = fabs(x[2]);
  if (absxk > scale) {
    t = scale / absxk;
    y = y * t * t + 1.0;
    scale = absxk;
  } else {
    t = absxk / scale;
    y += t * t;
  }

  return scale * sqrt(y);
}

real_T rt_powd_snf(real_T u0, real_T u1)
{
  real_T y;
  real_T tmp;
  real_T tmp_0;
  if (rtIsNaN(u0) || rtIsNaN(u1)) {
    y = (rtNaN);
  } else {
    tmp = fabs(u0);
    tmp_0 = fabs(u1);
    if (rtIsInf(u1)) {
      if (tmp == 1.0) {
        y = 1.0;
      } else if (tmp > 1.0) {
        if (u1 > 0.0) {
          y = (rtInf);
        } else {
          y = 0.0;
        }
      } else if (u1 > 0.0) {
        y = 0.0;
      } else {
        y = (rtInf);
      }
    } else if (tmp_0 == 0.0) {
      y = 1.0;
    } else if (tmp_0 == 1.0) {
      if (u1 > 0.0) {
        y = u0;
      } else {
        y = 1.0 / u0;
      }
    } else if (u1 == 2.0) {
      y = u0 * u0;
    } else if ((u1 == 0.5) && (u0 >= 0.0)) {
      y = sqrt(u0);
    } else if ((u0 < 0.0) && (u1 > floor(u1))) {
      y = (rtNaN);
    } else {
      y = pow(u0, u1);
    }
  }

  return y;
}

/* Function for MATLAB Function: '<S10>/MATLAB Function1' */
static void Ball_and_Plate_MicroLa_mldivide(const real_T A[36], real_T B[6])
{
  real_T b_A[36];
  int8_T ipiv[6];
  int32_T j;
  int32_T kAcol;
  int32_T ix;
  real_T smax;
  int32_T c_k;
  int32_T iy;
  real_T y;
  int32_T c_ix;
  int32_T d;
  int32_T ijA;
  int8_T ipiv_0;
  memcpy(&b_A[0], &A[0], 36U * sizeof(real_T));
  for (j = 0; j < 6; j++) {
    ipiv[j] = (int8_T)(j + 1);
  }

  for (j = 0; j < 5; j++) {
    kAcol = j * 7;
    iy = 0;
    ix = kAcol;
    smax = fabs(b_A[kAcol]);
    for (c_k = 2; c_k <= 6 - j; c_k++) {
      ix++;
      y = fabs(b_A[ix]);
      if (y > smax) {
        iy = c_k - 1;
        smax = y;
      }
    }

    if (b_A[kAcol + iy] != 0.0) {
      if (iy != 0) {
        ipiv[j] = (int8_T)((j + iy) + 1);
        ix = j;
        iy += j;
        for (c_k = 0; c_k < 6; c_k++) {
          smax = b_A[ix];
          b_A[ix] = b_A[iy];
          b_A[iy] = smax;
          ix += 6;
          iy += 6;
        }
      }

      iy = (kAcol - j) + 6;
      for (ix = kAcol + 1; ix < iy; ix++) {
        b_A[ix] /= b_A[kAcol];
      }
    }

    iy = kAcol;
    ix = kAcol + 6;
    for (c_k = 0; c_k <= 4 - j; c_k++) {
      if (b_A[ix] != 0.0) {
        smax = -b_A[ix];
        c_ix = kAcol + 1;
        d = (iy - j) + 12;
        for (ijA = iy + 7; ijA < d; ijA++) {
          b_A[ijA] += b_A[c_ix] * smax;
          c_ix++;
        }
      }

      ix += 6;
      iy += 6;
    }
  }

  for (j = 0; j < 5; j++) {
    ipiv_0 = ipiv[j];
    if (j + 1 != ipiv_0) {
      smax = B[j];
      B[j] = B[ipiv_0 - 1];
      B[ipiv_0 - 1] = smax;
    }
  }

  for (j = 0; j < 6; j++) {
    kAcol = 6 * j;
    if (B[j] != 0.0) {
      for (iy = j + 1; iy + 1 < 7; iy++) {
        B[iy] -= b_A[iy + kAcol] * B[j];
      }
    }
  }

  for (j = 5; j >= 0; j--) {
    kAcol = 6 * j;
    if (B[j] != 0.0) {
      B[j] /= b_A[j + kAcol];
      for (iy = 0; iy < j; iy++) {
        B[iy] -= b_A[iy + kAcol] * B[j];
      }
    }
  }
}

/* Model output function */
void Ball_and_Plate_MicroLabBox_student_output(void)
{
  real_T R_BtoP[9];
  real_T reference_end;
  real_T Ts;
  real_T distance;
  ZCEventType zcEvent;
  real_T b[36];
  real_T T[3];
  real_T T_0[3];
  real_T T_1[3];
  real_T tmp;
  real_T tmp_0;
  real_T tmp_1;
  real_T tmp_2[9];
  real_T tmp_3[9];
  real_T tmp_4[9];
  real_T e[9];
  int32_T i;
  int32_T i_0;
  real_T T_2;
  real_T R_BtoP_0;
  static const int8_T e_0[3] = { 1, 0, 0 };

  static const real_T T_3[3] = { 0.0, 0.0, 0.322 };

  static const real_T b_0[3] = { 0.17, 0.0, 0.0 };

  static const real_T c[3] = { -0.085000000000000075, -0.14722431864335456, 0.0
  };

  static const real_T d[3] = { -0.084999999999999964, 0.14722431864335458, 0.0 };

  static const int8_T b_1[6] = { 1, 0, 0, 0, 0, 0 };

  static const int8_T c_0[6] = { 0, 1, 0, 0, 0, 0 };

  static const int8_T d_0[6] = { 0, 0, 2, 0, 0, 0 };

  if (rtmIsMajorTimeStep(Ball_and_Plate_MicroLabBox_student_M)) {
    /* set solver stop time */
    if (!(Ball_and_Plate_MicroLabBox_student_M->Timing.clockTick0+1)) {
      rtsiSetSolverStopTime(&Ball_and_Plate_MicroLabBox_student_M->solverInfo,
                            ((Ball_and_Plate_MicroLabBox_student_M->Timing.clockTickH0
        + 1) * Ball_and_Plate_MicroLabBox_student_M->Timing.stepSize0 *
        4294967296.0));
    } else {
      rtsiSetSolverStopTime(&Ball_and_Plate_MicroLabBox_student_M->solverInfo,
                            ((Ball_and_Plate_MicroLabBox_student_M->Timing.clockTick0
        + 1) * Ball_and_Plate_MicroLabBox_student_M->Timing.stepSize0 +
        Ball_and_Plate_MicroLabBox_student_M->Timing.clockTickH0 *
        Ball_and_Plate_MicroLabBox_student_M->Timing.stepSize0 * 4294967296.0));
    }
  }                                    /* end MajorTimeStep */

  /* Update absolute time of base rate at minor time step */
  if (rtmIsMinorTimeStep(Ball_and_Plate_MicroLabBox_student_M)) {
    Ball_and_Plate_MicroLabBox_student_M->Timing.t[0] = rtsiGetT
      (&Ball_and_Plate_MicroLabBox_student_M->solverInfo);
  }

  if (rtmIsMajorTimeStep(Ball_and_Plate_MicroLabBox_student_M) &&
      Ball_and_Plate_MicroLabBox_student_M->Timing.TaskCounters.TID[1] == 0) {
    /* DataTypeConversion: '<S2>/Data Type Conversion' incorporates:
     *  Constant: '<S2>/Constant'
     */
    Ball_and_Plate_MicroLabBox_student_B.DataTypeConversion =
      (Ball_and_Plate_MicroLabBox_student_P.Constant_Value_i != 0.0);

    /* S-Function (rti_commonblock): '<S7>/S-Function1' incorporates:
     *  Outport: '<Root>/NumRXFrames'
     *  Outport: '<Root>/Status'
     */
    /* This comment workarounds a code generation problem */
    {
      /* --- Ball_and_Plate_MicroLabBox_student/Ethernet communication/ETHERNET_UDP_RX_BL1: ==> Socket ID = (1) --- */
      /* dSPACE I/O Board DSETHERNET #1 Unit:RXUDP Group:RXUDP */

      /* variable frame size mode is adjusted */

      /* variable declarations */
      DsSSockAddrIn remoteAddr;
      UInt32 addrLen = sizeof(remoteAddr);
      Int32 realLen, tmpStatus;

      /* set remote IP and port initially to 0 within the socket structure */
      remoteAddr.sin_addr.s_addr = (UInt32) 0;
      remoteAddr.sin_port = (UInt16) 0;
      remoteAddr.sin_family = DSIOETH_AF_INET;

      /* whether block is enabled */
      if (Ball_and_Plate_MicroLabBox_student_B.DataTypeConversion == 0) {
        /* block is disabled */
        /**/

        /* set output Received Bytes to 0 */
        Ball_and_Plate_MicroLabBox_student_B.SFunction1_o4 = (uint32_T) 0;

        /* set output Status to state 1 indicating a disabled RX block */
        tmpStatus = 1;
      } else {
        /* block is enabled */
        /**/

        /* receive data from a socket and obtain the address of the sender */
        realLen =
          DsIoEth_recvfrom( DSIOETH_CONNECTION_ID_1,
                           (uint8_T *)
                           &Ball_and_Plate_MicroLabBox_student_B.SFunction1_o1_c[
                           0],
                           32U,
                           DSIOETH_FLAG_NONE,
                           (DsSSockAddr *) &remoteAddr,
                           &addrLen
                           );
        if (realLen > 0) {
          /* set output Status to state 0 indicating successfully received data */
          tmpStatus = 0;

          /* increment output Received Frames  */
          Ball_and_Plate_MicroLabBox_student_Y.NumRXFrames = (real_T)
            (Ball_and_Plate_MicroLabBox_student_Y.NumRXFrames + 1);

          /* set output Received Bytes  to real data length realLen */
          Ball_and_Plate_MicroLabBox_student_B.SFunction1_o4 = (uint32_T)
            realLen;
        } else {
          /* realLen -1: No data received due to empty rx buffer *
           * realLen  0: Connection socket not ready (closed)    */
          tmpStatus = (realLen == -1) ? 2 : 3;

          /* no data received due to empty rx buffer or socket not ready yet */
          Ball_and_Plate_MicroLabBox_student_B.SFunction1_o4 = (uint32_T) 0;
        }
      }                                // if (Inport_Enable == 0)

      /* assign receive status to the related outport */
      Ball_and_Plate_MicroLabBox_student_Y.Status = (uint32_T) tmpStatus;

      /* assign remote IP (s_addr: uint32) to the related outport as (uint8.uint8.uint8.uint8) */
      *((UInt32*) &Ball_and_Plate_MicroLabBox_student_B.SFunction1_o5[0]) =
        remoteAddr.sin_addr.s_addr;

      /* assign remote port to the related outport */
      Ball_and_Plate_MicroLabBox_student_B.SFunction1_o6 = DsIoEth_ntohs
        (remoteAddr.sin_port);
    }

    /* RateTransition: '<Root>/Rate Transition3' incorporates:
     *  Outport: '<Root>/NumRXFrames'
     */
    if (Ball_and_Plate_MicroLabBox_student_DW.RateTransition3_semaphoreTaken ==
        0) {
      Ball_and_Plate_MicroLabBox_student_DW.RateTransition3_Buffer0 =
        Ball_and_Plate_MicroLabBox_student_Y.NumRXFrames;
    }
  }

  /* RateTransition: '<Root>/Rate Transition3' */
  if (rtmIsMajorTimeStep(Ball_and_Plate_MicroLabBox_student_M) &&
      Ball_and_Plate_MicroLabBox_student_M->Timing.TaskCounters.TID[2] == 0) {
    Ball_and_Plate_MicroLabBox_student_DW.RateTransition3_semaphoreTaken = 1;
    Ball_and_Plate_MicroLabBox_student_B.RateTransition3 =
      Ball_and_Plate_MicroLabBox_student_DW.RateTransition3_Buffer0;
    Ball_and_Plate_MicroLabBox_student_DW.RateTransition3_semaphoreTaken = 0;

    /* UnitDelay: '<S1>/UD' */
    Ball_and_Plate_MicroLabBox_student_B.Uk1 =
      Ball_and_Plate_MicroLabBox_student_DW.UD_DSTATE;

    /* Sum: '<S1>/Diff' */
    Ball_and_Plate_MicroLabBox_student_B.Diff =
      Ball_and_Plate_MicroLabBox_student_B.RateTransition3 -
      Ball_and_Plate_MicroLabBox_student_B.Uk1;

    /* Gain: '<Root>/Gain' */
    reference_end = 1.0 / Ball_and_Plate_MicroLabBox_student_P.Ts_Outer;
    Ball_and_Plate_MicroLabBox_student_B.Gain = reference_end *
      Ball_and_Plate_MicroLabBox_student_B.Diff;

    /* Outport: '<Root>/FrameRate ' */
    Ball_and_Plate_MicroLabBox_student_Y.FrameRate =
      Ball_and_Plate_MicroLabBox_student_B.Gain;
  }

  if (rtmIsMajorTimeStep(Ball_and_Plate_MicroLabBox_student_M) &&
      Ball_and_Plate_MicroLabBox_student_M->Timing.TaskCounters.TID[1] == 0) {
    /* Gain: '<S2>/Gain2' */
    Ball_and_Plate_MicroLabBox_student_B.Gain2 = (uint16_T)((uint16_T)((uint32_T)
      Ball_and_Plate_MicroLabBox_student_P.Gain2_Gain *
      Ball_and_Plate_MicroLabBox_student_B.SFunction1_o1_c[1]) << 1);

    /* Sum: '<S2>/Add' */
    Ball_and_Plate_MicroLabBox_student_B.Add = (uint16_T)((uint32_T)
      Ball_and_Plate_MicroLabBox_student_B.SFunction1_o1_c[0] +
      Ball_and_Plate_MicroLabBox_student_B.Gain2);

    /* Gain: '<S2>/Gain1' */
    Ball_and_Plate_MicroLabBox_student_B.Gain1_b = (uint16_T)((uint16_T)
      ((uint32_T)Ball_and_Plate_MicroLabBox_student_P.Gain1_Gain_j *
       Ball_and_Plate_MicroLabBox_student_B.SFunction1_o1_c[5]) << 1);

    /* Sum: '<S2>/Add1' */
    Ball_and_Plate_MicroLabBox_student_B.Add1_l = (uint16_T)((uint32_T)
      Ball_and_Plate_MicroLabBox_student_B.SFunction1_o1_c[4] +
      Ball_and_Plate_MicroLabBox_student_B.Gain1_b);

    /* S-Function (rti_commonblock): '<S6>/S-Function1' */
    /* This comment workarounds a code generation problem */

    /* --- Ball_and_Plate_MicroLabBox_student/Ethernet communication/ETHERNET_SETUP_BL1 --- */
    /* dSPACE I/O Board DSETHERNET #1 Unit:SETUP Group:SETUP */
    {
      /* returns the link state of the ethernet connection. Number of available connectors: uint32_T) */
      Ball_and_Plate_MicroLabBox_student_B.SFunction1_o1_m[0] = (uint32_T)
        DsIoEth_getLinkState((UInt32) 0);
      Ball_and_Plate_MicroLabBox_student_B.SFunction1_o1_m[1] = (uint32_T)
        DsIoEth_getLinkState((UInt32) 1);
      Ball_and_Plate_MicroLabBox_student_B.SFunction1_o1_m[2] = (uint32_T)
        DsIoEth_getLinkState((UInt32) 2);

      /* returns the current IP address of the ethernet interface */
      *((UInt32*) (&Ball_and_Plate_MicroLabBox_student_B.SFunction1_o2_f[0])) =
        DsIoEth_getIpAddress();
    }

    /* S-Function (rti_commonblock): '<S8>/S-Function1' */
    /* This comment workarounds a code generation problem */

    /* --- Ball_and_Plate_MicroLabBox_student/Ethernet communication/ETHERNET_UDP_SETUP_BL1: ==> Socket ID = (1) --- */
    {
      /* dSPACE I/O Board DSETHERNET #1 Unit:SETUPUDP Group:SETUPUDP */

      /* gets the port state (open or closed) of the specified UDP socket */
      if ((DsIoEth_getPortState(DSIOETH_CONNECTION_ID_1) == DSIOETH_PORT_CLOSED))
      {
        /* Opens a non-blocking UDP connection. A Socket must be created before, using    *
         * DsIoEth_create(). It also flushs the related rx socket receive queue to ensure *
         * not to accept any data from a previously adjusted communication                */
        DsIoEth_open(DSIOETH_CONNECTION_ID_1);
      }

      {
        /* variable declarations */
        UInt32 droppedByFilter, droppedByOverflow ;

        /* returns number of dropped frames due to: RecvFrameSize filter setting or RX fifo overflow */
        DsIoEth_getRecvFramesDropped( DSIOETH_CONNECTION_ID_1,
          &droppedByFilter,
          &droppedByOverflow
          ) ;

        /* outport "RX Dropped Frames" contains two double signals that are counters for dropped frames *
         * the first due to RecvFrameSize filter setting, the second due to RX fifo overflow            */
        Ball_and_Plate_MicroLabBox_student_B.SFunction1[0] = (real_T)
          droppedByFilter;
        Ball_and_Plate_MicroLabBox_student_B.SFunction1[1] = (real_T)
          droppedByOverflow;
      }

      /* reads out the pending management event (internal event queue must *
       * be always read, to prevent event queue overrun)                   */
      DsIoEth_getMgmtEvent(DSIOETH_CONNECTION_ID_1);
    }

    /* MATLAB Function: '<S3>/AngleToPos ' incorporates:
     *  Constant: '<Root>/Alpha_ref '
     *  Constant: '<Root>/Beta_ref '
     *  Constant: '<Root>/Psi_ref '
     */
    /* MATLAB Function 'Innerloop_Actuator/AngleToPos ': '<S9>:1' */
    /* '<S9>:1:12' */
    /* '<S9>:1:16' */
    /* '<S9>:1:17' */
    /* '<S9>:1:18' */
    /* '<S9>:1:23' */
    /* '<S9>:1:7' */
    /* '<S9>:1:27' */
    /* '<S9>:1:31' */
    /* '<S9>:1:19' */
    distance = cos(Ball_and_Plate_MicroLabBox_student_P.Psi_ref_Value);
    T_2 = sin(Ball_and_Plate_MicroLabBox_student_P.Psi_ref_Value);
    R_BtoP_0 = sin(Ball_and_Plate_MicroLabBox_student_P.Psi_ref_Value);
    tmp = cos(Ball_and_Plate_MicroLabBox_student_P.Psi_ref_Value);
    tmp_0 = cos(Ball_and_Plate_MicroLabBox_student_P.Beta_ref_Value);
    tmp_1 = sin(Ball_and_Plate_MicroLabBox_student_P.Beta_ref_Value);
    reference_end = sin(Ball_and_Plate_MicroLabBox_student_P.Beta_ref_Value);
    Ts = cos(Ball_and_Plate_MicroLabBox_student_P.Beta_ref_Value);
    tmp_2[0] = distance;
    tmp_2[3] = -T_2;
    tmp_2[6] = 0.0;
    tmp_2[1] = R_BtoP_0;
    tmp_2[4] = tmp;
    tmp_2[7] = 0.0;
    tmp_3[0] = tmp_0;
    tmp_3[3] = 0.0;
    tmp_3[6] = tmp_1;
    tmp_2[2] = 0.0;
    tmp_3[1] = 0.0;
    tmp_2[5] = 0.0;
    tmp_3[4] = 1.0;
    tmp_2[8] = 1.0;
    tmp_3[7] = 0.0;
    tmp_3[2] = -reference_end;
    tmp_3[5] = 0.0;
    tmp_3[8] = Ts;
    distance = cos(Ball_and_Plate_MicroLabBox_student_P.Alpha_ref_Value);
    T_2 = sin(Ball_and_Plate_MicroLabBox_student_P.Alpha_ref_Value);
    R_BtoP_0 = sin(Ball_and_Plate_MicroLabBox_student_P.Alpha_ref_Value);
    tmp = cos(Ball_and_Plate_MicroLabBox_student_P.Alpha_ref_Value);
    for (i = 0; i < 3; i++) {
      for (i_0 = 0; i_0 < 3; i_0++) {
        tmp_4[i + 3 * i_0] = 0.0;
        tmp_0 = tmp_4[3 * i_0 + i];
        tmp_0 += tmp_3[3 * i_0] * tmp_2[i];
        tmp_4[i + 3 * i_0] = tmp_0;
        tmp_0 = tmp_4[3 * i_0 + i];
        tmp_0 += tmp_3[3 * i_0 + 1] * tmp_2[i + 3];
        tmp_4[i + 3 * i_0] = tmp_0;
        tmp_0 = tmp_4[3 * i_0 + i];
        tmp_0 += tmp_3[3 * i_0 + 2] * tmp_2[i + 6];
        tmp_4[i + 3 * i_0] = tmp_0;
      }

      e[3 * i] = e_0[i];
    }

    e[1] = 0.0;
    e[4] = distance;
    e[7] = -T_2;
    e[2] = 0.0;
    e[5] = R_BtoP_0;
    e[8] = tmp;

    /* '<S9>:1:23' */
    /* '<S9>:1:24' */
    for (i = 0; i < 3; i++) {
      T_2 = T_3[i];
      distance = 0.0;
      for (i_0 = 0; i_0 < 3; i_0++) {
        R_BtoP[i + 3 * i_0] = 0.0;
        R_BtoP_0 = R_BtoP[3 * i_0 + i];
        R_BtoP_0 += e[3 * i_0] * tmp_4[i];
        R_BtoP[i + 3 * i_0] = R_BtoP_0;
        R_BtoP_0 = R_BtoP[3 * i_0 + i];
        R_BtoP_0 += e[3 * i_0 + 1] * tmp_4[i + 3];
        R_BtoP[i + 3 * i_0] = R_BtoP_0;
        R_BtoP_0 = R_BtoP[3 * i_0 + i];
        R_BtoP_0 += e[3 * i_0 + 2] * tmp_4[i + 6];
        R_BtoP[i + 3 * i_0] = R_BtoP_0;
        distance += R_BtoP[3 * i_0 + i] * b_0[i_0];
      }

      T_1[i] = (T_2 + distance) - b_0[i];
      distance = R_BtoP[i] * -0.085000000000000075;
      distance += R_BtoP[i + 3] * -0.14722431864335456;
      distance += R_BtoP[i + 6] * 0.0;
      T_0[i] = (T_2 + distance) - c[i];
      distance = R_BtoP[i] * -0.084999999999999964;
      distance += R_BtoP[i + 3] * 0.14722431864335458;
      distance += R_BtoP[i + 6] * 0.0;
      T[i] = (T_2 + distance) - d[i];
    }

    Ball_and_Plate_MicroLabBox_student_B.pos1 = Ball_and_Plate_MicroLabBox_norm
      (T_1);

    /* '<S9>:1:27' */
    /* '<S9>:1:28' */
    Ball_and_Plate_MicroLabBox_student_B.pos2 = Ball_and_Plate_MicroLabBox_norm
      (T_0);

    /* '<S9>:1:31' */
    /* '<S9>:1:32' */
    Ball_and_Plate_MicroLabBox_student_B.pos3 = Ball_and_Plate_MicroLabBox_norm
      (T);

    /* End of MATLAB Function: '<S3>/AngleToPos ' */

    /* Sum: '<S3>/Add1' incorporates:
     *  Constant: '<S3>/Constant'
     */
    Ball_and_Plate_MicroLabBox_student_B.Add1 =
      Ball_and_Plate_MicroLabBox_student_B.pos1 -
      Ball_and_Plate_MicroLabBox_student_P.Constant_Value_o;

    /* Sum: '<S3>/Add2' incorporates:
     *  Constant: '<S3>/Constant1'
     */
    Ball_and_Plate_MicroLabBox_student_B.Add2 =
      Ball_and_Plate_MicroLabBox_student_B.pos2 -
      Ball_and_Plate_MicroLabBox_student_P.Constant1_Value_n;

    /* Sum: '<S3>/Add3' incorporates:
     *  Constant: '<S3>/Constant2'
     */
    Ball_and_Plate_MicroLabBox_student_B.Add3 =
      Ball_and_Plate_MicroLabBox_student_B.pos3 -
      Ball_and_Plate_MicroLabBox_student_P.Constant2_Value;
  }

  /* RateTransition: '<Root>/Rate Transition1' */
  if (rtmIsMajorTimeStep(Ball_and_Plate_MicroLabBox_student_M) &&
      Ball_and_Plate_MicroLabBox_student_M->Timing.TaskCounters.TID[2] == 0) {
    Ball_and_Plate_MicroLabBox_student_B.RateTransition1 =
      Ball_and_Plate_MicroLabBox_student_B.Add1_l;

    /* RateTransition: '<Root>/Rate Transition2' */
    Ball_and_Plate_MicroLabBox_student_B.RateTransition2 =
      Ball_and_Plate_MicroLabBox_student_B.Add;

    /* MATLAB Function: '<Root>/coordinates_ball' */
    /* MATLAB Function 'coordinates_ball': '<S5>:1' */
    /* '<S5>:1:7' */
    /* '<S5>:1:8' */
    /* '<S5>:1:11' */
    /* '<S5>:1:12' */
    /* '<S5>:1:19' */
    Ball_and_Plate_MicroLabBox_student_B.x_2 = ((real_T)
      Ball_and_Plate_MicroLabBox_student_B.RateTransition2 - 295.83) / 1120.25;

    /* '<S5>:1:20' */
    Ball_and_Plate_MicroLabBox_student_B.y_2 = ((real_T)
      Ball_and_Plate_MicroLabBox_student_B.RateTransition1 - 242.82) / 1120.25;
  }

  /* End of RateTransition: '<Root>/Rate Transition1' */

  /* Outputs for Atomic SubSystem: '<S10>/Position Measurement' */
  if (rtmIsMajorTimeStep(Ball_and_Plate_MicroLabBox_student_M) &&
      Ball_and_Plate_MicroLabBox_student_M->Timing.TaskCounters.TID[1] == 0) {
    /* Outputs for Triggered SubSystem: '<S18>/EMC_ENCODER_POS_SET_BL1' incorporates:
     *  TriggerPort: '<S24>/Trigger'
     */
    if (rtmIsMajorTimeStep(Ball_and_Plate_MicroLabBox_student_M)) {
      /* Constant: '<S3>/Init  ' */
      zcEvent = rt_ZCFcn(RISING_ZERO_CROSSING,
                         &Ball_and_Plate_MicroLabBox_student_PrevZCX.EMC_ENCODER_POS_SET_BL1_Trig_fh,
                         (Ball_and_Plate_MicroLabBox_student_P.Init_Value));
      if (zcEvent != NO_ZCEVENT) {
        /* S-Function (rti_commonblock): '<S24>/S-Function1' incorporates:
         *  Constant: '<S18>/Constant'
         */
        /* This comment workarounds a code generation problem */
        {
          /* --- Ball_and_Plate_MicroLabBox_student/Innerloop_Actuator/Motor_A/Position Measurement/EMC_ENCODER_POS_SET_BL1 --- */
          /* --- [RTIEMC, Encoder] - DIO class: 2 - Unit: 1 - Port: 1 - Channel: 1 --- */
          {
            /* -- Position measurement enabled -- */

            /* Set counter position for the selected incremental Encoder (in lines) */
            DioCl2EncoderIn_setEncPosition
              (pRTIEmcEncoder_Unit_1_DioCl_2_Port_1_Ch1,
               (Float64)(Ball_and_Plate_MicroLabBox_student_P.Constant_Value_j *
                         1)
               );

            /* Set positions state for the selected incremental Encoder to VALID */
            DioCl2EncoderIn_setEncPosValidity
              (pRTIEmcEncoder_Unit_1_DioCl_2_Port_1_Ch1,
               DIO_ENC_POSITION_VALID
               );

            /* Writes settings for the incremental Encoder */
            DioCl2EncoderIn_write(pRTIEmcEncoder_Unit_1_DioCl_2_Port_1_Ch1);
          }
        }
      }
    }

    /* End of Outputs for SubSystem: '<S18>/EMC_ENCODER_POS_SET_BL1' */

    /* S-Function (rti_commonblock): '<S23>/S-Function1' */
    /* This comment workarounds a code generation problem */

    /* Gain: '<S18>/Inc2Pos' */
    Ball_and_Plate_MicroLabBox_student_B.Inc2Pos_p =
      Ball_and_Plate_MicroLabBox_student_P.Inc2Pos_Gain *
      Ball_and_Plate_MicroLabBox_student_B.SFunction1_o1_p;

    /* Sum: '<S18>/AbsPosition' incorporates:
     *  Constant: '<S18>/Pos_offset'
     *  Constant: '<S18>/ZP'
     */
    Ball_and_Plate_MicroLabBox_student_B.AbsPosition_m =
      (Ball_and_Plate_MicroLabBox_student_B.Inc2Pos_p -
       Ball_and_Plate_MicroLabBox_student_P.ZP_Value) -
      Ball_and_Plate_MicroLabBox_student_P.Pos_offset_Value;

    /* Abs: '<S25>/Abs' */
    Ball_and_Plate_MicroLabBox_student_B.Abs_h = fabs
      (Ball_and_Plate_MicroLabBox_student_B.SFunction1_o2_d);

    /* Outputs for Enabled SubSystem: '<S25>/Enabled Subsystem' */
    /* Constant: '<S3>/Init  ' */
    Ball_and_Plate_EnabledSubsystem(Ball_and_Plate_MicroLabBox_student_M,
      Ball_and_Plate_MicroLabBox_student_P.Init_Value,
      &Ball_and_Plate_MicroLabBox_student_B.EnabledSubsystem,
      &Ball_and_Plate_MicroLabBox_student_DW.EnabledSubsystem,
      &Ball_and_Plate_MicroLabBox_student_P.EnabledSubsystem);

    /* End of Outputs for SubSystem: '<S25>/Enabled Subsystem' */

    /* RelationalOperator: '<S25>/Relational Operator1' incorporates:
     *  Constant: '<S25>/Const'
     */
    Ball_and_Plate_MicroLabBox_student_B.RelationalOperator1_f =
      (Ball_and_Plate_MicroLabBox_student_B.Abs_h <=
       Ball_and_Plate_MicroLabBox_student_P.Const_Value);
  }

  /* TransferFcn: '<S25>/Transfer Fcn' */
  Ball_and_Plate_MicroLabBox_student_B.TransferFcn_f = 0.0;
  Ball_and_Plate_MicroLabBox_student_B.TransferFcn_f +=
    Ball_and_Plate_MicroLabBox_student_P.TransferFcn_C *
    Ball_and_Plate_MicroLabBox_student_X.TransferFcn_CSTATE_l;

  /* RelationalOperator: '<S25>/Relational Operator' incorporates:
   *  Constant: '<S25>/Const1'
   */
  Ball_and_Plate_MicroLabBox_student_B.RelationalOperator_a =
    (Ball_and_Plate_MicroLabBox_student_P.Const1_Value >=
     Ball_and_Plate_MicroLabBox_student_B.TransferFcn_f);

  /* Logic: '<S25>/Logical Operator2' incorporates:
   *  Constant: '<S3>/Init  '
   */
  Ball_and_Plate_MicroLabBox_student_B.LogicalOperator2_a =
    (Ball_and_Plate_MicroLabBox_student_B.RelationalOperator1_f &&
     Ball_and_Plate_MicroLabBox_student_B.RelationalOperator_a &&
     (Ball_and_Plate_MicroLabBox_student_P.Init_Value != 0.0));

  /* Outputs for Enabled SubSystem: '<S25>/Enabled Subsystem1' */
  Ball_and_Plat_EnabledSubsystem1(Ball_and_Plate_MicroLabBox_student_M,
    Ball_and_Plate_MicroLabBox_student_B.LogicalOperator2_a,
    Ball_and_Plate_MicroLabBox_student_B.Inc2Pos_p,
    &Ball_and_Plate_MicroLabBox_student_B.EnabledSubsystem1,
    &Ball_and_Plate_MicroLabBox_student_DW.EnabledSubsystem1);

  /* End of Outputs for SubSystem: '<S25>/Enabled Subsystem1' */
  if (rtmIsMajorTimeStep(Ball_and_Plate_MicroLabBox_student_M) &&
      Ball_and_Plate_MicroLabBox_student_M->Timing.TaskCounters.TID[1] == 0) {
    /* Gain: '<S18>/mm2m' */
    Ball_and_Plate_MicroLabBox_student_B.mm2m_po =
      Ball_and_Plate_MicroLabBox_student_P.mm2m_Gain *
      Ball_and_Plate_MicroLabBox_student_B.AbsPosition_m;
  }

  /* End of Outputs for SubSystem: '<S10>/Position Measurement' */
  if (rtmIsMajorTimeStep(Ball_and_Plate_MicroLabBox_student_M) &&
      Ball_and_Plate_MicroLabBox_student_M->Timing.TaskCounters.TID[1] == 0) {
    /* Product: '<S3>/Divide1' */
    Ball_and_Plate_MicroLabBox_student_B.Divide1 =
      Ball_and_Plate_MicroLabBox_student_B.mm2m_po /
      Ball_and_Plate_MicroLabBox_student_B.Add1;
  }

  /* Outputs for Atomic SubSystem: '<S11>/Position Measurement' */
  if (rtmIsMajorTimeStep(Ball_and_Plate_MicroLabBox_student_M) &&
      Ball_and_Plate_MicroLabBox_student_M->Timing.TaskCounters.TID[1] == 0) {
    /* Outputs for Triggered SubSystem: '<S32>/EMC_ENCODER_POS_SET_BL1' incorporates:
     *  TriggerPort: '<S38>/Trigger'
     */
    if (rtmIsMajorTimeStep(Ball_and_Plate_MicroLabBox_student_M)) {
      /* Constant: '<S3>/Init  ' */
      zcEvent = rt_ZCFcn(RISING_ZERO_CROSSING,
                         &Ball_and_Plate_MicroLabBox_student_PrevZCX.EMC_ENCODER_POS_SET_BL1_Trig__f,
                         (Ball_and_Plate_MicroLabBox_student_P.Init_Value));
      if (zcEvent != NO_ZCEVENT) {
        /* S-Function (rti_commonblock): '<S38>/S-Function1' incorporates:
         *  Constant: '<S32>/Constant'
         */
        /* This comment workarounds a code generation problem */
        {
          /* --- Ball_and_Plate_MicroLabBox_student/Innerloop_Actuator/Motor_B/Position Measurement/EMC_ENCODER_POS_SET_BL1 --- */
          /* --- [RTIEMC, Encoder] - DIO class: 2 - Unit: 2 - Port: 1 - Channel: 3 --- */
          {
            /* -- Position measurement enabled -- */

            /* Set counter position for the selected incremental Encoder (in lines) */
            DioCl2EncoderIn_setEncPosition
              (pRTIEmcEncoder_Unit_2_DioCl_2_Port_1_Ch3,
               (Float64)(Ball_and_Plate_MicroLabBox_student_P.Constant_Value_j0 *
                         1)
               );

            /* Set positions state for the selected incremental Encoder to VALID */
            DioCl2EncoderIn_setEncPosValidity
              (pRTIEmcEncoder_Unit_2_DioCl_2_Port_1_Ch3,
               DIO_ENC_POSITION_VALID
               );

            /* Writes settings for the incremental Encoder */
            DioCl2EncoderIn_write(pRTIEmcEncoder_Unit_2_DioCl_2_Port_1_Ch3);
          }
        }
      }
    }

    /* End of Outputs for SubSystem: '<S32>/EMC_ENCODER_POS_SET_BL1' */

    /* S-Function (rti_commonblock): '<S37>/S-Function1' */
    /* This comment workarounds a code generation problem */

    /* Gain: '<S32>/Inc2Pos' */
    Ball_and_Plate_MicroLabBox_student_B.Inc2Pos_e =
      Ball_and_Plate_MicroLabBox_student_P.Inc2Pos_Gain_k *
      Ball_and_Plate_MicroLabBox_student_B.SFunction1_o1_i;

    /* Sum: '<S32>/AbsPosition' incorporates:
     *  Constant: '<S32>/Pos_offset'
     *  Constant: '<S32>/ZP'
     */
    Ball_and_Plate_MicroLabBox_student_B.AbsPosition_i =
      (Ball_and_Plate_MicroLabBox_student_B.Inc2Pos_e -
       Ball_and_Plate_MicroLabBox_student_P.ZP_Value_m) -
      Ball_and_Plate_MicroLabBox_student_P.Pos_offset_Value_k;

    /* Abs: '<S39>/Abs' */
    Ball_and_Plate_MicroLabBox_student_B.Abs_j = fabs
      (Ball_and_Plate_MicroLabBox_student_B.SFunction1_o2_p);

    /* Outputs for Enabled SubSystem: '<S39>/Enabled Subsystem' */
    /* Constant: '<S3>/Init  ' */
    Ball_and_Plate_EnabledSubsystem(Ball_and_Plate_MicroLabBox_student_M,
      Ball_and_Plate_MicroLabBox_student_P.Init_Value,
      &Ball_and_Plate_MicroLabBox_student_B.EnabledSubsystem_a,
      &Ball_and_Plate_MicroLabBox_student_DW.EnabledSubsystem_a,
      &Ball_and_Plate_MicroLabBox_student_P.EnabledSubsystem_a);

    /* End of Outputs for SubSystem: '<S39>/Enabled Subsystem' */

    /* RelationalOperator: '<S39>/Relational Operator1' incorporates:
     *  Constant: '<S39>/Const'
     */
    Ball_and_Plate_MicroLabBox_student_B.RelationalOperator1_c =
      (Ball_and_Plate_MicroLabBox_student_B.Abs_j <=
       Ball_and_Plate_MicroLabBox_student_P.Const_Value_g);
  }

  /* TransferFcn: '<S39>/Transfer Fcn' */
  Ball_and_Plate_MicroLabBox_student_B.TransferFcn_o = 0.0;
  Ball_and_Plate_MicroLabBox_student_B.TransferFcn_o +=
    Ball_and_Plate_MicroLabBox_student_P.TransferFcn_C_e *
    Ball_and_Plate_MicroLabBox_student_X.TransferFcn_CSTATE_h;

  /* RelationalOperator: '<S39>/Relational Operator' incorporates:
   *  Constant: '<S39>/Const1'
   */
  Ball_and_Plate_MicroLabBox_student_B.RelationalOperator_n =
    (Ball_and_Plate_MicroLabBox_student_P.Const1_Value_a >=
     Ball_and_Plate_MicroLabBox_student_B.TransferFcn_o);

  /* Logic: '<S39>/Logical Operator2' incorporates:
   *  Constant: '<S3>/Init  '
   */
  Ball_and_Plate_MicroLabBox_student_B.LogicalOperator2_l =
    (Ball_and_Plate_MicroLabBox_student_B.RelationalOperator1_c &&
     Ball_and_Plate_MicroLabBox_student_B.RelationalOperator_n &&
     (Ball_and_Plate_MicroLabBox_student_P.Init_Value != 0.0));

  /* Outputs for Enabled SubSystem: '<S39>/Enabled Subsystem1' */
  Ball_and_Plat_EnabledSubsystem1(Ball_and_Plate_MicroLabBox_student_M,
    Ball_and_Plate_MicroLabBox_student_B.LogicalOperator2_l,
    Ball_and_Plate_MicroLabBox_student_B.Inc2Pos_e,
    &Ball_and_Plate_MicroLabBox_student_B.EnabledSubsystem1_f,
    &Ball_and_Plate_MicroLabBox_student_DW.EnabledSubsystem1_f);

  /* End of Outputs for SubSystem: '<S39>/Enabled Subsystem1' */
  if (rtmIsMajorTimeStep(Ball_and_Plate_MicroLabBox_student_M) &&
      Ball_and_Plate_MicroLabBox_student_M->Timing.TaskCounters.TID[1] == 0) {
    /* Gain: '<S32>/mm2m' */
    Ball_and_Plate_MicroLabBox_student_B.mm2m_p =
      Ball_and_Plate_MicroLabBox_student_P.mm2m_Gain_f *
      Ball_and_Plate_MicroLabBox_student_B.AbsPosition_i;
  }

  /* End of Outputs for SubSystem: '<S11>/Position Measurement' */
  if (rtmIsMajorTimeStep(Ball_and_Plate_MicroLabBox_student_M) &&
      Ball_and_Plate_MicroLabBox_student_M->Timing.TaskCounters.TID[1] == 0) {
    /* Product: '<S3>/Divide2' */
    Ball_and_Plate_MicroLabBox_student_B.Divide2 =
      Ball_and_Plate_MicroLabBox_student_B.mm2m_p /
      Ball_and_Plate_MicroLabBox_student_B.Add2;
  }

  /* Outputs for Atomic SubSystem: '<S12>/Position Measurement' */
  if (rtmIsMajorTimeStep(Ball_and_Plate_MicroLabBox_student_M) &&
      Ball_and_Plate_MicroLabBox_student_M->Timing.TaskCounters.TID[1] == 0) {
    /* Outputs for Triggered SubSystem: '<S47>/EMC_ENCODER_POS_SET_BL1' incorporates:
     *  TriggerPort: '<S54>/Trigger'
     */
    if (rtmIsMajorTimeStep(Ball_and_Plate_MicroLabBox_student_M)) {
      /* Constant: '<S3>/Init  ' */
      zcEvent = rt_ZCFcn(RISING_ZERO_CROSSING,
                         &Ball_and_Plate_MicroLabBox_student_PrevZCX.EMC_ENCODER_POS_SET_BL1_Trig_ZC,
                         (Ball_and_Plate_MicroLabBox_student_P.Init_Value));
      if (zcEvent != NO_ZCEVENT) {
        /* S-Function (rti_commonblock): '<S54>/S-Function1' incorporates:
         *  Constant: '<S47>/Constant'
         */
        /* This comment workarounds a code generation problem */
        {
          /* --- Ball_and_Plate_MicroLabBox_student/Innerloop_Actuator/Motor_C/Position Measurement/EMC_ENCODER_POS_SET_BL1 --- */
          /* --- [RTIEMC, Encoder] - DIO class: 2 - Unit: 3 - Port: 1 - Channel: 5 --- */
          {
            /* -- Position measurement enabled -- */

            /* Set counter position for the selected incremental Encoder (in lines) */
            DioCl2EncoderIn_setEncPosition
              (pRTIEmcEncoder_Unit_3_DioCl_2_Port_1_Ch5,
               (Float64)(Ball_and_Plate_MicroLabBox_student_P.Constant_Value_f *
                         1)
               );

            /* Set positions state for the selected incremental Encoder to VALID */
            DioCl2EncoderIn_setEncPosValidity
              (pRTIEmcEncoder_Unit_3_DioCl_2_Port_1_Ch5,
               DIO_ENC_POSITION_VALID
               );

            /* Writes settings for the incremental Encoder */
            DioCl2EncoderIn_write(pRTIEmcEncoder_Unit_3_DioCl_2_Port_1_Ch5);
          }
        }
      }
    }

    /* End of Outputs for SubSystem: '<S47>/EMC_ENCODER_POS_SET_BL1' */

    /* S-Function (rti_commonblock): '<S53>/S-Function1' */
    /* This comment workarounds a code generation problem */

    /* Gain: '<S47>/Inc2Pos' */
    Ball_and_Plate_MicroLabBox_student_B.Inc2Pos =
      Ball_and_Plate_MicroLabBox_student_P.Inc2Pos_Gain_h *
      Ball_and_Plate_MicroLabBox_student_B.SFunction1_o1;

    /* Sum: '<S47>/AbsPosition' incorporates:
     *  Constant: '<S47>/Pos_offset'
     *  Constant: '<S47>/ZP'
     */
    Ball_and_Plate_MicroLabBox_student_B.AbsPosition =
      (Ball_and_Plate_MicroLabBox_student_B.Inc2Pos -
       Ball_and_Plate_MicroLabBox_student_P.ZP_Value_p) -
      Ball_and_Plate_MicroLabBox_student_P.Pos_offset_Value_f;

    /* Abs: '<S55>/Abs' */
    Ball_and_Plate_MicroLabBox_student_B.Abs = fabs
      (Ball_and_Plate_MicroLabBox_student_B.SFunction1_o2);

    /* Outputs for Enabled SubSystem: '<S55>/Enabled Subsystem' */
    /* Constant: '<S3>/Init  ' */
    Ball_and_Plate_EnabledSubsystem(Ball_and_Plate_MicroLabBox_student_M,
      Ball_and_Plate_MicroLabBox_student_P.Init_Value,
      &Ball_and_Plate_MicroLabBox_student_B.EnabledSubsystem_n,
      &Ball_and_Plate_MicroLabBox_student_DW.EnabledSubsystem_n,
      &Ball_and_Plate_MicroLabBox_student_P.EnabledSubsystem_n);

    /* End of Outputs for SubSystem: '<S55>/Enabled Subsystem' */

    /* RelationalOperator: '<S55>/Relational Operator1' incorporates:
     *  Constant: '<S55>/Const'
     */
    Ball_and_Plate_MicroLabBox_student_B.RelationalOperator1 =
      (Ball_and_Plate_MicroLabBox_student_B.Abs <=
       Ball_and_Plate_MicroLabBox_student_P.Const_Value_m);
  }

  /* TransferFcn: '<S55>/Transfer Fcn' */
  Ball_and_Plate_MicroLabBox_student_B.TransferFcn = 0.0;
  Ball_and_Plate_MicroLabBox_student_B.TransferFcn +=
    Ball_and_Plate_MicroLabBox_student_P.TransferFcn_C_c *
    Ball_and_Plate_MicroLabBox_student_X.TransferFcn_CSTATE;

  /* RelationalOperator: '<S55>/Relational Operator' incorporates:
   *  Constant: '<S55>/Const1'
   */
  Ball_and_Plate_MicroLabBox_student_B.RelationalOperator =
    (Ball_and_Plate_MicroLabBox_student_P.Const1_Value_p >=
     Ball_and_Plate_MicroLabBox_student_B.TransferFcn);

  /* Logic: '<S55>/Logical Operator2' incorporates:
   *  Constant: '<S3>/Init  '
   */
  Ball_and_Plate_MicroLabBox_student_B.LogicalOperator2 =
    (Ball_and_Plate_MicroLabBox_student_B.RelationalOperator1 &&
     Ball_and_Plate_MicroLabBox_student_B.RelationalOperator &&
     (Ball_and_Plate_MicroLabBox_student_P.Init_Value != 0.0));

  /* Outputs for Enabled SubSystem: '<S55>/Enabled Subsystem1' */
  Ball_and_Plat_EnabledSubsystem1(Ball_and_Plate_MicroLabBox_student_M,
    Ball_and_Plate_MicroLabBox_student_B.LogicalOperator2,
    Ball_and_Plate_MicroLabBox_student_B.Inc2Pos,
    &Ball_and_Plate_MicroLabBox_student_B.EnabledSubsystem1_b,
    &Ball_and_Plate_MicroLabBox_student_DW.EnabledSubsystem1_b);

  /* End of Outputs for SubSystem: '<S55>/Enabled Subsystem1' */
  if (rtmIsMajorTimeStep(Ball_and_Plate_MicroLabBox_student_M) &&
      Ball_and_Plate_MicroLabBox_student_M->Timing.TaskCounters.TID[1] == 0) {
    /* Gain: '<S47>/mm2m' */
    Ball_and_Plate_MicroLabBox_student_B.mm2m =
      Ball_and_Plate_MicroLabBox_student_P.mm2m_Gain_o *
      Ball_and_Plate_MicroLabBox_student_B.AbsPosition;
  }

  /* End of Outputs for SubSystem: '<S12>/Position Measurement' */
  if (rtmIsMajorTimeStep(Ball_and_Plate_MicroLabBox_student_M) &&
      Ball_and_Plate_MicroLabBox_student_M->Timing.TaskCounters.TID[1] == 0) {
    /* Product: '<S3>/Divide3' */
    Ball_and_Plate_MicroLabBox_student_B.Divide3 =
      Ball_and_Plate_MicroLabBox_student_B.mm2m /
      Ball_and_Plate_MicroLabBox_student_B.Add3;

    /* MATLAB Function: '<S14>/MATLAB Function' incorporates:
     *  Constant: '<S14>/Constant4'
     *  Constant: '<S3>/enable_start '
     */
    Ball_and_Plate_M_MATLABFunction
      (Ball_and_Plate_MicroLabBox_student_P.enable_start_Value,
       Ball_and_Plate_MicroLabBox_student_P.path,
       &Ball_and_Plate_MicroLabBox_student_B.sf_MATLABFunction,
       &Ball_and_Plate_MicroLabBox_student_DW.sf_MATLABFunction);

    /* MATLAB Function: '<S10>/MATLAB Function1' incorporates:
     *  Constant: '<S10>/Constant1'
     *  Constant: '<S10>/Constant2'
     */
    reference_end = Ball_and_Plate_MicroLabBox_student_B.Add1;
    Ts = Ball_and_Plate_MicroLabBox_student_P.Constant2_Value_j;

    /* MATLAB Function 'Innerloop_Actuator/Motor_A/MATLAB Function1': '<S15>:1' */
    /* '<S15>:1:44' */
    /* '<S15>:1:45' */
    /* '<S15>:1:46' */
    if (!Ball_and_Plate_MicroLabBox_student_DW.prev_ref_end_not_empty) {
      /* '<S15>:1:12' */
      /* '<S15>:1:13' */
      Ball_and_Plate_MicroLabBox_student_DW.prev_ref_end =
        Ball_and_Plate_MicroLabBox_student_B.Add1;
      Ball_and_Plate_MicroLabBox_student_DW.prev_ref_end_not_empty = true;
    }

    if (Ball_and_Plate_MicroLabBox_student_B.Add1 !=
        Ball_and_Plate_MicroLabBox_student_DW.prev_ref_end) {
      /* '<S15>:1:20' */
      /* '<S15>:1:22' */
      distance = fabs(Ball_and_Plate_MicroLabBox_student_B.mm2m_po -
                      Ball_and_Plate_MicroLabBox_student_B.Add1);
      if ((Ball_and_Plate_MicroLabBox_student_P.Constant1_Value_l > 0.0) &&
          (distance > 0.0)) {
        /* '<S15>:1:25' */
        /* '<S15>:1:26' */
        Ball_and_Plate_MicroLabBox_student_DW.tf_internal = distance /
          Ball_and_Plate_MicroLabBox_student_P.Constant1_Value_l;
      } else {
        /* '<S15>:1:28' */
        Ball_and_Plate_MicroLabBox_student_DW.tf_internal = 0.0;
      }

      /* '<S15>:1:32' */
      Ball_and_Plate_MicroLabBox_student_DW.t_elapsed = 0.0;
      if (Ball_and_Plate_MicroLabBox_student_DW.tf_internal > 0.0) {
        /* '<S15>:1:42' */
        /* '<S15>:1:44' */
        /* '<S15>:1:51' */
        /* '<S15>:1:52' */
        Ball_and_Plate_MicroLabBox_student_DW.coeffs[0] =
          Ball_and_Plate_MicroLabBox_student_B.mm2m_po;
        Ball_and_Plate_MicroLabBox_student_DW.coeffs[1] = 0.0;
        Ball_and_Plate_MicroLabBox_student_DW.coeffs[2] = 0.0;
        Ball_and_Plate_MicroLabBox_student_DW.coeffs[3] =
          Ball_and_Plate_MicroLabBox_student_B.Add1;
        Ball_and_Plate_MicroLabBox_student_DW.coeffs[4] = 0.0;
        Ball_and_Plate_MicroLabBox_student_DW.coeffs[5] = 0.0;
        distance = rt_powd_snf(Ball_and_Plate_MicroLabBox_student_DW.tf_internal,
          3.0);
        T_2 = rt_powd_snf(Ball_and_Plate_MicroLabBox_student_DW.tf_internal, 4.0);
        R_BtoP_0 = rt_powd_snf(Ball_and_Plate_MicroLabBox_student_DW.tf_internal,
          5.0);
        tmp = rt_powd_snf(Ball_and_Plate_MicroLabBox_student_DW.tf_internal, 3.0);
        tmp_0 = rt_powd_snf(Ball_and_Plate_MicroLabBox_student_DW.tf_internal,
                            4.0);
        tmp_1 = rt_powd_snf(Ball_and_Plate_MicroLabBox_student_DW.tf_internal,
                            3.0);
        for (i = 0; i < 6; i++) {
          b[6 * i] = b_1[i];
          b[6 * i + 1] = c_0[i];
          b[6 * i + 2] = d_0[i];
        }

        b[3] = 1.0;
        b[9] = Ball_and_Plate_MicroLabBox_student_DW.tf_internal;
        b[15] = Ball_and_Plate_MicroLabBox_student_DW.tf_internal *
          Ball_and_Plate_MicroLabBox_student_DW.tf_internal;
        b[21] = distance;
        b[27] = T_2;
        b[33] = R_BtoP_0;
        b[4] = 0.0;
        b[10] = 1.0;
        b[16] = 2.0 * Ball_and_Plate_MicroLabBox_student_DW.tf_internal;
        b[22] = Ball_and_Plate_MicroLabBox_student_DW.tf_internal *
          Ball_and_Plate_MicroLabBox_student_DW.tf_internal * 3.0;
        b[28] = 4.0 * tmp;
        b[34] = 5.0 * tmp_0;
        b[5] = 0.0;
        b[11] = 0.0;
        b[17] = 2.0;
        b[23] = 6.0 * Ball_and_Plate_MicroLabBox_student_DW.tf_internal;
        b[29] = Ball_and_Plate_MicroLabBox_student_DW.tf_internal *
          Ball_and_Plate_MicroLabBox_student_DW.tf_internal * 12.0;
        b[35] = 20.0 * tmp_1;
        Ball_and_Plate_MicroLa_mldivide(b,
          Ball_and_Plate_MicroLabBox_student_DW.coeffs);
      } else {
        /* '<S15>:1:55' */
        Ball_and_Plate_MicroLabBox_student_DW.coeffs[0] =
          Ball_and_Plate_MicroLabBox_student_B.Add1;
        Ball_and_Plate_MicroLabBox_student_DW.coeffs[1] = 0.0;
        Ball_and_Plate_MicroLabBox_student_DW.coeffs[2] = 0.0;
        Ball_and_Plate_MicroLabBox_student_DW.coeffs[3] = 0.0;
        Ball_and_Plate_MicroLabBox_student_DW.coeffs[4] = 0.0;
        Ball_and_Plate_MicroLabBox_student_DW.coeffs[5] = 0.0;
      }
    }

    if (Ball_and_Plate_MicroLabBox_student_DW.t_elapsed <=
        Ball_and_Plate_MicroLabBox_student_DW.tf_internal) {
      /* '<S15>:1:60' */
      /* '<S15>:1:63' */
      distance = ((((Ball_and_Plate_MicroLabBox_student_DW.coeffs[1] *
                     Ball_and_Plate_MicroLabBox_student_DW.t_elapsed +
                     Ball_and_Plate_MicroLabBox_student_DW.coeffs[0]) +
                    Ball_and_Plate_MicroLabBox_student_DW.t_elapsed *
                    Ball_and_Plate_MicroLabBox_student_DW.t_elapsed *
                    Ball_and_Plate_MicroLabBox_student_DW.coeffs[2]) +
                   Ball_and_Plate_MicroLabBox_student_DW.coeffs[3] * rt_powd_snf
                   (Ball_and_Plate_MicroLabBox_student_DW.t_elapsed, 3.0)) +
                  Ball_and_Plate_MicroLabBox_student_DW.coeffs[4] * rt_powd_snf
                  (Ball_and_Plate_MicroLabBox_student_DW.t_elapsed, 4.0)) +
        Ball_and_Plate_MicroLabBox_student_DW.coeffs[5] * rt_powd_snf
        (Ball_and_Plate_MicroLabBox_student_DW.t_elapsed, 5.0);

      /* '<S15>:1:67' */
      Ball_and_Plate_MicroLabBox_student_DW.t_elapsed += Ts;
    } else {
      /* '<S15>:1:70' */
      distance = reference_end;
    }

    /* '<S15>:1:74' */
    Ball_and_Plate_MicroLabBox_student_DW.prev_ref_end = reference_end;
    Ball_and_Plate_MicroLabBox_student_B.path = distance;

    /* End of MATLAB Function: '<S10>/MATLAB Function1' */

    /* Switch: '<S10>/enable_ref ' incorporates:
     *  Constant: '<S3>/enable_ref '
     */
    if (Ball_and_Plate_MicroLabBox_student_P.enable_ref_Value >
        Ball_and_Plate_MicroLabBox_student_P.enable_ref_Threshold) {
      Ball_and_Plate_MicroLabBox_student_B.enable_ref =
        Ball_and_Plate_MicroLabBox_student_B.sf_MATLABFunction.y;
    } else {
      Ball_and_Plate_MicroLabBox_student_B.enable_ref =
        Ball_and_Plate_MicroLabBox_student_B.path;
    }

    /* End of Switch: '<S10>/enable_ref ' */

    /* Sum: '<S10>/Sum1' */
    Ball_and_Plate_MicroLabBox_student_B.Sum1 =
      Ball_and_Plate_MicroLabBox_student_B.enable_ref -
      Ball_and_Plate_MicroLabBox_student_B.mm2m_po;

    /* Gain: '<S13>/Gain1' */
    Ball_and_Plate_MicroLabBox_student_B.Gain1 =
      Ball_and_Plate_MicroLabBox_student_P.Gain1_Gain *
      Ball_and_Plate_MicroLabBox_student_B.Sum1;

    /* S-Function (dleadlag): '<S13>/Dctleadlag2' */

    /* Level2 S-Function Block: '<S13>/Dctleadlag2' (dleadlag) */
    {
      SimStruct *rts = Ball_and_Plate_MicroLabBox_student_M->childSfunctions[0];
      sfcnOutputs(rts,1);
    }

    /* S-Function (dlowpass1): '<S13>/Dct1lowpass3' */

    /* Level2 S-Function Block: '<S13>/Dct1lowpass3' (dlowpass1) */
    {
      SimStruct *rts = Ball_and_Plate_MicroLabBox_student_M->childSfunctions[1];
      sfcnOutputs(rts,1);
    }

    /* Switch: '<S10>/Switch2' incorporates:
     *  Constant: '<S10>/Constant'
     *  Constant: '<S3>/controller_disable'
     */
    if (Ball_and_Plate_MicroLabBox_student_P.controller_disable_Value >=
        Ball_and_Plate_MicroLabBox_student_P.Switch2_Threshold) {
      Ball_and_Plate_MicroLabBox_student_B.Switch2 =
        Ball_and_Plate_MicroLabBox_student_P.Constant_Value;
    } else {
      Ball_and_Plate_MicroLabBox_student_B.Switch2 =
        Ball_and_Plate_MicroLabBox_student_B.Dct1lowpass3;
    }

    /* End of Switch: '<S10>/Switch2' */

    /* MATLAB Function: '<S16>/MATLAB Function' incorporates:
     *  Constant: '<S10>/enable_ID_A '
     *  Constant: '<S16>/Constant'
     */
    Ball_and_Plate_MATLABFunction_e
      (Ball_and_Plate_MicroLabBox_student_P.enable_ID_A_Value,
       Ball_and_Plate_MicroLabBox_student_P.uA,
       &Ball_and_Plate_MicroLabBox_student_B.sf_MATLABFunction_e,
       &Ball_and_Plate_MicroLabBox_student_DW.sf_MATLABFunction_e);

    /* Sum: '<S10>/Sum' */
    Ball_and_Plate_MicroLabBox_student_B.Sum =
      Ball_and_Plate_MicroLabBox_student_B.Switch2 +
      Ball_and_Plate_MicroLabBox_student_B.sf_MATLABFunction_e.y;
  }

  /* Switch: '<S10>/Switch3' incorporates:
   *  Constant: '<S3>/CloseLoop_disable'
   */
  if (Ball_and_Plate_MicroLabBox_student_P.CloseLoop_disable_Value >=
      Ball_and_Plate_MicroLabBox_student_P.Switch3_Threshold) {
    Ball_and_Plate_MicroLabBox_student_B.Switch3 =
      Ball_and_Plate_MicroLabBox_student_B.TransferFcn_f;
  } else {
    Ball_and_Plate_MicroLabBox_student_B.Switch3 =
      Ball_and_Plate_MicroLabBox_student_B.Sum;
  }

  /* End of Switch: '<S10>/Switch3' */

  /* Outputs for Atomic SubSystem: '<S10>/Outputs to Amplifier' */

  /* Saturate: '<S17>/Saturation' */
  distance = Ball_and_Plate_MicroLabBox_student_B.Switch3;
  T_2 = Ball_and_Plate_MicroLabBox_student_P.Saturation_LowerSat;
  R_BtoP_0 = Ball_and_Plate_MicroLabBox_student_P.Saturation_UpperSat;
  if (distance > R_BtoP_0) {
    Ball_and_Plate_MicroLabBox_student_B.Saturation_a = R_BtoP_0;
  } else if (distance < T_2) {
    Ball_and_Plate_MicroLabBox_student_B.Saturation_a = T_2;
  } else {
    Ball_and_Plate_MicroLabBox_student_B.Saturation_a = distance;
  }

  /* End of Saturate: '<S17>/Saturation' */

  /* Gain: '<S17>/Current2V' */
  Ball_and_Plate_MicroLabBox_student_B.Current2V_h =
    Ball_and_Plate_MicroLabBox_student_P.Current2V_Gain *
    Ball_and_Plate_MicroLabBox_student_B.Saturation_a;

  /* Gain: '<S17>/DSPscale' */
  Ball_and_Plate_MicroLabBox_student_B.DSPscale_m =
    Ball_and_Plate_MicroLabBox_student_P.DSPscale_Gain *
    Ball_and_Plate_MicroLabBox_student_B.Current2V_h;
  if (rtmIsMajorTimeStep(Ball_and_Plate_MicroLabBox_student_M) &&
      Ball_and_Plate_MicroLabBox_student_M->Timing.TaskCounters.TID[1] == 0) {
    /* S-Function (rti_commonblock): '<S22>/S-Function1' */
    /* This comment workarounds a code generation problem */

    /* --- Ball_and_Plate_MicroLabBox_student/Innerloop_Actuator/Motor_A/Outputs to Amplifier/DAC_CLASS1_BL1 --- */
    /* --- [RTI120X, DAC C1] - Channel: 1 --- */
    {
      /* define variables required for DAC realtime functions */
      Float64 inportDacData= 0.0;
      inportDacData = (real_T) Ball_and_Plate_MicroLabBox_student_B.DSPscale_m;

      /* write value of CL1 DAC for output channel 1 */
      DacCl1AnalogOut_setOutputValue(pRTIDacC1AnalogOut_Ch_1,
        DAC_CLASS1_CHANNEL_1, inportDacData);
      DacCl1AnalogOut_write(pRTIDacC1AnalogOut_Ch_1);
    }
  }

  /* End of Outputs for SubSystem: '<S10>/Outputs to Amplifier' */
  if (rtmIsMajorTimeStep(Ball_and_Plate_MicroLabBox_student_M) &&
      Ball_and_Plate_MicroLabBox_student_M->Timing.TaskCounters.TID[1] == 0) {
    /* MATLAB Function: '<S14>/MATLAB Function2' incorporates:
     *  Constant: '<S10>/1_no_0_init_motion'
     *  Constant: '<S10>/Constant4'
     */
    Ball_and_Plate__MATLABFunction2(0.0,
      Ball_and_Plate_MicroLabBox_student_P.u_no_0_init_motion_Value,
      Ball_and_Plate_MicroLabBox_student_P.Constant4_Value,
      &Ball_and_Plate_MicroLabBox_student_B.sf_MATLABFunction2);

    /* Switch: '<S14>/Switch' incorporates:
     *  Constant: '<S10>/1_no_0_init_motion'
     *  Constant: '<S10>/Constant4'
     *  Constant: '<S14>/Constant3'
     */
    if (Ball_and_Plate_MicroLabBox_student_P.u_no_0_init_motion_Value >
        Ball_and_Plate_MicroLabBox_student_P.Switch_Threshold) {
      Ball_and_Plate_MicroLabBox_student_B.Switch =
        Ball_and_Plate_MicroLabBox_student_P.Constant4_Value;
    } else {
      Ball_and_Plate_MicroLabBox_student_B.Switch =
        Ball_and_Plate_MicroLabBox_student_P.Constant3_Value;
    }

    /* End of Switch: '<S14>/Switch' */

    /* Constant: '<S14>/Constant1' */
    Ball_and_Plate_MicroLabBox_student_B.Constant1 =
      Ball_and_Plate_MicroLabBox_student_P.Constant1_Value_i;

    /* Constant: '<S14>/Constant2' */
    Ball_and_Plate_MicroLabBox_student_B.Constant2 =
      Ball_and_Plate_MicroLabBox_student_P.Constant2_Value_e;

    /* MATLAB Function: '<S29>/MATLAB Function' incorporates:
     *  Constant: '<S29>/Constant4'
     *  Constant: '<S3>/enable_start '
     */
    Ball_and_Plate_M_MATLABFunction
      (Ball_and_Plate_MicroLabBox_student_P.enable_start_Value,
       Ball_and_Plate_MicroLabBox_student_P.path,
       &Ball_and_Plate_MicroLabBox_student_B.sf_MATLABFunction_k,
       &Ball_and_Plate_MicroLabBox_student_DW.sf_MATLABFunction_k);

    /* Switch: '<S11>/Switch' incorporates:
     *  Constant: '<S3>/enable_ref '
     */
    if (Ball_and_Plate_MicroLabBox_student_P.enable_ref_Value >
        Ball_and_Plate_MicroLabBox_student_P.Switch_Threshold_a) {
      Ball_and_Plate_MicroLabBox_student_B.Switch_j =
        Ball_and_Plate_MicroLabBox_student_B.sf_MATLABFunction_k.y;
    } else {
      Ball_and_Plate_MicroLabBox_student_B.Switch_j =
        Ball_and_Plate_MicroLabBox_student_B.Add2;
    }

    /* End of Switch: '<S11>/Switch' */

    /* Sum: '<S11>/Sum1' */
    Ball_and_Plate_MicroLabBox_student_B.Sum1_f =
      Ball_and_Plate_MicroLabBox_student_B.Switch_j -
      Ball_and_Plate_MicroLabBox_student_B.mm2m_p;

    /* Gain: '<S28>/Gain1' */
    Ball_and_Plate_MicroLabBox_student_B.Gain1_o =
      Ball_and_Plate_MicroLabBox_student_P.Gain1_Gain_n *
      Ball_and_Plate_MicroLabBox_student_B.Sum1_f;

    /* S-Function (dleadlag): '<S28>/Dctleadlag2' */

    /* Level2 S-Function Block: '<S28>/Dctleadlag2' (dleadlag) */
    {
      SimStruct *rts = Ball_and_Plate_MicroLabBox_student_M->childSfunctions[2];
      sfcnOutputs(rts,1);
    }

    /* S-Function (dlowpass1): '<S28>/Dct1lowpass3' */

    /* Level2 S-Function Block: '<S28>/Dct1lowpass3' (dlowpass1) */
    {
      SimStruct *rts = Ball_and_Plate_MicroLabBox_student_M->childSfunctions[3];
      sfcnOutputs(rts,1);
    }

    /* Switch: '<S11>/Switch2' incorporates:
     *  Constant: '<S11>/Constant1'
     *  Constant: '<S3>/controller_disable'
     */
    if (Ball_and_Plate_MicroLabBox_student_P.controller_disable_Value >=
        Ball_and_Plate_MicroLabBox_student_P.Switch2_Threshold_k) {
      Ball_and_Plate_MicroLabBox_student_B.Switch2_p =
        Ball_and_Plate_MicroLabBox_student_P.Constant1_Value;
    } else {
      Ball_and_Plate_MicroLabBox_student_B.Switch2_p =
        Ball_and_Plate_MicroLabBox_student_B.Dct1lowpass3_c;
    }

    /* End of Switch: '<S11>/Switch2' */

    /* MATLAB Function: '<S30>/MATLAB Function' incorporates:
     *  Constant: '<S11>/enable_ID_B '
     *  Constant: '<S30>/Constant'
     */
    Ball_and_Plate_MATLABFunction_e
      (Ball_and_Plate_MicroLabBox_student_P.enable_ID_B_Value,
       Ball_and_Plate_MicroLabBox_student_P.uB,
       &Ball_and_Plate_MicroLabBox_student_B.sf_MATLABFunction_ko,
       &Ball_and_Plate_MicroLabBox_student_DW.sf_MATLABFunction_ko);

    /* Sum: '<S11>/Sum' */
    Ball_and_Plate_MicroLabBox_student_B.Sum_g =
      Ball_and_Plate_MicroLabBox_student_B.Switch2_p +
      Ball_and_Plate_MicroLabBox_student_B.sf_MATLABFunction_ko.y;
  }

  /* Switch: '<S11>/Switch3' incorporates:
   *  Constant: '<S3>/CloseLoop_disable'
   */
  if (Ball_and_Plate_MicroLabBox_student_P.CloseLoop_disable_Value >=
      Ball_and_Plate_MicroLabBox_student_P.Switch3_Threshold_m) {
    Ball_and_Plate_MicroLabBox_student_B.Switch3_d =
      Ball_and_Plate_MicroLabBox_student_B.TransferFcn_o;
  } else {
    Ball_and_Plate_MicroLabBox_student_B.Switch3_d =
      Ball_and_Plate_MicroLabBox_student_B.Sum_g;
  }

  /* End of Switch: '<S11>/Switch3' */

  /* Outputs for Atomic SubSystem: '<S11>/Outputs to Amplifier' */

  /* Saturate: '<S31>/Saturation' */
  distance = Ball_and_Plate_MicroLabBox_student_B.Switch3_d;
  T_2 = Ball_and_Plate_MicroLabBox_student_P.Saturation_LowerSat_f;
  R_BtoP_0 = Ball_and_Plate_MicroLabBox_student_P.Saturation_UpperSat_e;
  if (distance > R_BtoP_0) {
    Ball_and_Plate_MicroLabBox_student_B.Saturation_l = R_BtoP_0;
  } else if (distance < T_2) {
    Ball_and_Plate_MicroLabBox_student_B.Saturation_l = T_2;
  } else {
    Ball_and_Plate_MicroLabBox_student_B.Saturation_l = distance;
  }

  /* End of Saturate: '<S31>/Saturation' */

  /* Gain: '<S31>/Current2V' */
  Ball_and_Plate_MicroLabBox_student_B.Current2V_f =
    Ball_and_Plate_MicroLabBox_student_P.Current2V_Gain_f *
    Ball_and_Plate_MicroLabBox_student_B.Saturation_l;

  /* Gain: '<S31>/DSPscale' */
  Ball_and_Plate_MicroLabBox_student_B.DSPscale_o =
    Ball_and_Plate_MicroLabBox_student_P.DSPscale_Gain_d *
    Ball_and_Plate_MicroLabBox_student_B.Current2V_f;
  if (rtmIsMajorTimeStep(Ball_and_Plate_MicroLabBox_student_M) &&
      Ball_and_Plate_MicroLabBox_student_M->Timing.TaskCounters.TID[1] == 0) {
    /* S-Function (rti_commonblock): '<S36>/S-Function1' */
    /* This comment workarounds a code generation problem */

    /* --- Ball_and_Plate_MicroLabBox_student/Innerloop_Actuator/Motor_B/Outputs to Amplifier/DAC_CLASS1_BL1 --- */
    /* --- [RTI120X, DAC C1] - Channel: 2 --- */
    {
      /* define variables required for DAC realtime functions */
      Float64 inportDacData= 0.0;
      inportDacData = (real_T) Ball_and_Plate_MicroLabBox_student_B.DSPscale_o;

      /* write value of CL1 DAC for output channel 2 */
      DacCl1AnalogOut_setOutputValue(pRTIDacC1AnalogOut_Ch_2,
        DAC_CLASS1_CHANNEL_2, inportDacData);
      DacCl1AnalogOut_write(pRTIDacC1AnalogOut_Ch_2);
    }
  }

  /* End of Outputs for SubSystem: '<S11>/Outputs to Amplifier' */
  if (rtmIsMajorTimeStep(Ball_and_Plate_MicroLabBox_student_M) &&
      Ball_and_Plate_MicroLabBox_student_M->Timing.TaskCounters.TID[1] == 0) {
    /* MATLAB Function: '<S29>/MATLAB Function2' incorporates:
     *  Constant: '<S11>/1_no_0_init_motion'
     *  Constant: '<S11>/Constant4'
     */
    Ball_and_Plate__MATLABFunction2(0.0,
      Ball_and_Plate_MicroLabBox_student_P.u_no_0_init_motion_Value_e,
      Ball_and_Plate_MicroLabBox_student_P.Constant4_Value_b,
      &Ball_and_Plate_MicroLabBox_student_B.sf_MATLABFunction2_m);

    /* Switch: '<S29>/Switch' incorporates:
     *  Constant: '<S11>/1_no_0_init_motion'
     *  Constant: '<S11>/Constant4'
     *  Constant: '<S29>/Constant3'
     */
    if (Ball_and_Plate_MicroLabBox_student_P.u_no_0_init_motion_Value_e >
        Ball_and_Plate_MicroLabBox_student_P.Switch_Threshold_k) {
      Ball_and_Plate_MicroLabBox_student_B.Switch_n =
        Ball_and_Plate_MicroLabBox_student_P.Constant4_Value_b;
    } else {
      Ball_and_Plate_MicroLabBox_student_B.Switch_n =
        Ball_and_Plate_MicroLabBox_student_P.Constant3_Value_i;
    }

    /* End of Switch: '<S29>/Switch' */

    /* Constant: '<S29>/Constant1' */
    Ball_and_Plate_MicroLabBox_student_B.Constant1_c =
      Ball_and_Plate_MicroLabBox_student_P.Constant1_Value_p;

    /* Constant: '<S29>/Constant2' */
    Ball_and_Plate_MicroLabBox_student_B.Constant2_b =
      Ball_and_Plate_MicroLabBox_student_P.Constant2_Value_g;

    /* MATLAB Function: '<S43>/MATLAB Function' incorporates:
     *  Constant: '<S3>/enable_start '
     *  Constant: '<S43>/Constant4'
     */
    Ball_and_Plate_M_MATLABFunction
      (Ball_and_Plate_MicroLabBox_student_P.enable_start_Value,
       Ball_and_Plate_MicroLabBox_student_P.path,
       &Ball_and_Plate_MicroLabBox_student_B.sf_MATLABFunction_h,
       &Ball_and_Plate_MicroLabBox_student_DW.sf_MATLABFunction_h);

    /* Switch: '<S12>/Switch' incorporates:
     *  Constant: '<S3>/enable_ref '
     */
    if (Ball_and_Plate_MicroLabBox_student_P.enable_ref_Value >
        Ball_and_Plate_MicroLabBox_student_P.Switch_Threshold_e) {
      Ball_and_Plate_MicroLabBox_student_B.Switch_jo =
        Ball_and_Plate_MicroLabBox_student_B.sf_MATLABFunction_h.y;
    } else {
      Ball_and_Plate_MicroLabBox_student_B.Switch_jo =
        Ball_and_Plate_MicroLabBox_student_B.Add3;
    }

    /* End of Switch: '<S12>/Switch' */

    /* Sum: '<S12>/Sum1' */
    Ball_and_Plate_MicroLabBox_student_B.Sum1_h =
      Ball_and_Plate_MicroLabBox_student_B.Switch_jo -
      Ball_and_Plate_MicroLabBox_student_B.mm2m;

    /* Gain: '<S42>/Gain1' */
    Ball_and_Plate_MicroLabBox_student_B.Gain1_e =
      Ball_and_Plate_MicroLabBox_student_P.Gain1_Gain_m *
      Ball_and_Plate_MicroLabBox_student_B.Sum1_h;

    /* S-Function (dleadlag): '<S42>/Dctleadlag2' */

    /* Level2 S-Function Block: '<S42>/Dctleadlag2' (dleadlag) */
    {
      SimStruct *rts = Ball_and_Plate_MicroLabBox_student_M->childSfunctions[4];
      sfcnOutputs(rts,1);
    }

    /* S-Function (dlowpass1): '<S42>/Dct1lowpass3' */

    /* Level2 S-Function Block: '<S42>/Dct1lowpass3' (dlowpass1) */
    {
      SimStruct *rts = Ball_and_Plate_MicroLabBox_student_M->childSfunctions[5];
      sfcnOutputs(rts,1);
    }

    /* Switch: '<S12>/Switch2' incorporates:
     *  Constant: '<S12>/Constant'
     *  Constant: '<S3>/controller_disable'
     */
    if (Ball_and_Plate_MicroLabBox_student_P.controller_disable_Value >=
        Ball_and_Plate_MicroLabBox_student_P.Switch2_Threshold_b) {
      Ball_and_Plate_MicroLabBox_student_B.Switch2_h =
        Ball_and_Plate_MicroLabBox_student_P.Constant_Value_e;
    } else {
      Ball_and_Plate_MicroLabBox_student_B.Switch2_h =
        Ball_and_Plate_MicroLabBox_student_B.Dct1lowpass3_l;
    }

    /* End of Switch: '<S12>/Switch2' */

    /* MATLAB Function: '<S45>/MATLAB Function' incorporates:
     *  Constant: '<S12>/Enable_ID_C '
     *  Constant: '<S45>/Constant'
     */
    Ball_and_Plate_MATLABFunction_e
      (Ball_and_Plate_MicroLabBox_student_P.Enable_ID_C_Value,
       Ball_and_Plate_MicroLabBox_student_P.uC,
       &Ball_and_Plate_MicroLabBox_student_B.sf_MATLABFunction_hg,
       &Ball_and_Plate_MicroLabBox_student_DW.sf_MATLABFunction_hg);

    /* Sum: '<S12>/Sum' */
    Ball_and_Plate_MicroLabBox_student_B.Sum_h =
      Ball_and_Plate_MicroLabBox_student_B.Switch2_h +
      Ball_and_Plate_MicroLabBox_student_B.sf_MATLABFunction_hg.y;
  }

  /* Switch: '<S12>/Switch3' incorporates:
   *  Constant: '<S3>/CloseLoop_disable'
   */
  if (Ball_and_Plate_MicroLabBox_student_P.CloseLoop_disable_Value >=
      Ball_and_Plate_MicroLabBox_student_P.Switch3_Threshold_g) {
    Ball_and_Plate_MicroLabBox_student_B.Switch3_l =
      Ball_and_Plate_MicroLabBox_student_B.TransferFcn;
  } else {
    Ball_and_Plate_MicroLabBox_student_B.Switch3_l =
      Ball_and_Plate_MicroLabBox_student_B.Sum_h;
  }

  /* End of Switch: '<S12>/Switch3' */

  /* Outputs for Atomic SubSystem: '<S12>/Outputs to Amplifier' */

  /* Saturate: '<S46>/Saturation' */
  distance = Ball_and_Plate_MicroLabBox_student_B.Switch3_l;
  T_2 = Ball_and_Plate_MicroLabBox_student_P.Saturation_LowerSat_p;
  R_BtoP_0 = Ball_and_Plate_MicroLabBox_student_P.Saturation_UpperSat_h;
  if (distance > R_BtoP_0) {
    Ball_and_Plate_MicroLabBox_student_B.Saturation = R_BtoP_0;
  } else if (distance < T_2) {
    Ball_and_Plate_MicroLabBox_student_B.Saturation = T_2;
  } else {
    Ball_and_Plate_MicroLabBox_student_B.Saturation = distance;
  }

  /* End of Saturate: '<S46>/Saturation' */

  /* Gain: '<S46>/Current2V' */
  Ball_and_Plate_MicroLabBox_student_B.Current2V =
    Ball_and_Plate_MicroLabBox_student_P.Current2V_Gain_p *
    Ball_and_Plate_MicroLabBox_student_B.Saturation;

  /* Gain: '<S46>/DSPscale' */
  Ball_and_Plate_MicroLabBox_student_B.DSPscale =
    Ball_and_Plate_MicroLabBox_student_P.DSPscale_Gain_h *
    Ball_and_Plate_MicroLabBox_student_B.Current2V;
  if (rtmIsMajorTimeStep(Ball_and_Plate_MicroLabBox_student_M) &&
      Ball_and_Plate_MicroLabBox_student_M->Timing.TaskCounters.TID[1] == 0) {
    /* S-Function (rti_commonblock): '<S52>/S-Function1' */
    /* This comment workarounds a code generation problem */

    /* --- Ball_and_Plate_MicroLabBox_student/Innerloop_Actuator/Motor_C/Outputs to Amplifier/DAC_CLASS1_BL1 --- */
    /* --- [RTI120X, DAC C1] - Channel: 3 --- */
    {
      /* define variables required for DAC realtime functions */
      Float64 inportDacData= 0.0;
      inportDacData = (real_T) Ball_and_Plate_MicroLabBox_student_B.DSPscale;

      /* write value of CL1 DAC for output channel 3 */
      DacCl1AnalogOut_setOutputValue(pRTIDacC1AnalogOut_Ch_3,
        DAC_CLASS1_CHANNEL_3, inportDacData);
      DacCl1AnalogOut_write(pRTIDacC1AnalogOut_Ch_3);
    }
  }

  /* End of Outputs for SubSystem: '<S12>/Outputs to Amplifier' */
  if (rtmIsMajorTimeStep(Ball_and_Plate_MicroLabBox_student_M) &&
      Ball_and_Plate_MicroLabBox_student_M->Timing.TaskCounters.TID[1] == 0) {
    /* MATLAB Function: '<S43>/MATLAB Function2' incorporates:
     *  Constant: '<S12>/1_no_0_init_motion'
     *  Constant: '<S12>/Constant4'
     */
    Ball_and_Plate__MATLABFunction2(0.0,
      Ball_and_Plate_MicroLabBox_student_P.u_no_0_init_motion_Value_n,
      Ball_and_Plate_MicroLabBox_student_P.Constant4_Value_c,
      &Ball_and_Plate_MicroLabBox_student_B.sf_MATLABFunction2_e);

    /* Switch: '<S43>/Switch' incorporates:
     *  Constant: '<S12>/1_no_0_init_motion'
     *  Constant: '<S12>/Constant4'
     *  Constant: '<S43>/Constant3'
     */
    if (Ball_and_Plate_MicroLabBox_student_P.u_no_0_init_motion_Value_n >
        Ball_and_Plate_MicroLabBox_student_P.Switch_Threshold_l) {
      Ball_and_Plate_MicroLabBox_student_B.Switch_e =
        Ball_and_Plate_MicroLabBox_student_P.Constant4_Value_c;
    } else {
      Ball_and_Plate_MicroLabBox_student_B.Switch_e =
        Ball_and_Plate_MicroLabBox_student_P.Constant3_Value_p;
    }

    /* End of Switch: '<S43>/Switch' */

    /* Constant: '<S43>/Constant1' */
    Ball_and_Plate_MicroLabBox_student_B.Constant1_h =
      Ball_and_Plate_MicroLabBox_student_P.Constant1_Value_f;

    /* Constant: '<S43>/Constant2' */
    Ball_and_Plate_MicroLabBox_student_B.Constant2_h =
      Ball_and_Plate_MicroLabBox_student_P.Constant2_Value_i;

    /* MATLAB Function: '<S44>/MATLAB Function' incorporates:
     *  Constant: '<S12>/1_to_enable_id'
     *  Constant: '<S44>/Constant'
     */
    Ball_and_Plate_MATLABFunction_e
      (Ball_and_Plate_MicroLabBox_student_P.u_to_enable_id_Value,
       Ball_and_Plate_MicroLabBox_student_P.uA,
       &Ball_and_Plate_MicroLabBox_student_B.sf_MATLABFunction_g,
       &Ball_and_Plate_MicroLabBox_student_DW.sf_MATLABFunction_g);

    /* Constant: '<S3>/reser_integrator' */
    Ball_and_Plate_MicroLabBox_student_B.reser_integrator =
      Ball_and_Plate_MicroLabBox_student_P.reser_integrator_Value;
  }
}

/* Model update function */
void Ball_and_Plate_MicroLabBox_student_update(void)
{
  if (rtmIsMajorTimeStep(Ball_and_Plate_MicroLabBox_student_M) &&
      Ball_and_Plate_MicroLabBox_student_M->Timing.TaskCounters.TID[2] == 0) {
    /* Update for UnitDelay: '<S1>/UD' */
    Ball_and_Plate_MicroLabBox_student_DW.UD_DSTATE =
      Ball_and_Plate_MicroLabBox_student_B.RateTransition3;
  }

  if (rtmIsMajorTimeStep(Ball_and_Plate_MicroLabBox_student_M)) {
    rt_ertODEUpdateContinuousStates
      (&Ball_and_Plate_MicroLabBox_student_M->solverInfo);
  }

  /* Update absolute time for base rate */
  /* The "clockTick0" counts the number of times the code of this task has
   * been executed. The absolute time is the multiplication of "clockTick0"
   * and "Timing.stepSize0". Size of "clockTick0" ensures timer will not
   * overflow during the application lifespan selected.
   * Timer of this task consists of two 32 bit unsigned integers.
   * The two integers represent the low bits Timing.clockTick0 and the high bits
   * Timing.clockTickH0. When the low bit overflows to 0, the high bits increment.
   */
  if (!(++Ball_and_Plate_MicroLabBox_student_M->Timing.clockTick0)) {
    ++Ball_and_Plate_MicroLabBox_student_M->Timing.clockTickH0;
  }

  Ball_and_Plate_MicroLabBox_student_M->Timing.t[0] = rtsiGetSolverStopTime
    (&Ball_and_Plate_MicroLabBox_student_M->solverInfo);

  {
    /* Update absolute timer for sample time: [0.001s, 0.0s] */
    /* The "clockTick1" counts the number of times the code of this task has
     * been executed. The absolute time is the multiplication of "clockTick1"
     * and "Timing.stepSize1". Size of "clockTick1" ensures timer will not
     * overflow during the application lifespan selected.
     * Timer of this task consists of two 32 bit unsigned integers.
     * The two integers represent the low bits Timing.clockTick1 and the high bits
     * Timing.clockTickH1. When the low bit overflows to 0, the high bits increment.
     */
    if (!(++Ball_and_Plate_MicroLabBox_student_M->Timing.clockTick1)) {
      ++Ball_and_Plate_MicroLabBox_student_M->Timing.clockTickH1;
    }

    Ball_and_Plate_MicroLabBox_student_M->Timing.t[1] =
      Ball_and_Plate_MicroLabBox_student_M->Timing.clockTick1 *
      Ball_and_Plate_MicroLabBox_student_M->Timing.stepSize1 +
      Ball_and_Plate_MicroLabBox_student_M->Timing.clockTickH1 *
      Ball_and_Plate_MicroLabBox_student_M->Timing.stepSize1 * 4294967296.0;
  }

  rate_scheduler();
}

/* Derivatives for root system: '<Root>' */
void Ball_and_Plate_MicroLabBox_student_derivatives(void)
{
  XDot_Ball_and_Plate_MicroLabBox_student_T *_rtXdot;
  _rtXdot = ((XDot_Ball_and_Plate_MicroLabBox_student_T *)
             Ball_and_Plate_MicroLabBox_student_M->derivs);

  /* Derivatives for Atomic SubSystem: '<S10>/Position Measurement' */
  /* Derivatives for TransferFcn: '<S25>/Transfer Fcn' */
  _rtXdot->TransferFcn_CSTATE_l = 0.0;
  _rtXdot->TransferFcn_CSTATE_l +=
    Ball_and_Plate_MicroLabBox_student_P.TransferFcn_A *
    Ball_and_Plate_MicroLabBox_student_X.TransferFcn_CSTATE_l;
  _rtXdot->TransferFcn_CSTATE_l +=
    Ball_and_Plate_MicroLabBox_student_B.EnabledSubsystem.OutportBufferForOut1;

  /* End of Derivatives for SubSystem: '<S10>/Position Measurement' */

  /* Derivatives for Atomic SubSystem: '<S11>/Position Measurement' */
  /* Derivatives for TransferFcn: '<S39>/Transfer Fcn' */
  _rtXdot->TransferFcn_CSTATE_h = 0.0;
  _rtXdot->TransferFcn_CSTATE_h +=
    Ball_and_Plate_MicroLabBox_student_P.TransferFcn_A_g *
    Ball_and_Plate_MicroLabBox_student_X.TransferFcn_CSTATE_h;
  _rtXdot->TransferFcn_CSTATE_h +=
    Ball_and_Plate_MicroLabBox_student_B.EnabledSubsystem_a.OutportBufferForOut1;

  /* End of Derivatives for SubSystem: '<S11>/Position Measurement' */

  /* Derivatives for Atomic SubSystem: '<S12>/Position Measurement' */
  /* Derivatives for TransferFcn: '<S55>/Transfer Fcn' */
  _rtXdot->TransferFcn_CSTATE = 0.0;
  _rtXdot->TransferFcn_CSTATE +=
    Ball_and_Plate_MicroLabBox_student_P.TransferFcn_A_m *
    Ball_and_Plate_MicroLabBox_student_X.TransferFcn_CSTATE;
  _rtXdot->TransferFcn_CSTATE +=
    Ball_and_Plate_MicroLabBox_student_B.EnabledSubsystem_n.OutportBufferForOut1;

  /* End of Derivatives for SubSystem: '<S12>/Position Measurement' */
}

/* Model initialize function */
void Ball_and_Plate_MicroLabBox_student_initialize(void)
{
  /* Registration code */

  /* initialize non-finites */
  rt_InitInfAndNaN(sizeof(real_T));

  /* initialize real-time model */
  (void) memset((void *)Ball_and_Plate_MicroLabBox_student_M, 0,
                sizeof(RT_MODEL_Ball_and_Plate_MicroLabBox_student_T));

  {
    /* Setup solver object */
    rtsiSetSimTimeStepPtr(&Ball_and_Plate_MicroLabBox_student_M->solverInfo,
                          &Ball_and_Plate_MicroLabBox_student_M->Timing.simTimeStep);
    rtsiSetTPtr(&Ball_and_Plate_MicroLabBox_student_M->solverInfo, &rtmGetTPtr
                (Ball_and_Plate_MicroLabBox_student_M));
    rtsiSetStepSizePtr(&Ball_and_Plate_MicroLabBox_student_M->solverInfo,
                       &Ball_and_Plate_MicroLabBox_student_M->Timing.stepSize0);
    rtsiSetdXPtr(&Ball_and_Plate_MicroLabBox_student_M->solverInfo,
                 &Ball_and_Plate_MicroLabBox_student_M->derivs);
    rtsiSetContStatesPtr(&Ball_and_Plate_MicroLabBox_student_M->solverInfo,
                         (real_T **)
                         &Ball_and_Plate_MicroLabBox_student_M->contStates);
    rtsiSetNumContStatesPtr(&Ball_and_Plate_MicroLabBox_student_M->solverInfo,
      &Ball_and_Plate_MicroLabBox_student_M->Sizes.numContStates);
    rtsiSetNumPeriodicContStatesPtr
      (&Ball_and_Plate_MicroLabBox_student_M->solverInfo,
       &Ball_and_Plate_MicroLabBox_student_M->Sizes.numPeriodicContStates);
    rtsiSetPeriodicContStateIndicesPtr
      (&Ball_and_Plate_MicroLabBox_student_M->solverInfo,
       &Ball_and_Plate_MicroLabBox_student_M->periodicContStateIndices);
    rtsiSetPeriodicContStateRangesPtr
      (&Ball_and_Plate_MicroLabBox_student_M->solverInfo,
       &Ball_and_Plate_MicroLabBox_student_M->periodicContStateRanges);
    rtsiSetErrorStatusPtr(&Ball_and_Plate_MicroLabBox_student_M->solverInfo,
                          (&rtmGetErrorStatus
      (Ball_and_Plate_MicroLabBox_student_M)));
    rtsiSetRTModelPtr(&Ball_and_Plate_MicroLabBox_student_M->solverInfo,
                      Ball_and_Plate_MicroLabBox_student_M);
  }

  rtsiSetSimTimeStep(&Ball_and_Plate_MicroLabBox_student_M->solverInfo,
                     MAJOR_TIME_STEP);
  Ball_and_Plate_MicroLabBox_student_M->intgData.f[0] =
    Ball_and_Plate_MicroLabBox_student_M->odeF[0];
  Ball_and_Plate_MicroLabBox_student_M->contStates =
    ((X_Ball_and_Plate_MicroLabBox_student_T *)
     &Ball_and_Plate_MicroLabBox_student_X);
  rtsiSetSolverData(&Ball_and_Plate_MicroLabBox_student_M->solverInfo, (void *)
                    &Ball_and_Plate_MicroLabBox_student_M->intgData);
  rtsiSetSolverName(&Ball_and_Plate_MicroLabBox_student_M->solverInfo,"ode1");
  Ball_and_Plate_MicroLabBox_student_M->solverInfoPtr =
    (&Ball_and_Plate_MicroLabBox_student_M->solverInfo);

  /* Initialize timing info */
  {
    int_T *mdlTsMap =
      Ball_and_Plate_MicroLabBox_student_M->Timing.sampleTimeTaskIDArray;
    mdlTsMap[0] = 0;
    mdlTsMap[1] = 1;
    mdlTsMap[2] = 2;
    Ball_and_Plate_MicroLabBox_student_M->Timing.sampleTimeTaskIDPtr =
      (&mdlTsMap[0]);
    Ball_and_Plate_MicroLabBox_student_M->Timing.sampleTimes =
      (&Ball_and_Plate_MicroLabBox_student_M->Timing.sampleTimesArray[0]);
    Ball_and_Plate_MicroLabBox_student_M->Timing.offsetTimes =
      (&Ball_and_Plate_MicroLabBox_student_M->Timing.offsetTimesArray[0]);

    /* task periods */
    Ball_and_Plate_MicroLabBox_student_M->Timing.sampleTimes[0] = (0.0);
    Ball_and_Plate_MicroLabBox_student_M->Timing.sampleTimes[1] = (0.001);
    Ball_and_Plate_MicroLabBox_student_M->Timing.sampleTimes[2] = (0.1);

    /* task offsets */
    Ball_and_Plate_MicroLabBox_student_M->Timing.offsetTimes[0] = (0.0);
    Ball_and_Plate_MicroLabBox_student_M->Timing.offsetTimes[1] = (0.0);
    Ball_and_Plate_MicroLabBox_student_M->Timing.offsetTimes[2] = (0.0);
  }

  rtmSetTPtr(Ball_and_Plate_MicroLabBox_student_M,
             &Ball_and_Plate_MicroLabBox_student_M->Timing.tArray[0]);

  {
    int_T *mdlSampleHits =
      Ball_and_Plate_MicroLabBox_student_M->Timing.sampleHitArray;
    mdlSampleHits[0] = 1;
    mdlSampleHits[1] = 1;
    mdlSampleHits[2] = 1;
    Ball_and_Plate_MicroLabBox_student_M->Timing.sampleHits = (&mdlSampleHits[0]);
  }

  rtmSetTFinal(Ball_and_Plate_MicroLabBox_student_M, -1);
  Ball_and_Plate_MicroLabBox_student_M->Timing.stepSize0 = 0.001;
  Ball_and_Plate_MicroLabBox_student_M->Timing.stepSize1 = 0.001;
  Ball_and_Plate_MicroLabBox_student_M->solverInfoPtr =
    (&Ball_and_Plate_MicroLabBox_student_M->solverInfo);
  Ball_and_Plate_MicroLabBox_student_M->Timing.stepSize = (0.001);
  rtsiSetFixedStepSize(&Ball_and_Plate_MicroLabBox_student_M->solverInfo, 0.001);
  rtsiSetSolverMode(&Ball_and_Plate_MicroLabBox_student_M->solverInfo,
                    SOLVER_MODE_SINGLETASKING);

  /* block I/O */
  (void) memset(((void *) &Ball_and_Plate_MicroLabBox_student_B), 0,
                sizeof(B_Ball_and_Plate_MicroLabBox_student_T));

  /* states (continuous) */
  {
    (void) memset((void *)&Ball_and_Plate_MicroLabBox_student_X, 0,
                  sizeof(X_Ball_and_Plate_MicroLabBox_student_T));
  }

  /* states (dwork) */
  (void) memset((void *)&Ball_and_Plate_MicroLabBox_student_DW, 0,
                sizeof(DW_Ball_and_Plate_MicroLabBox_student_T));

  /* external outputs */
  (void) memset((void *)&Ball_and_Plate_MicroLabBox_student_Y, 0,
                sizeof(ExtY_Ball_and_Plate_MicroLabBox_student_T));

  /* child S-Function registration */
  {
    RTWSfcnInfo *sfcnInfo =
      &Ball_and_Plate_MicroLabBox_student_M->NonInlinedSFcns.sfcnInfo;
    Ball_and_Plate_MicroLabBox_student_M->sfcnInfo = (sfcnInfo);
    rtssSetErrorStatusPtr(sfcnInfo, (&rtmGetErrorStatus
      (Ball_and_Plate_MicroLabBox_student_M)));
    rtssSetNumRootSampTimesPtr(sfcnInfo,
      &Ball_and_Plate_MicroLabBox_student_M->Sizes.numSampTimes);
    Ball_and_Plate_MicroLabBox_student_M->NonInlinedSFcns.taskTimePtrs[0] =
      &(rtmGetTPtr(Ball_and_Plate_MicroLabBox_student_M)[0]);
    Ball_and_Plate_MicroLabBox_student_M->NonInlinedSFcns.taskTimePtrs[1] =
      &(rtmGetTPtr(Ball_and_Plate_MicroLabBox_student_M)[1]);
    Ball_and_Plate_MicroLabBox_student_M->NonInlinedSFcns.taskTimePtrs[2] =
      &(rtmGetTPtr(Ball_and_Plate_MicroLabBox_student_M)[2]);
    rtssSetTPtrPtr(sfcnInfo,
                   Ball_and_Plate_MicroLabBox_student_M->NonInlinedSFcns.taskTimePtrs);
    rtssSetTStartPtr(sfcnInfo, &rtmGetTStart
                     (Ball_and_Plate_MicroLabBox_student_M));
    rtssSetTFinalPtr(sfcnInfo, &rtmGetTFinal
                     (Ball_and_Plate_MicroLabBox_student_M));
    rtssSetTimeOfLastOutputPtr(sfcnInfo, &rtmGetTimeOfLastOutput
      (Ball_and_Plate_MicroLabBox_student_M));
    rtssSetStepSizePtr(sfcnInfo,
                       &Ball_and_Plate_MicroLabBox_student_M->Timing.stepSize);
    rtssSetStopRequestedPtr(sfcnInfo, &rtmGetStopRequested
      (Ball_and_Plate_MicroLabBox_student_M));
    rtssSetDerivCacheNeedsResetPtr(sfcnInfo,
      &Ball_and_Plate_MicroLabBox_student_M->derivCacheNeedsReset);
    rtssSetZCCacheNeedsResetPtr(sfcnInfo,
      &Ball_and_Plate_MicroLabBox_student_M->zCCacheNeedsReset);
    rtssSetContTimeOutputInconsistentWithStateAtMajorStepPtr(sfcnInfo,
      &Ball_and_Plate_MicroLabBox_student_M->CTOutputIncnstWithState);
    rtssSetSampleHitsPtr(sfcnInfo,
                         &Ball_and_Plate_MicroLabBox_student_M->Timing.sampleHits);
    rtssSetPerTaskSampleHitsPtr(sfcnInfo,
      &Ball_and_Plate_MicroLabBox_student_M->Timing.perTaskSampleHits);
    rtssSetSimModePtr(sfcnInfo, &Ball_and_Plate_MicroLabBox_student_M->simMode);
    rtssSetSolverInfoPtr(sfcnInfo,
                         &Ball_and_Plate_MicroLabBox_student_M->solverInfoPtr);
  }

  Ball_and_Plate_MicroLabBox_student_M->Sizes.numSFcns = (6);

  /* register each child */
  {
    (void) memset((void *)
                  &Ball_and_Plate_MicroLabBox_student_M->NonInlinedSFcns.childSFunctions
                  [0], 0,
                  6*sizeof(SimStruct));
    Ball_and_Plate_MicroLabBox_student_M->childSfunctions =
      (&Ball_and_Plate_MicroLabBox_student_M->
       NonInlinedSFcns.childSFunctionPtrs[0]);

    {
      int_T i;
      for (i = 0; i < 6; i++) {
        Ball_and_Plate_MicroLabBox_student_M->childSfunctions[i] =
          (&Ball_and_Plate_MicroLabBox_student_M->NonInlinedSFcns.childSFunctions
           [i]);
      }
    }

    /* Level2 S-Function Block: Ball_and_Plate_MicroLabBox_student/<S13>/Dctleadlag2 (dleadlag) */
    {
      SimStruct *rts = Ball_and_Plate_MicroLabBox_student_M->childSfunctions[0];

      /* timing info */
      time_T *sfcnPeriod =
        Ball_and_Plate_MicroLabBox_student_M->NonInlinedSFcns.Sfcn0.sfcnPeriod;
      time_T *sfcnOffset =
        Ball_and_Plate_MicroLabBox_student_M->NonInlinedSFcns.Sfcn0.sfcnOffset;
      int_T *sfcnTsMap =
        Ball_and_Plate_MicroLabBox_student_M->NonInlinedSFcns.Sfcn0.sfcnTsMap;
      (void) memset((void*)sfcnPeriod, 0,
                    sizeof(time_T)*1);
      (void) memset((void*)sfcnOffset, 0,
                    sizeof(time_T)*1);
      ssSetSampleTimePtr(rts, &sfcnPeriod[0]);
      ssSetOffsetTimePtr(rts, &sfcnOffset[0]);
      ssSetSampleTimeTaskIDPtr(rts, sfcnTsMap);

      {
        ssSetBlkInfo2Ptr(rts,
                         &Ball_and_Plate_MicroLabBox_student_M->NonInlinedSFcns.blkInfo2
                         [0]);
      }

      _ssSetBlkInfo2PortInfo2Ptr(rts,
        &Ball_and_Plate_MicroLabBox_student_M->NonInlinedSFcns.inputOutputPortInfo2
        [0]);

      /* Set up the mdlInfo pointer */
      ssSetRTWSfcnInfo(rts, Ball_and_Plate_MicroLabBox_student_M->sfcnInfo);

      /* Allocate memory of model methods 2 */
      {
        ssSetModelMethods2(rts,
                           &Ball_and_Plate_MicroLabBox_student_M->NonInlinedSFcns.methods2
                           [0]);
      }

      /* Allocate memory of model methods 3 */
      {
        ssSetModelMethods3(rts,
                           &Ball_and_Plate_MicroLabBox_student_M->NonInlinedSFcns.methods3
                           [0]);
      }

      /* Allocate memory of model methods 4 */
      {
        ssSetModelMethods4(rts,
                           &Ball_and_Plate_MicroLabBox_student_M->NonInlinedSFcns.methods4
                           [0]);
      }

      /* Allocate memory for states auxilliary information */
      {
        ssSetStatesInfo2(rts,
                         &Ball_and_Plate_MicroLabBox_student_M->NonInlinedSFcns.statesInfo2
                         [0]);
        ssSetPeriodicStatesInfo(rts,
          &Ball_and_Plate_MicroLabBox_student_M->NonInlinedSFcns.periodicStatesInfo
          [0]);
      }

      /* inputs */
      {
        _ssSetNumInputPorts(rts, 1);
        ssSetPortInfoForInputs(rts,
          &Ball_and_Plate_MicroLabBox_student_M->NonInlinedSFcns.Sfcn0.inputPortInfo
          [0]);
        _ssSetPortInfo2ForInputUnits(rts,
          &Ball_and_Plate_MicroLabBox_student_M->NonInlinedSFcns.Sfcn0.inputPortUnits
          [0]);
        ssSetInputPortUnit(rts, 0, 0);
        _ssSetPortInfo2ForInputCoSimAttribute(rts,
          &Ball_and_Plate_MicroLabBox_student_M->NonInlinedSFcns.Sfcn0.inputPortCoSimAttribute
          [0]);
        ssSetInputPortIsContinuousQuantity(rts, 0, 0);

        /* port 0 */
        {
          real_T const **sfcnUPtrs = (real_T const **)
            &Ball_and_Plate_MicroLabBox_student_M->NonInlinedSFcns.Sfcn0.UPtrs0;
          sfcnUPtrs[0] = &Ball_and_Plate_MicroLabBox_student_B.Gain1;
          ssSetInputPortSignalPtrs(rts, 0, (InputPtrsType)&sfcnUPtrs[0]);
          _ssSetInputPortNumDimensions(rts, 0, 1);
          ssSetInputPortWidth(rts, 0, 1);
        }
      }

      /* outputs */
      {
        ssSetPortInfoForOutputs(rts,
          &Ball_and_Plate_MicroLabBox_student_M->NonInlinedSFcns.Sfcn0.outputPortInfo
          [0]);
        _ssSetNumOutputPorts(rts, 1);
        _ssSetPortInfo2ForOutputUnits(rts,
          &Ball_and_Plate_MicroLabBox_student_M->NonInlinedSFcns.Sfcn0.outputPortUnits
          [0]);
        ssSetOutputPortUnit(rts, 0, 0);
        _ssSetPortInfo2ForOutputCoSimAttribute(rts,
          &Ball_and_Plate_MicroLabBox_student_M->NonInlinedSFcns.Sfcn0.outputPortCoSimAttribute
          [0]);
        ssSetOutputPortIsContinuousQuantity(rts, 0, 0);

        /* port 0 */
        {
          _ssSetOutputPortNumDimensions(rts, 0, 1);
          ssSetOutputPortWidth(rts, 0, 1);
          ssSetOutputPortSignal(rts, 0, ((real_T *)
            &Ball_and_Plate_MicroLabBox_student_B.Dctleadlag2));
        }
      }

      /* path info */
      ssSetModelName(rts, "Dctleadlag2");
      ssSetPath(rts,
                "Ball_and_Plate_MicroLabBox_student/Innerloop_Actuator/Motor_A/Controller_10_05/Dctleadlag2");
      ssSetRTModel(rts,Ball_and_Plate_MicroLabBox_student_M);
      ssSetParentSS(rts, (NULL));
      ssSetRootSS(rts, rts);
      ssSetVersion(rts, SIMSTRUCT_VERSION_LEVEL2);

      /* parameters */
      {
        mxArray **sfcnParams = (mxArray **)
          &Ball_and_Plate_MicroLabBox_student_M->NonInlinedSFcns.Sfcn0.params;
        ssSetSFcnParamsCount(rts, 3);
        ssSetSFcnParamsPtr(rts, &sfcnParams[0]);
        ssSetSFcnParam(rts, 0, (mxArray*)
                       Ball_and_Plate_MicroLabBox_student_P.Dctleadlag2_P1_Size);
        ssSetSFcnParam(rts, 1, (mxArray*)
                       Ball_and_Plate_MicroLabBox_student_P.Dctleadlag2_P2_Size);
        ssSetSFcnParam(rts, 2, (mxArray*)
                       Ball_and_Plate_MicroLabBox_student_P.Dctleadlag2_P3_Size);
      }

      /* work vectors */
      ssSetRWork(rts, (real_T *)
                 &Ball_and_Plate_MicroLabBox_student_DW.Dctleadlag2_RWORK[0]);

      {
        struct _ssDWorkRecord *dWorkRecord = (struct _ssDWorkRecord *)
          &Ball_and_Plate_MicroLabBox_student_M->NonInlinedSFcns.Sfcn0.dWork;
        struct _ssDWorkAuxRecord *dWorkAuxRecord = (struct _ssDWorkAuxRecord *)
          &Ball_and_Plate_MicroLabBox_student_M->NonInlinedSFcns.Sfcn0.dWorkAux;
        ssSetSFcnDWork(rts, dWorkRecord);
        ssSetSFcnDWorkAux(rts, dWorkAuxRecord);
        _ssSetNumDWork(rts, 1);

        /* RWORK */
        ssSetDWorkWidth(rts, 0, 2);
        ssSetDWorkDataType(rts, 0,SS_DOUBLE);
        ssSetDWorkComplexSignal(rts, 0, 0);
        ssSetDWork(rts, 0,
                   &Ball_and_Plate_MicroLabBox_student_DW.Dctleadlag2_RWORK[0]);
      }

      /* registration */
      dleadlag(rts);
      sfcnInitializeSizes(rts);
      sfcnInitializeSampleTimes(rts);

      /* adjust sample time */
      ssSetSampleTime(rts, 0, 0.001);
      ssSetOffsetTime(rts, 0, 0.0);
      sfcnTsMap[0] = 1;

      /* set compiled values of dynamic vector attributes */
      ssSetNumNonsampledZCs(rts, 0);

      /* Update connectivity flags for each port */
      _ssSetInputPortConnected(rts, 0, 1);
      _ssSetOutputPortConnected(rts, 0, 1);
      _ssSetOutputPortBeingMerged(rts, 0, 0);

      /* Update the BufferDstPort flags for each input port */
      ssSetInputPortBufferDstPort(rts, 0, -1);
    }

    /* Level2 S-Function Block: Ball_and_Plate_MicroLabBox_student/<S13>/Dct1lowpass3 (dlowpass1) */
    {
      SimStruct *rts = Ball_and_Plate_MicroLabBox_student_M->childSfunctions[1];

      /* timing info */
      time_T *sfcnPeriod =
        Ball_and_Plate_MicroLabBox_student_M->NonInlinedSFcns.Sfcn1.sfcnPeriod;
      time_T *sfcnOffset =
        Ball_and_Plate_MicroLabBox_student_M->NonInlinedSFcns.Sfcn1.sfcnOffset;
      int_T *sfcnTsMap =
        Ball_and_Plate_MicroLabBox_student_M->NonInlinedSFcns.Sfcn1.sfcnTsMap;
      (void) memset((void*)sfcnPeriod, 0,
                    sizeof(time_T)*1);
      (void) memset((void*)sfcnOffset, 0,
                    sizeof(time_T)*1);
      ssSetSampleTimePtr(rts, &sfcnPeriod[0]);
      ssSetOffsetTimePtr(rts, &sfcnOffset[0]);
      ssSetSampleTimeTaskIDPtr(rts, sfcnTsMap);

      {
        ssSetBlkInfo2Ptr(rts,
                         &Ball_and_Plate_MicroLabBox_student_M->NonInlinedSFcns.blkInfo2
                         [1]);
      }

      _ssSetBlkInfo2PortInfo2Ptr(rts,
        &Ball_and_Plate_MicroLabBox_student_M->NonInlinedSFcns.inputOutputPortInfo2
        [1]);

      /* Set up the mdlInfo pointer */
      ssSetRTWSfcnInfo(rts, Ball_and_Plate_MicroLabBox_student_M->sfcnInfo);

      /* Allocate memory of model methods 2 */
      {
        ssSetModelMethods2(rts,
                           &Ball_and_Plate_MicroLabBox_student_M->NonInlinedSFcns.methods2
                           [1]);
      }

      /* Allocate memory of model methods 3 */
      {
        ssSetModelMethods3(rts,
                           &Ball_and_Plate_MicroLabBox_student_M->NonInlinedSFcns.methods3
                           [1]);
      }

      /* Allocate memory of model methods 4 */
      {
        ssSetModelMethods4(rts,
                           &Ball_and_Plate_MicroLabBox_student_M->NonInlinedSFcns.methods4
                           [1]);
      }

      /* Allocate memory for states auxilliary information */
      {
        ssSetStatesInfo2(rts,
                         &Ball_and_Plate_MicroLabBox_student_M->NonInlinedSFcns.statesInfo2
                         [1]);
        ssSetPeriodicStatesInfo(rts,
          &Ball_and_Plate_MicroLabBox_student_M->NonInlinedSFcns.periodicStatesInfo
          [1]);
      }

      /* inputs */
      {
        _ssSetNumInputPorts(rts, 1);
        ssSetPortInfoForInputs(rts,
          &Ball_and_Plate_MicroLabBox_student_M->NonInlinedSFcns.Sfcn1.inputPortInfo
          [0]);
        _ssSetPortInfo2ForInputUnits(rts,
          &Ball_and_Plate_MicroLabBox_student_M->NonInlinedSFcns.Sfcn1.inputPortUnits
          [0]);
        ssSetInputPortUnit(rts, 0, 0);
        _ssSetPortInfo2ForInputCoSimAttribute(rts,
          &Ball_and_Plate_MicroLabBox_student_M->NonInlinedSFcns.Sfcn1.inputPortCoSimAttribute
          [0]);
        ssSetInputPortIsContinuousQuantity(rts, 0, 0);

        /* port 0 */
        {
          real_T const **sfcnUPtrs = (real_T const **)
            &Ball_and_Plate_MicroLabBox_student_M->NonInlinedSFcns.Sfcn1.UPtrs0;
          sfcnUPtrs[0] = &Ball_and_Plate_MicroLabBox_student_B.Dctleadlag2;
          ssSetInputPortSignalPtrs(rts, 0, (InputPtrsType)&sfcnUPtrs[0]);
          _ssSetInputPortNumDimensions(rts, 0, 1);
          ssSetInputPortWidth(rts, 0, 1);
        }
      }

      /* outputs */
      {
        ssSetPortInfoForOutputs(rts,
          &Ball_and_Plate_MicroLabBox_student_M->NonInlinedSFcns.Sfcn1.outputPortInfo
          [0]);
        _ssSetNumOutputPorts(rts, 1);
        _ssSetPortInfo2ForOutputUnits(rts,
          &Ball_and_Plate_MicroLabBox_student_M->NonInlinedSFcns.Sfcn1.outputPortUnits
          [0]);
        ssSetOutputPortUnit(rts, 0, 0);
        _ssSetPortInfo2ForOutputCoSimAttribute(rts,
          &Ball_and_Plate_MicroLabBox_student_M->NonInlinedSFcns.Sfcn1.outputPortCoSimAttribute
          [0]);
        ssSetOutputPortIsContinuousQuantity(rts, 0, 0);

        /* port 0 */
        {
          _ssSetOutputPortNumDimensions(rts, 0, 1);
          ssSetOutputPortWidth(rts, 0, 1);
          ssSetOutputPortSignal(rts, 0, ((real_T *)
            &Ball_and_Plate_MicroLabBox_student_B.Dct1lowpass3));
        }
      }

      /* path info */
      ssSetModelName(rts, "Dct1lowpass3");
      ssSetPath(rts,
                "Ball_and_Plate_MicroLabBox_student/Innerloop_Actuator/Motor_A/Controller_10_05/Dct1lowpass3");
      ssSetRTModel(rts,Ball_and_Plate_MicroLabBox_student_M);
      ssSetParentSS(rts, (NULL));
      ssSetRootSS(rts, rts);
      ssSetVersion(rts, SIMSTRUCT_VERSION_LEVEL2);

      /* parameters */
      {
        mxArray **sfcnParams = (mxArray **)
          &Ball_and_Plate_MicroLabBox_student_M->NonInlinedSFcns.Sfcn1.params;
        ssSetSFcnParamsCount(rts, 2);
        ssSetSFcnParamsPtr(rts, &sfcnParams[0]);
        ssSetSFcnParam(rts, 0, (mxArray*)
                       Ball_and_Plate_MicroLabBox_student_P.Dct1lowpass3_P1_Size);
        ssSetSFcnParam(rts, 1, (mxArray*)
                       Ball_and_Plate_MicroLabBox_student_P.Dct1lowpass3_P2_Size);
      }

      /* work vectors */
      ssSetRWork(rts, (real_T *)
                 &Ball_and_Plate_MicroLabBox_student_DW.Dct1lowpass3_RWORK[0]);

      {
        struct _ssDWorkRecord *dWorkRecord = (struct _ssDWorkRecord *)
          &Ball_and_Plate_MicroLabBox_student_M->NonInlinedSFcns.Sfcn1.dWork;
        struct _ssDWorkAuxRecord *dWorkAuxRecord = (struct _ssDWorkAuxRecord *)
          &Ball_and_Plate_MicroLabBox_student_M->NonInlinedSFcns.Sfcn1.dWorkAux;
        ssSetSFcnDWork(rts, dWorkRecord);
        ssSetSFcnDWorkAux(rts, dWorkAuxRecord);
        _ssSetNumDWork(rts, 1);

        /* RWORK */
        ssSetDWorkWidth(rts, 0, 2);
        ssSetDWorkDataType(rts, 0,SS_DOUBLE);
        ssSetDWorkComplexSignal(rts, 0, 0);
        ssSetDWork(rts, 0,
                   &Ball_and_Plate_MicroLabBox_student_DW.Dct1lowpass3_RWORK[0]);
      }

      /* registration */
      dlowpass1(rts);
      sfcnInitializeSizes(rts);
      sfcnInitializeSampleTimes(rts);

      /* adjust sample time */
      ssSetSampleTime(rts, 0, 0.001);
      ssSetOffsetTime(rts, 0, 0.0);
      sfcnTsMap[0] = 1;

      /* set compiled values of dynamic vector attributes */
      ssSetNumNonsampledZCs(rts, 0);

      /* Update connectivity flags for each port */
      _ssSetInputPortConnected(rts, 0, 1);
      _ssSetOutputPortConnected(rts, 0, 1);
      _ssSetOutputPortBeingMerged(rts, 0, 0);

      /* Update the BufferDstPort flags for each input port */
      ssSetInputPortBufferDstPort(rts, 0, -1);
    }

    /* Level2 S-Function Block: Ball_and_Plate_MicroLabBox_student/<S28>/Dctleadlag2 (dleadlag) */
    {
      SimStruct *rts = Ball_and_Plate_MicroLabBox_student_M->childSfunctions[2];

      /* timing info */
      time_T *sfcnPeriod =
        Ball_and_Plate_MicroLabBox_student_M->NonInlinedSFcns.Sfcn2.sfcnPeriod;
      time_T *sfcnOffset =
        Ball_and_Plate_MicroLabBox_student_M->NonInlinedSFcns.Sfcn2.sfcnOffset;
      int_T *sfcnTsMap =
        Ball_and_Plate_MicroLabBox_student_M->NonInlinedSFcns.Sfcn2.sfcnTsMap;
      (void) memset((void*)sfcnPeriod, 0,
                    sizeof(time_T)*1);
      (void) memset((void*)sfcnOffset, 0,
                    sizeof(time_T)*1);
      ssSetSampleTimePtr(rts, &sfcnPeriod[0]);
      ssSetOffsetTimePtr(rts, &sfcnOffset[0]);
      ssSetSampleTimeTaskIDPtr(rts, sfcnTsMap);

      {
        ssSetBlkInfo2Ptr(rts,
                         &Ball_and_Plate_MicroLabBox_student_M->NonInlinedSFcns.blkInfo2
                         [2]);
      }

      _ssSetBlkInfo2PortInfo2Ptr(rts,
        &Ball_and_Plate_MicroLabBox_student_M->NonInlinedSFcns.inputOutputPortInfo2
        [2]);

      /* Set up the mdlInfo pointer */
      ssSetRTWSfcnInfo(rts, Ball_and_Plate_MicroLabBox_student_M->sfcnInfo);

      /* Allocate memory of model methods 2 */
      {
        ssSetModelMethods2(rts,
                           &Ball_and_Plate_MicroLabBox_student_M->NonInlinedSFcns.methods2
                           [2]);
      }

      /* Allocate memory of model methods 3 */
      {
        ssSetModelMethods3(rts,
                           &Ball_and_Plate_MicroLabBox_student_M->NonInlinedSFcns.methods3
                           [2]);
      }

      /* Allocate memory of model methods 4 */
      {
        ssSetModelMethods4(rts,
                           &Ball_and_Plate_MicroLabBox_student_M->NonInlinedSFcns.methods4
                           [2]);
      }

      /* Allocate memory for states auxilliary information */
      {
        ssSetStatesInfo2(rts,
                         &Ball_and_Plate_MicroLabBox_student_M->NonInlinedSFcns.statesInfo2
                         [2]);
        ssSetPeriodicStatesInfo(rts,
          &Ball_and_Plate_MicroLabBox_student_M->NonInlinedSFcns.periodicStatesInfo
          [2]);
      }

      /* inputs */
      {
        _ssSetNumInputPorts(rts, 1);
        ssSetPortInfoForInputs(rts,
          &Ball_and_Plate_MicroLabBox_student_M->NonInlinedSFcns.Sfcn2.inputPortInfo
          [0]);
        _ssSetPortInfo2ForInputUnits(rts,
          &Ball_and_Plate_MicroLabBox_student_M->NonInlinedSFcns.Sfcn2.inputPortUnits
          [0]);
        ssSetInputPortUnit(rts, 0, 0);
        _ssSetPortInfo2ForInputCoSimAttribute(rts,
          &Ball_and_Plate_MicroLabBox_student_M->NonInlinedSFcns.Sfcn2.inputPortCoSimAttribute
          [0]);
        ssSetInputPortIsContinuousQuantity(rts, 0, 0);

        /* port 0 */
        {
          real_T const **sfcnUPtrs = (real_T const **)
            &Ball_and_Plate_MicroLabBox_student_M->NonInlinedSFcns.Sfcn2.UPtrs0;
          sfcnUPtrs[0] = &Ball_and_Plate_MicroLabBox_student_B.Gain1_o;
          ssSetInputPortSignalPtrs(rts, 0, (InputPtrsType)&sfcnUPtrs[0]);
          _ssSetInputPortNumDimensions(rts, 0, 1);
          ssSetInputPortWidth(rts, 0, 1);
        }
      }

      /* outputs */
      {
        ssSetPortInfoForOutputs(rts,
          &Ball_and_Plate_MicroLabBox_student_M->NonInlinedSFcns.Sfcn2.outputPortInfo
          [0]);
        _ssSetNumOutputPorts(rts, 1);
        _ssSetPortInfo2ForOutputUnits(rts,
          &Ball_and_Plate_MicroLabBox_student_M->NonInlinedSFcns.Sfcn2.outputPortUnits
          [0]);
        ssSetOutputPortUnit(rts, 0, 0);
        _ssSetPortInfo2ForOutputCoSimAttribute(rts,
          &Ball_and_Plate_MicroLabBox_student_M->NonInlinedSFcns.Sfcn2.outputPortCoSimAttribute
          [0]);
        ssSetOutputPortIsContinuousQuantity(rts, 0, 0);

        /* port 0 */
        {
          _ssSetOutputPortNumDimensions(rts, 0, 1);
          ssSetOutputPortWidth(rts, 0, 1);
          ssSetOutputPortSignal(rts, 0, ((real_T *)
            &Ball_and_Plate_MicroLabBox_student_B.Dctleadlag2_g));
        }
      }

      /* path info */
      ssSetModelName(rts, "Dctleadlag2");
      ssSetPath(rts,
                "Ball_and_Plate_MicroLabBox_student/Innerloop_Actuator/Motor_B/Controller_10_05/Dctleadlag2");
      ssSetRTModel(rts,Ball_and_Plate_MicroLabBox_student_M);
      ssSetParentSS(rts, (NULL));
      ssSetRootSS(rts, rts);
      ssSetVersion(rts, SIMSTRUCT_VERSION_LEVEL2);

      /* parameters */
      {
        mxArray **sfcnParams = (mxArray **)
          &Ball_and_Plate_MicroLabBox_student_M->NonInlinedSFcns.Sfcn2.params;
        ssSetSFcnParamsCount(rts, 3);
        ssSetSFcnParamsPtr(rts, &sfcnParams[0]);
        ssSetSFcnParam(rts, 0, (mxArray*)
                       Ball_and_Plate_MicroLabBox_student_P.Dctleadlag2_P1_Size_i);
        ssSetSFcnParam(rts, 1, (mxArray*)
                       Ball_and_Plate_MicroLabBox_student_P.Dctleadlag2_P2_Size_f);
        ssSetSFcnParam(rts, 2, (mxArray*)
                       Ball_and_Plate_MicroLabBox_student_P.Dctleadlag2_P3_Size_g);
      }

      /* work vectors */
      ssSetRWork(rts, (real_T *)
                 &Ball_and_Plate_MicroLabBox_student_DW.Dctleadlag2_RWORK_k[0]);

      {
        struct _ssDWorkRecord *dWorkRecord = (struct _ssDWorkRecord *)
          &Ball_and_Plate_MicroLabBox_student_M->NonInlinedSFcns.Sfcn2.dWork;
        struct _ssDWorkAuxRecord *dWorkAuxRecord = (struct _ssDWorkAuxRecord *)
          &Ball_and_Plate_MicroLabBox_student_M->NonInlinedSFcns.Sfcn2.dWorkAux;
        ssSetSFcnDWork(rts, dWorkRecord);
        ssSetSFcnDWorkAux(rts, dWorkAuxRecord);
        _ssSetNumDWork(rts, 1);

        /* RWORK */
        ssSetDWorkWidth(rts, 0, 2);
        ssSetDWorkDataType(rts, 0,SS_DOUBLE);
        ssSetDWorkComplexSignal(rts, 0, 0);
        ssSetDWork(rts, 0,
                   &Ball_and_Plate_MicroLabBox_student_DW.Dctleadlag2_RWORK_k[0]);
      }

      /* registration */
      dleadlag(rts);
      sfcnInitializeSizes(rts);
      sfcnInitializeSampleTimes(rts);

      /* adjust sample time */
      ssSetSampleTime(rts, 0, 0.001);
      ssSetOffsetTime(rts, 0, 0.0);
      sfcnTsMap[0] = 1;

      /* set compiled values of dynamic vector attributes */
      ssSetNumNonsampledZCs(rts, 0);

      /* Update connectivity flags for each port */
      _ssSetInputPortConnected(rts, 0, 1);
      _ssSetOutputPortConnected(rts, 0, 1);
      _ssSetOutputPortBeingMerged(rts, 0, 0);

      /* Update the BufferDstPort flags for each input port */
      ssSetInputPortBufferDstPort(rts, 0, -1);
    }

    /* Level2 S-Function Block: Ball_and_Plate_MicroLabBox_student/<S28>/Dct1lowpass3 (dlowpass1) */
    {
      SimStruct *rts = Ball_and_Plate_MicroLabBox_student_M->childSfunctions[3];

      /* timing info */
      time_T *sfcnPeriod =
        Ball_and_Plate_MicroLabBox_student_M->NonInlinedSFcns.Sfcn3.sfcnPeriod;
      time_T *sfcnOffset =
        Ball_and_Plate_MicroLabBox_student_M->NonInlinedSFcns.Sfcn3.sfcnOffset;
      int_T *sfcnTsMap =
        Ball_and_Plate_MicroLabBox_student_M->NonInlinedSFcns.Sfcn3.sfcnTsMap;
      (void) memset((void*)sfcnPeriod, 0,
                    sizeof(time_T)*1);
      (void) memset((void*)sfcnOffset, 0,
                    sizeof(time_T)*1);
      ssSetSampleTimePtr(rts, &sfcnPeriod[0]);
      ssSetOffsetTimePtr(rts, &sfcnOffset[0]);
      ssSetSampleTimeTaskIDPtr(rts, sfcnTsMap);

      {
        ssSetBlkInfo2Ptr(rts,
                         &Ball_and_Plate_MicroLabBox_student_M->NonInlinedSFcns.blkInfo2
                         [3]);
      }

      _ssSetBlkInfo2PortInfo2Ptr(rts,
        &Ball_and_Plate_MicroLabBox_student_M->NonInlinedSFcns.inputOutputPortInfo2
        [3]);

      /* Set up the mdlInfo pointer */
      ssSetRTWSfcnInfo(rts, Ball_and_Plate_MicroLabBox_student_M->sfcnInfo);

      /* Allocate memory of model methods 2 */
      {
        ssSetModelMethods2(rts,
                           &Ball_and_Plate_MicroLabBox_student_M->NonInlinedSFcns.methods2
                           [3]);
      }

      /* Allocate memory of model methods 3 */
      {
        ssSetModelMethods3(rts,
                           &Ball_and_Plate_MicroLabBox_student_M->NonInlinedSFcns.methods3
                           [3]);
      }

      /* Allocate memory of model methods 4 */
      {
        ssSetModelMethods4(rts,
                           &Ball_and_Plate_MicroLabBox_student_M->NonInlinedSFcns.methods4
                           [3]);
      }

      /* Allocate memory for states auxilliary information */
      {
        ssSetStatesInfo2(rts,
                         &Ball_and_Plate_MicroLabBox_student_M->NonInlinedSFcns.statesInfo2
                         [3]);
        ssSetPeriodicStatesInfo(rts,
          &Ball_and_Plate_MicroLabBox_student_M->NonInlinedSFcns.periodicStatesInfo
          [3]);
      }

      /* inputs */
      {
        _ssSetNumInputPorts(rts, 1);
        ssSetPortInfoForInputs(rts,
          &Ball_and_Plate_MicroLabBox_student_M->NonInlinedSFcns.Sfcn3.inputPortInfo
          [0]);
        _ssSetPortInfo2ForInputUnits(rts,
          &Ball_and_Plate_MicroLabBox_student_M->NonInlinedSFcns.Sfcn3.inputPortUnits
          [0]);
        ssSetInputPortUnit(rts, 0, 0);
        _ssSetPortInfo2ForInputCoSimAttribute(rts,
          &Ball_and_Plate_MicroLabBox_student_M->NonInlinedSFcns.Sfcn3.inputPortCoSimAttribute
          [0]);
        ssSetInputPortIsContinuousQuantity(rts, 0, 0);

        /* port 0 */
        {
          real_T const **sfcnUPtrs = (real_T const **)
            &Ball_and_Plate_MicroLabBox_student_M->NonInlinedSFcns.Sfcn3.UPtrs0;
          sfcnUPtrs[0] = &Ball_and_Plate_MicroLabBox_student_B.Dctleadlag2_g;
          ssSetInputPortSignalPtrs(rts, 0, (InputPtrsType)&sfcnUPtrs[0]);
          _ssSetInputPortNumDimensions(rts, 0, 1);
          ssSetInputPortWidth(rts, 0, 1);
        }
      }

      /* outputs */
      {
        ssSetPortInfoForOutputs(rts,
          &Ball_and_Plate_MicroLabBox_student_M->NonInlinedSFcns.Sfcn3.outputPortInfo
          [0]);
        _ssSetNumOutputPorts(rts, 1);
        _ssSetPortInfo2ForOutputUnits(rts,
          &Ball_and_Plate_MicroLabBox_student_M->NonInlinedSFcns.Sfcn3.outputPortUnits
          [0]);
        ssSetOutputPortUnit(rts, 0, 0);
        _ssSetPortInfo2ForOutputCoSimAttribute(rts,
          &Ball_and_Plate_MicroLabBox_student_M->NonInlinedSFcns.Sfcn3.outputPortCoSimAttribute
          [0]);
        ssSetOutputPortIsContinuousQuantity(rts, 0, 0);

        /* port 0 */
        {
          _ssSetOutputPortNumDimensions(rts, 0, 1);
          ssSetOutputPortWidth(rts, 0, 1);
          ssSetOutputPortSignal(rts, 0, ((real_T *)
            &Ball_and_Plate_MicroLabBox_student_B.Dct1lowpass3_c));
        }
      }

      /* path info */
      ssSetModelName(rts, "Dct1lowpass3");
      ssSetPath(rts,
                "Ball_and_Plate_MicroLabBox_student/Innerloop_Actuator/Motor_B/Controller_10_05/Dct1lowpass3");
      ssSetRTModel(rts,Ball_and_Plate_MicroLabBox_student_M);
      ssSetParentSS(rts, (NULL));
      ssSetRootSS(rts, rts);
      ssSetVersion(rts, SIMSTRUCT_VERSION_LEVEL2);

      /* parameters */
      {
        mxArray **sfcnParams = (mxArray **)
          &Ball_and_Plate_MicroLabBox_student_M->NonInlinedSFcns.Sfcn3.params;
        ssSetSFcnParamsCount(rts, 2);
        ssSetSFcnParamsPtr(rts, &sfcnParams[0]);
        ssSetSFcnParam(rts, 0, (mxArray*)
                       Ball_and_Plate_MicroLabBox_student_P.Dct1lowpass3_P1_Size_e);
        ssSetSFcnParam(rts, 1, (mxArray*)
                       Ball_and_Plate_MicroLabBox_student_P.Dct1lowpass3_P2_Size_p);
      }

      /* work vectors */
      ssSetRWork(rts, (real_T *)
                 &Ball_and_Plate_MicroLabBox_student_DW.Dct1lowpass3_RWORK_c[0]);

      {
        struct _ssDWorkRecord *dWorkRecord = (struct _ssDWorkRecord *)
          &Ball_and_Plate_MicroLabBox_student_M->NonInlinedSFcns.Sfcn3.dWork;
        struct _ssDWorkAuxRecord *dWorkAuxRecord = (struct _ssDWorkAuxRecord *)
          &Ball_and_Plate_MicroLabBox_student_M->NonInlinedSFcns.Sfcn3.dWorkAux;
        ssSetSFcnDWork(rts, dWorkRecord);
        ssSetSFcnDWorkAux(rts, dWorkAuxRecord);
        _ssSetNumDWork(rts, 1);

        /* RWORK */
        ssSetDWorkWidth(rts, 0, 2);
        ssSetDWorkDataType(rts, 0,SS_DOUBLE);
        ssSetDWorkComplexSignal(rts, 0, 0);
        ssSetDWork(rts, 0,
                   &Ball_and_Plate_MicroLabBox_student_DW.Dct1lowpass3_RWORK_c[0]);
      }

      /* registration */
      dlowpass1(rts);
      sfcnInitializeSizes(rts);
      sfcnInitializeSampleTimes(rts);

      /* adjust sample time */
      ssSetSampleTime(rts, 0, 0.001);
      ssSetOffsetTime(rts, 0, 0.0);
      sfcnTsMap[0] = 1;

      /* set compiled values of dynamic vector attributes */
      ssSetNumNonsampledZCs(rts, 0);

      /* Update connectivity flags for each port */
      _ssSetInputPortConnected(rts, 0, 1);
      _ssSetOutputPortConnected(rts, 0, 1);
      _ssSetOutputPortBeingMerged(rts, 0, 0);

      /* Update the BufferDstPort flags for each input port */
      ssSetInputPortBufferDstPort(rts, 0, -1);
    }

    /* Level2 S-Function Block: Ball_and_Plate_MicroLabBox_student/<S42>/Dctleadlag2 (dleadlag) */
    {
      SimStruct *rts = Ball_and_Plate_MicroLabBox_student_M->childSfunctions[4];

      /* timing info */
      time_T *sfcnPeriod =
        Ball_and_Plate_MicroLabBox_student_M->NonInlinedSFcns.Sfcn4.sfcnPeriod;
      time_T *sfcnOffset =
        Ball_and_Plate_MicroLabBox_student_M->NonInlinedSFcns.Sfcn4.sfcnOffset;
      int_T *sfcnTsMap =
        Ball_and_Plate_MicroLabBox_student_M->NonInlinedSFcns.Sfcn4.sfcnTsMap;
      (void) memset((void*)sfcnPeriod, 0,
                    sizeof(time_T)*1);
      (void) memset((void*)sfcnOffset, 0,
                    sizeof(time_T)*1);
      ssSetSampleTimePtr(rts, &sfcnPeriod[0]);
      ssSetOffsetTimePtr(rts, &sfcnOffset[0]);
      ssSetSampleTimeTaskIDPtr(rts, sfcnTsMap);

      {
        ssSetBlkInfo2Ptr(rts,
                         &Ball_and_Plate_MicroLabBox_student_M->NonInlinedSFcns.blkInfo2
                         [4]);
      }

      _ssSetBlkInfo2PortInfo2Ptr(rts,
        &Ball_and_Plate_MicroLabBox_student_M->NonInlinedSFcns.inputOutputPortInfo2
        [4]);

      /* Set up the mdlInfo pointer */
      ssSetRTWSfcnInfo(rts, Ball_and_Plate_MicroLabBox_student_M->sfcnInfo);

      /* Allocate memory of model methods 2 */
      {
        ssSetModelMethods2(rts,
                           &Ball_and_Plate_MicroLabBox_student_M->NonInlinedSFcns.methods2
                           [4]);
      }

      /* Allocate memory of model methods 3 */
      {
        ssSetModelMethods3(rts,
                           &Ball_and_Plate_MicroLabBox_student_M->NonInlinedSFcns.methods3
                           [4]);
      }

      /* Allocate memory of model methods 4 */
      {
        ssSetModelMethods4(rts,
                           &Ball_and_Plate_MicroLabBox_student_M->NonInlinedSFcns.methods4
                           [4]);
      }

      /* Allocate memory for states auxilliary information */
      {
        ssSetStatesInfo2(rts,
                         &Ball_and_Plate_MicroLabBox_student_M->NonInlinedSFcns.statesInfo2
                         [4]);
        ssSetPeriodicStatesInfo(rts,
          &Ball_and_Plate_MicroLabBox_student_M->NonInlinedSFcns.periodicStatesInfo
          [4]);
      }

      /* inputs */
      {
        _ssSetNumInputPorts(rts, 1);
        ssSetPortInfoForInputs(rts,
          &Ball_and_Plate_MicroLabBox_student_M->NonInlinedSFcns.Sfcn4.inputPortInfo
          [0]);
        _ssSetPortInfo2ForInputUnits(rts,
          &Ball_and_Plate_MicroLabBox_student_M->NonInlinedSFcns.Sfcn4.inputPortUnits
          [0]);
        ssSetInputPortUnit(rts, 0, 0);
        _ssSetPortInfo2ForInputCoSimAttribute(rts,
          &Ball_and_Plate_MicroLabBox_student_M->NonInlinedSFcns.Sfcn4.inputPortCoSimAttribute
          [0]);
        ssSetInputPortIsContinuousQuantity(rts, 0, 0);

        /* port 0 */
        {
          real_T const **sfcnUPtrs = (real_T const **)
            &Ball_and_Plate_MicroLabBox_student_M->NonInlinedSFcns.Sfcn4.UPtrs0;
          sfcnUPtrs[0] = &Ball_and_Plate_MicroLabBox_student_B.Gain1_e;
          ssSetInputPortSignalPtrs(rts, 0, (InputPtrsType)&sfcnUPtrs[0]);
          _ssSetInputPortNumDimensions(rts, 0, 1);
          ssSetInputPortWidth(rts, 0, 1);
        }
      }

      /* outputs */
      {
        ssSetPortInfoForOutputs(rts,
          &Ball_and_Plate_MicroLabBox_student_M->NonInlinedSFcns.Sfcn4.outputPortInfo
          [0]);
        _ssSetNumOutputPorts(rts, 1);
        _ssSetPortInfo2ForOutputUnits(rts,
          &Ball_and_Plate_MicroLabBox_student_M->NonInlinedSFcns.Sfcn4.outputPortUnits
          [0]);
        ssSetOutputPortUnit(rts, 0, 0);
        _ssSetPortInfo2ForOutputCoSimAttribute(rts,
          &Ball_and_Plate_MicroLabBox_student_M->NonInlinedSFcns.Sfcn4.outputPortCoSimAttribute
          [0]);
        ssSetOutputPortIsContinuousQuantity(rts, 0, 0);

        /* port 0 */
        {
          _ssSetOutputPortNumDimensions(rts, 0, 1);
          ssSetOutputPortWidth(rts, 0, 1);
          ssSetOutputPortSignal(rts, 0, ((real_T *)
            &Ball_and_Plate_MicroLabBox_student_B.Dctleadlag2_b));
        }
      }

      /* path info */
      ssSetModelName(rts, "Dctleadlag2");
      ssSetPath(rts,
                "Ball_and_Plate_MicroLabBox_student/Innerloop_Actuator/Motor_C/Controller_10_05/Dctleadlag2");
      ssSetRTModel(rts,Ball_and_Plate_MicroLabBox_student_M);
      ssSetParentSS(rts, (NULL));
      ssSetRootSS(rts, rts);
      ssSetVersion(rts, SIMSTRUCT_VERSION_LEVEL2);

      /* parameters */
      {
        mxArray **sfcnParams = (mxArray **)
          &Ball_and_Plate_MicroLabBox_student_M->NonInlinedSFcns.Sfcn4.params;
        ssSetSFcnParamsCount(rts, 3);
        ssSetSFcnParamsPtr(rts, &sfcnParams[0]);
        ssSetSFcnParam(rts, 0, (mxArray*)
                       Ball_and_Plate_MicroLabBox_student_P.Dctleadlag2_P1_Size_o);
        ssSetSFcnParam(rts, 1, (mxArray*)
                       Ball_and_Plate_MicroLabBox_student_P.Dctleadlag2_P2_Size_fd);
        ssSetSFcnParam(rts, 2, (mxArray*)
                       Ball_and_Plate_MicroLabBox_student_P.Dctleadlag2_P3_Size_b);
      }

      /* work vectors */
      ssSetRWork(rts, (real_T *)
                 &Ball_and_Plate_MicroLabBox_student_DW.Dctleadlag2_RWORK_kg[0]);

      {
        struct _ssDWorkRecord *dWorkRecord = (struct _ssDWorkRecord *)
          &Ball_and_Plate_MicroLabBox_student_M->NonInlinedSFcns.Sfcn4.dWork;
        struct _ssDWorkAuxRecord *dWorkAuxRecord = (struct _ssDWorkAuxRecord *)
          &Ball_and_Plate_MicroLabBox_student_M->NonInlinedSFcns.Sfcn4.dWorkAux;
        ssSetSFcnDWork(rts, dWorkRecord);
        ssSetSFcnDWorkAux(rts, dWorkAuxRecord);
        _ssSetNumDWork(rts, 1);

        /* RWORK */
        ssSetDWorkWidth(rts, 0, 2);
        ssSetDWorkDataType(rts, 0,SS_DOUBLE);
        ssSetDWorkComplexSignal(rts, 0, 0);
        ssSetDWork(rts, 0,
                   &Ball_and_Plate_MicroLabBox_student_DW.Dctleadlag2_RWORK_kg[0]);
      }

      /* registration */
      dleadlag(rts);
      sfcnInitializeSizes(rts);
      sfcnInitializeSampleTimes(rts);

      /* adjust sample time */
      ssSetSampleTime(rts, 0, 0.001);
      ssSetOffsetTime(rts, 0, 0.0);
      sfcnTsMap[0] = 1;

      /* set compiled values of dynamic vector attributes */
      ssSetNumNonsampledZCs(rts, 0);

      /* Update connectivity flags for each port */
      _ssSetInputPortConnected(rts, 0, 1);
      _ssSetOutputPortConnected(rts, 0, 1);
      _ssSetOutputPortBeingMerged(rts, 0, 0);

      /* Update the BufferDstPort flags for each input port */
      ssSetInputPortBufferDstPort(rts, 0, -1);
    }

    /* Level2 S-Function Block: Ball_and_Plate_MicroLabBox_student/<S42>/Dct1lowpass3 (dlowpass1) */
    {
      SimStruct *rts = Ball_and_Plate_MicroLabBox_student_M->childSfunctions[5];

      /* timing info */
      time_T *sfcnPeriod =
        Ball_and_Plate_MicroLabBox_student_M->NonInlinedSFcns.Sfcn5.sfcnPeriod;
      time_T *sfcnOffset =
        Ball_and_Plate_MicroLabBox_student_M->NonInlinedSFcns.Sfcn5.sfcnOffset;
      int_T *sfcnTsMap =
        Ball_and_Plate_MicroLabBox_student_M->NonInlinedSFcns.Sfcn5.sfcnTsMap;
      (void) memset((void*)sfcnPeriod, 0,
                    sizeof(time_T)*1);
      (void) memset((void*)sfcnOffset, 0,
                    sizeof(time_T)*1);
      ssSetSampleTimePtr(rts, &sfcnPeriod[0]);
      ssSetOffsetTimePtr(rts, &sfcnOffset[0]);
      ssSetSampleTimeTaskIDPtr(rts, sfcnTsMap);

      {
        ssSetBlkInfo2Ptr(rts,
                         &Ball_and_Plate_MicroLabBox_student_M->NonInlinedSFcns.blkInfo2
                         [5]);
      }

      _ssSetBlkInfo2PortInfo2Ptr(rts,
        &Ball_and_Plate_MicroLabBox_student_M->NonInlinedSFcns.inputOutputPortInfo2
        [5]);

      /* Set up the mdlInfo pointer */
      ssSetRTWSfcnInfo(rts, Ball_and_Plate_MicroLabBox_student_M->sfcnInfo);

      /* Allocate memory of model methods 2 */
      {
        ssSetModelMethods2(rts,
                           &Ball_and_Plate_MicroLabBox_student_M->NonInlinedSFcns.methods2
                           [5]);
      }

      /* Allocate memory of model methods 3 */
      {
        ssSetModelMethods3(rts,
                           &Ball_and_Plate_MicroLabBox_student_M->NonInlinedSFcns.methods3
                           [5]);
      }

      /* Allocate memory of model methods 4 */
      {
        ssSetModelMethods4(rts,
                           &Ball_and_Plate_MicroLabBox_student_M->NonInlinedSFcns.methods4
                           [5]);
      }

      /* Allocate memory for states auxilliary information */
      {
        ssSetStatesInfo2(rts,
                         &Ball_and_Plate_MicroLabBox_student_M->NonInlinedSFcns.statesInfo2
                         [5]);
        ssSetPeriodicStatesInfo(rts,
          &Ball_and_Plate_MicroLabBox_student_M->NonInlinedSFcns.periodicStatesInfo
          [5]);
      }

      /* inputs */
      {
        _ssSetNumInputPorts(rts, 1);
        ssSetPortInfoForInputs(rts,
          &Ball_and_Plate_MicroLabBox_student_M->NonInlinedSFcns.Sfcn5.inputPortInfo
          [0]);
        _ssSetPortInfo2ForInputUnits(rts,
          &Ball_and_Plate_MicroLabBox_student_M->NonInlinedSFcns.Sfcn5.inputPortUnits
          [0]);
        ssSetInputPortUnit(rts, 0, 0);
        _ssSetPortInfo2ForInputCoSimAttribute(rts,
          &Ball_and_Plate_MicroLabBox_student_M->NonInlinedSFcns.Sfcn5.inputPortCoSimAttribute
          [0]);
        ssSetInputPortIsContinuousQuantity(rts, 0, 0);

        /* port 0 */
        {
          real_T const **sfcnUPtrs = (real_T const **)
            &Ball_and_Plate_MicroLabBox_student_M->NonInlinedSFcns.Sfcn5.UPtrs0;
          sfcnUPtrs[0] = &Ball_and_Plate_MicroLabBox_student_B.Dctleadlag2_b;
          ssSetInputPortSignalPtrs(rts, 0, (InputPtrsType)&sfcnUPtrs[0]);
          _ssSetInputPortNumDimensions(rts, 0, 1);
          ssSetInputPortWidth(rts, 0, 1);
        }
      }

      /* outputs */
      {
        ssSetPortInfoForOutputs(rts,
          &Ball_and_Plate_MicroLabBox_student_M->NonInlinedSFcns.Sfcn5.outputPortInfo
          [0]);
        _ssSetNumOutputPorts(rts, 1);
        _ssSetPortInfo2ForOutputUnits(rts,
          &Ball_and_Plate_MicroLabBox_student_M->NonInlinedSFcns.Sfcn5.outputPortUnits
          [0]);
        ssSetOutputPortUnit(rts, 0, 0);
        _ssSetPortInfo2ForOutputCoSimAttribute(rts,
          &Ball_and_Plate_MicroLabBox_student_M->NonInlinedSFcns.Sfcn5.outputPortCoSimAttribute
          [0]);
        ssSetOutputPortIsContinuousQuantity(rts, 0, 0);

        /* port 0 */
        {
          _ssSetOutputPortNumDimensions(rts, 0, 1);
          ssSetOutputPortWidth(rts, 0, 1);
          ssSetOutputPortSignal(rts, 0, ((real_T *)
            &Ball_and_Plate_MicroLabBox_student_B.Dct1lowpass3_l));
        }
      }

      /* path info */
      ssSetModelName(rts, "Dct1lowpass3");
      ssSetPath(rts,
                "Ball_and_Plate_MicroLabBox_student/Innerloop_Actuator/Motor_C/Controller_10_05/Dct1lowpass3");
      ssSetRTModel(rts,Ball_and_Plate_MicroLabBox_student_M);
      ssSetParentSS(rts, (NULL));
      ssSetRootSS(rts, rts);
      ssSetVersion(rts, SIMSTRUCT_VERSION_LEVEL2);

      /* parameters */
      {
        mxArray **sfcnParams = (mxArray **)
          &Ball_and_Plate_MicroLabBox_student_M->NonInlinedSFcns.Sfcn5.params;
        ssSetSFcnParamsCount(rts, 2);
        ssSetSFcnParamsPtr(rts, &sfcnParams[0]);
        ssSetSFcnParam(rts, 0, (mxArray*)
                       Ball_and_Plate_MicroLabBox_student_P.Dct1lowpass3_P1_Size_m);
        ssSetSFcnParam(rts, 1, (mxArray*)
                       Ball_and_Plate_MicroLabBox_student_P.Dct1lowpass3_P2_Size_g);
      }

      /* work vectors */
      ssSetRWork(rts, (real_T *)
                 &Ball_and_Plate_MicroLabBox_student_DW.Dct1lowpass3_RWORK_ci[0]);

      {
        struct _ssDWorkRecord *dWorkRecord = (struct _ssDWorkRecord *)
          &Ball_and_Plate_MicroLabBox_student_M->NonInlinedSFcns.Sfcn5.dWork;
        struct _ssDWorkAuxRecord *dWorkAuxRecord = (struct _ssDWorkAuxRecord *)
          &Ball_and_Plate_MicroLabBox_student_M->NonInlinedSFcns.Sfcn5.dWorkAux;
        ssSetSFcnDWork(rts, dWorkRecord);
        ssSetSFcnDWorkAux(rts, dWorkAuxRecord);
        _ssSetNumDWork(rts, 1);

        /* RWORK */
        ssSetDWorkWidth(rts, 0, 2);
        ssSetDWorkDataType(rts, 0,SS_DOUBLE);
        ssSetDWorkComplexSignal(rts, 0, 0);
        ssSetDWork(rts, 0,
                   &Ball_and_Plate_MicroLabBox_student_DW.Dct1lowpass3_RWORK_ci
                   [0]);
      }

      /* registration */
      dlowpass1(rts);
      sfcnInitializeSizes(rts);
      sfcnInitializeSampleTimes(rts);

      /* adjust sample time */
      ssSetSampleTime(rts, 0, 0.001);
      ssSetOffsetTime(rts, 0, 0.0);
      sfcnTsMap[0] = 1;

      /* set compiled values of dynamic vector attributes */
      ssSetNumNonsampledZCs(rts, 0);

      /* Update connectivity flags for each port */
      _ssSetInputPortConnected(rts, 0, 1);
      _ssSetOutputPortConnected(rts, 0, 1);
      _ssSetOutputPortBeingMerged(rts, 0, 0);

      /* Update the BufferDstPort flags for each input port */
      ssSetInputPortBufferDstPort(rts, 0, -1);
    }
  }

  {
    /* user code (registration function declaration) */
    /*Initialize global TRC pointers. */
    Ball_and_Plate_MicroLabBox_student_rti_init_trc_pointers();
  }

  /* Start for Enabled SubSystem: '<S25>/Enabled Subsystem' */
  Ball_and_EnabledSubsystem_Start
    (&Ball_and_Plate_MicroLabBox_student_DW.EnabledSubsystem);

  /* End of Start for SubSystem: '<S25>/Enabled Subsystem' */

  /* Start for Enabled SubSystem: '<S25>/Enabled Subsystem1' */
  Ball_an_EnabledSubsystem1_Start
    (&Ball_and_Plate_MicroLabBox_student_DW.EnabledSubsystem1);

  /* End of Start for SubSystem: '<S25>/Enabled Subsystem1' */
  /* End of Start for SubSystem: '<S10>/Position Measurement' */

  /* Start for Enabled SubSystem: '<S39>/Enabled Subsystem' */
  Ball_and_EnabledSubsystem_Start
    (&Ball_and_Plate_MicroLabBox_student_DW.EnabledSubsystem_a);

  /* End of Start for SubSystem: '<S39>/Enabled Subsystem' */

  /* Start for Enabled SubSystem: '<S39>/Enabled Subsystem1' */
  Ball_an_EnabledSubsystem1_Start
    (&Ball_and_Plate_MicroLabBox_student_DW.EnabledSubsystem1_f);

  /* End of Start for SubSystem: '<S39>/Enabled Subsystem1' */
  /* End of Start for SubSystem: '<S11>/Position Measurement' */

  /* Start for Enabled SubSystem: '<S55>/Enabled Subsystem' */
  Ball_and_EnabledSubsystem_Start
    (&Ball_and_Plate_MicroLabBox_student_DW.EnabledSubsystem_n);

  /* End of Start for SubSystem: '<S55>/Enabled Subsystem' */

  /* Start for Enabled SubSystem: '<S55>/Enabled Subsystem1' */
  Ball_an_EnabledSubsystem1_Start
    (&Ball_and_Plate_MicroLabBox_student_DW.EnabledSubsystem1_b);

  /* End of Start for SubSystem: '<S55>/Enabled Subsystem1' */
  /* End of Start for SubSystem: '<S12>/Position Measurement' */

  /* Start for S-Function (dleadlag): '<S13>/Dctleadlag2' */
  /* Level2 S-Function Block: '<S13>/Dctleadlag2' (dleadlag) */
  {
    SimStruct *rts = Ball_and_Plate_MicroLabBox_student_M->childSfunctions[0];
    sfcnStart(rts);
    if (ssGetErrorStatus(rts) != (NULL))
      return;
  }

  /* Start for S-Function (dlowpass1): '<S13>/Dct1lowpass3' */
  /* Level2 S-Function Block: '<S13>/Dct1lowpass3' (dlowpass1) */
  {
    SimStruct *rts = Ball_and_Plate_MicroLabBox_student_M->childSfunctions[1];
    sfcnStart(rts);
    if (ssGetErrorStatus(rts) != (NULL))
      return;
  }

  /* Start for S-Function (dleadlag): '<S28>/Dctleadlag2' */
  /* Level2 S-Function Block: '<S28>/Dctleadlag2' (dleadlag) */
  {
    SimStruct *rts = Ball_and_Plate_MicroLabBox_student_M->childSfunctions[2];
    sfcnStart(rts);
    if (ssGetErrorStatus(rts) != (NULL))
      return;
  }

  /* Start for S-Function (dlowpass1): '<S28>/Dct1lowpass3' */
  /* Level2 S-Function Block: '<S28>/Dct1lowpass3' (dlowpass1) */
  {
    SimStruct *rts = Ball_and_Plate_MicroLabBox_student_M->childSfunctions[3];
    sfcnStart(rts);
    if (ssGetErrorStatus(rts) != (NULL))
      return;
  }

  /* Start for S-Function (dleadlag): '<S42>/Dctleadlag2' */
  /* Level2 S-Function Block: '<S42>/Dctleadlag2' (dleadlag) */
  {
    SimStruct *rts = Ball_and_Plate_MicroLabBox_student_M->childSfunctions[4];
    sfcnStart(rts);
    if (ssGetErrorStatus(rts) != (NULL))
      return;
  }

  /* Start for S-Function (dlowpass1): '<S42>/Dct1lowpass3' */
  /* Level2 S-Function Block: '<S42>/Dct1lowpass3' (dlowpass1) */
  {
    SimStruct *rts = Ball_and_Plate_MicroLabBox_student_M->childSfunctions[5];
    sfcnStart(rts);
    if (ssGetErrorStatus(rts) != (NULL))
      return;
  }

  Ball_and_Plate_MicroLabBox_student_PrevZCX.EMC_ENCODER_POS_SET_BL1_Trig_fh =
    UNINITIALIZED_ZCSIG;
  Ball_and_Plate_MicroLabBox_student_PrevZCX.EMC_ENCODER_POS_SET_BL1_Trig__f =
    UNINITIALIZED_ZCSIG;
  Ball_and_Plate_MicroLabBox_student_PrevZCX.EMC_ENCODER_POS_SET_BL1_Trig_ZC =
    UNINITIALIZED_ZCSIG;

  {
    int32_T i;

    /* InitializeConditions for UnitDelay: '<S1>/UD' */
    Ball_and_Plate_MicroLabBox_student_DW.UD_DSTATE =
      Ball_and_Plate_MicroLabBox_student_P.Difference_ICPrevInput;

    /* SystemInitialize for Atomic SubSystem: '<S10>/Position Measurement' */
    /* InitializeConditions for TransferFcn: '<S25>/Transfer Fcn' */
    Ball_and_Plate_MicroLabBox_student_X.TransferFcn_CSTATE_l = 0.0;

    /* SystemInitialize for Enabled SubSystem: '<S25>/Enabled Subsystem' */
    Ball_and__EnabledSubsystem_Init
      (&Ball_and_Plate_MicroLabBox_student_B.EnabledSubsystem,
       &Ball_and_Plate_MicroLabBox_student_P.EnabledSubsystem);

    /* End of SystemInitialize for SubSystem: '<S25>/Enabled Subsystem' */

    /* SystemInitialize for Enabled SubSystem: '<S25>/Enabled Subsystem1' */
    Ball_and_EnabledSubsystem1_Init
      (&Ball_and_Plate_MicroLabBox_student_B.EnabledSubsystem1,
       &Ball_and_Plate_MicroLabBox_student_P.EnabledSubsystem1);

    /* End of SystemInitialize for SubSystem: '<S25>/Enabled Subsystem1' */
    /* End of SystemInitialize for SubSystem: '<S10>/Position Measurement' */

    /* SystemInitialize for Atomic SubSystem: '<S11>/Position Measurement' */
    /* InitializeConditions for TransferFcn: '<S39>/Transfer Fcn' */
    Ball_and_Plate_MicroLabBox_student_X.TransferFcn_CSTATE_h = 0.0;

    /* SystemInitialize for Enabled SubSystem: '<S39>/Enabled Subsystem' */
    Ball_and__EnabledSubsystem_Init
      (&Ball_and_Plate_MicroLabBox_student_B.EnabledSubsystem_a,
       &Ball_and_Plate_MicroLabBox_student_P.EnabledSubsystem_a);

    /* End of SystemInitialize for SubSystem: '<S39>/Enabled Subsystem' */

    /* SystemInitialize for Enabled SubSystem: '<S39>/Enabled Subsystem1' */
    Ball_and_EnabledSubsystem1_Init
      (&Ball_and_Plate_MicroLabBox_student_B.EnabledSubsystem1_f,
       &Ball_and_Plate_MicroLabBox_student_P.EnabledSubsystem1_f);

    /* End of SystemInitialize for SubSystem: '<S39>/Enabled Subsystem1' */
    /* End of SystemInitialize for SubSystem: '<S11>/Position Measurement' */

    /* SystemInitialize for Atomic SubSystem: '<S12>/Position Measurement' */
    /* InitializeConditions for TransferFcn: '<S55>/Transfer Fcn' */
    Ball_and_Plate_MicroLabBox_student_X.TransferFcn_CSTATE = 0.0;

    /* SystemInitialize for Enabled SubSystem: '<S55>/Enabled Subsystem' */
    Ball_and__EnabledSubsystem_Init
      (&Ball_and_Plate_MicroLabBox_student_B.EnabledSubsystem_n,
       &Ball_and_Plate_MicroLabBox_student_P.EnabledSubsystem_n);

    /* End of SystemInitialize for SubSystem: '<S55>/Enabled Subsystem' */

    /* SystemInitialize for Enabled SubSystem: '<S55>/Enabled Subsystem1' */
    Ball_and_EnabledSubsystem1_Init
      (&Ball_and_Plate_MicroLabBox_student_B.EnabledSubsystem1_b,
       &Ball_and_Plate_MicroLabBox_student_P.EnabledSubsystem1_b);

    /* End of SystemInitialize for SubSystem: '<S55>/Enabled Subsystem1' */
    /* End of SystemInitialize for SubSystem: '<S12>/Position Measurement' */

    /* SystemInitialize for MATLAB Function: '<S14>/MATLAB Function' */
    Ball_and_Pl_MATLABFunction_Init
      (&Ball_and_Plate_MicroLabBox_student_DW.sf_MATLABFunction);

    /* SystemInitialize for MATLAB Function: '<S10>/MATLAB Function1' */
    Ball_and_Plate_MicroLabBox_student_DW.prev_ref_end_not_empty = false;
    for (i = 0; i < 6; i++) {
      Ball_and_Plate_MicroLabBox_student_DW.coeffs[i] = 0.0;
    }

    Ball_and_Plate_MicroLabBox_student_DW.t_elapsed = 0.0;
    Ball_and_Plate_MicroLabBox_student_DW.tf_internal = 0.0;

    /* End of SystemInitialize for MATLAB Function: '<S10>/MATLAB Function1' */

    /* SystemInitialize for MATLAB Function: '<S16>/MATLAB Function' */
    Ball_and__MATLABFunction_o_Init
      (&Ball_and_Plate_MicroLabBox_student_DW.sf_MATLABFunction_e);

    /* SystemInitialize for MATLAB Function: '<S29>/MATLAB Function' */
    Ball_and_Pl_MATLABFunction_Init
      (&Ball_and_Plate_MicroLabBox_student_DW.sf_MATLABFunction_k);

    /* SystemInitialize for MATLAB Function: '<S30>/MATLAB Function' */
    Ball_and__MATLABFunction_o_Init
      (&Ball_and_Plate_MicroLabBox_student_DW.sf_MATLABFunction_ko);

    /* SystemInitialize for MATLAB Function: '<S43>/MATLAB Function' */
    Ball_and_Pl_MATLABFunction_Init
      (&Ball_and_Plate_MicroLabBox_student_DW.sf_MATLABFunction_h);

    /* SystemInitialize for MATLAB Function: '<S45>/MATLAB Function' */
    Ball_and__MATLABFunction_o_Init
      (&Ball_and_Plate_MicroLabBox_student_DW.sf_MATLABFunction_hg);

    /* SystemInitialize for MATLAB Function: '<S44>/MATLAB Function' */
    Ball_and__MATLABFunction_o_Init
      (&Ball_and_Plate_MicroLabBox_student_DW.sf_MATLABFunction_g);
  }
}

/* Model terminate function */
void Ball_and_Plate_MicroLabBox_student_terminate(void)
{
  /* Terminate for S-Function (rti_commonblock): '<S8>/S-Function1' */
  {
    /* --- Ball_and_Plate_MicroLabBox_student/Ethernet communication/ETHERNET_UDP_SETUP_BL1: ==> Socket ID = (1) --- */
    /* dSPACE I/O Board DSETHERNET #1 Unit:SETUPUDP Group:SETUPUDP */

    /* close a socket connection
     * After closing, the corresponding socket resource is still occupied.   *
     * If a connection has been closed and should be re-used for a different *
     * connection, or if a connection should be re-established it must be    *
     * re-opened using DsIoEth_open() again.                                 */
    DsIoEth_close(DSIOETH_CONNECTION_ID_1);
  }

  /* Terminate for Atomic SubSystem: '<S10>/Position Measurement' */
  /* Terminate for Triggered SubSystem: '<S18>/EMC_ENCODER_POS_SET_BL1' */
  /* Terminate for S-Function (rti_commonblock): '<S24>/S-Function1' incorporates:
   *  Constant: '<S18>/Constant'
   */

  /* --- Ball_and_Plate_MicroLabBox_student/Innerloop_Actuator/Motor_A/Position Measurement/EMC_ENCODER_POS_SET_BL1 --- */
  /* --- [RTIEMC, Encoder] - DIO class: 2 - Unit: 1 - Port: 1 - Channel: 1 --- */
  {
    /* Set positions state for the selected incremental Encoder to INVALID */
    DioCl2EncoderIn_setEncPosValidity(pRTIEmcEncoder_Unit_1_DioCl_2_Port_1_Ch1,
      DIO_ENC_POSITION_INVALID);

    /* Writes settings for the incremental Encoder */
    DioCl2EncoderIn_write(pRTIEmcEncoder_Unit_1_DioCl_2_Port_1_Ch1);
  }

  /* End of Terminate for SubSystem: '<S18>/EMC_ENCODER_POS_SET_BL1' */

  /* Terminate for S-Function (rti_commonblock): '<S23>/S-Function1' */

  /* --- Ball_and_Plate_MicroLabBox_student/Innerloop_Actuator/Motor_A/Position Measurement/EMC_ENCODER_BL1 --- */
  /* --- [RTIEMC, Encoder] - DIO class: 2 - Unit: 1 - Port: 1 - Channel: 1 --- */
  {
    /* Deactivates encoder interface functionality */
    DioCl2EncoderIn_stop(pRTIEmcEncoder_Unit_1_DioCl_2_Port_1_Ch1);
  }

  /* End of Terminate for SubSystem: '<S10>/Position Measurement' */

  /* Terminate for Atomic SubSystem: '<S11>/Position Measurement' */
  /* Terminate for Triggered SubSystem: '<S32>/EMC_ENCODER_POS_SET_BL1' */
  /* Terminate for S-Function (rti_commonblock): '<S38>/S-Function1' incorporates:
   *  Constant: '<S32>/Constant'
   */

  /* --- Ball_and_Plate_MicroLabBox_student/Innerloop_Actuator/Motor_B/Position Measurement/EMC_ENCODER_POS_SET_BL1 --- */
  /* --- [RTIEMC, Encoder] - DIO class: 2 - Unit: 2 - Port: 1 - Channel: 3 --- */
  {
    /* Set positions state for the selected incremental Encoder to INVALID */
    DioCl2EncoderIn_setEncPosValidity(pRTIEmcEncoder_Unit_2_DioCl_2_Port_1_Ch3,
      DIO_ENC_POSITION_INVALID);

    /* Writes settings for the incremental Encoder */
    DioCl2EncoderIn_write(pRTIEmcEncoder_Unit_2_DioCl_2_Port_1_Ch3);
  }

  /* End of Terminate for SubSystem: '<S32>/EMC_ENCODER_POS_SET_BL1' */

  /* Terminate for S-Function (rti_commonblock): '<S37>/S-Function1' */

  /* --- Ball_and_Plate_MicroLabBox_student/Innerloop_Actuator/Motor_B/Position Measurement/EMC_ENCODER_BL1 --- */
  /* --- [RTIEMC, Encoder] - DIO class: 2 - Unit: 2 - Port: 1 - Channel: 3 --- */
  {
    /* Deactivates encoder interface functionality */
    DioCl2EncoderIn_stop(pRTIEmcEncoder_Unit_2_DioCl_2_Port_1_Ch3);
  }

  /* End of Terminate for SubSystem: '<S11>/Position Measurement' */

  /* Terminate for Atomic SubSystem: '<S12>/Position Measurement' */
  /* Terminate for Triggered SubSystem: '<S47>/EMC_ENCODER_POS_SET_BL1' */
  /* Terminate for S-Function (rti_commonblock): '<S54>/S-Function1' incorporates:
   *  Constant: '<S47>/Constant'
   */

  /* --- Ball_and_Plate_MicroLabBox_student/Innerloop_Actuator/Motor_C/Position Measurement/EMC_ENCODER_POS_SET_BL1 --- */
  /* --- [RTIEMC, Encoder] - DIO class: 2 - Unit: 3 - Port: 1 - Channel: 5 --- */
  {
    /* Set positions state for the selected incremental Encoder to INVALID */
    DioCl2EncoderIn_setEncPosValidity(pRTIEmcEncoder_Unit_3_DioCl_2_Port_1_Ch5,
      DIO_ENC_POSITION_INVALID);

    /* Writes settings for the incremental Encoder */
    DioCl2EncoderIn_write(pRTIEmcEncoder_Unit_3_DioCl_2_Port_1_Ch5);
  }

  /* End of Terminate for SubSystem: '<S47>/EMC_ENCODER_POS_SET_BL1' */

  /* Terminate for S-Function (rti_commonblock): '<S53>/S-Function1' */

  /* --- Ball_and_Plate_MicroLabBox_student/Innerloop_Actuator/Motor_C/Position Measurement/EMC_ENCODER_BL1 --- */
  /* --- [RTIEMC, Encoder] - DIO class: 2 - Unit: 3 - Port: 1 - Channel: 5 --- */
  {
    /* Deactivates encoder interface functionality */
    DioCl2EncoderIn_stop(pRTIEmcEncoder_Unit_3_DioCl_2_Port_1_Ch5);
  }

  /* End of Terminate for SubSystem: '<S12>/Position Measurement' */

  /* Terminate for S-Function (dleadlag): '<S13>/Dctleadlag2' */
  /* Level2 S-Function Block: '<S13>/Dctleadlag2' (dleadlag) */
  {
    SimStruct *rts = Ball_and_Plate_MicroLabBox_student_M->childSfunctions[0];
    sfcnTerminate(rts);
  }

  /* Terminate for S-Function (dlowpass1): '<S13>/Dct1lowpass3' */
  /* Level2 S-Function Block: '<S13>/Dct1lowpass3' (dlowpass1) */
  {
    SimStruct *rts = Ball_and_Plate_MicroLabBox_student_M->childSfunctions[1];
    sfcnTerminate(rts);
  }

  /* Terminate for Atomic SubSystem: '<S10>/Outputs to Amplifier' */
  /* Terminate for S-Function (rti_commonblock): '<S22>/S-Function1' */

  /* --- Ball_and_Plate_MicroLabBox_student/Innerloop_Actuator/Motor_A/Outputs to Amplifier/DAC_CLASS1_BL1 --- */
  /* --- [RTI120X, DAC C1] - Channel: 1 --- */

  /* All channel outputs are set to high impedance state */
  DacCl1AnalogOut_setOutputHighZ(pRTIDacC1AnalogOut_Ch_1, DAC_CLASS1_HIGH_Z_ON);

  /* Deactivates AnalogOut functionality */
  DacCl1AnalogOut_stop(pRTIDacC1AnalogOut_Ch_1);

  /* End of Terminate for SubSystem: '<S10>/Outputs to Amplifier' */

  /* Terminate for S-Function (dleadlag): '<S28>/Dctleadlag2' */
  /* Level2 S-Function Block: '<S28>/Dctleadlag2' (dleadlag) */
  {
    SimStruct *rts = Ball_and_Plate_MicroLabBox_student_M->childSfunctions[2];
    sfcnTerminate(rts);
  }

  /* Terminate for S-Function (dlowpass1): '<S28>/Dct1lowpass3' */
  /* Level2 S-Function Block: '<S28>/Dct1lowpass3' (dlowpass1) */
  {
    SimStruct *rts = Ball_and_Plate_MicroLabBox_student_M->childSfunctions[3];
    sfcnTerminate(rts);
  }

  /* Terminate for Atomic SubSystem: '<S11>/Outputs to Amplifier' */
  /* Terminate for S-Function (rti_commonblock): '<S36>/S-Function1' */

  /* --- Ball_and_Plate_MicroLabBox_student/Innerloop_Actuator/Motor_B/Outputs to Amplifier/DAC_CLASS1_BL1 --- */
  /* --- [RTI120X, DAC C1] - Channel: 2 --- */

  /* All channel outputs are set to high impedance state */
  DacCl1AnalogOut_setOutputHighZ(pRTIDacC1AnalogOut_Ch_2, DAC_CLASS1_HIGH_Z_ON);

  /* Deactivates AnalogOut functionality */
  DacCl1AnalogOut_stop(pRTIDacC1AnalogOut_Ch_2);

  /* End of Terminate for SubSystem: '<S11>/Outputs to Amplifier' */

  /* Terminate for S-Function (dleadlag): '<S42>/Dctleadlag2' */
  /* Level2 S-Function Block: '<S42>/Dctleadlag2' (dleadlag) */
  {
    SimStruct *rts = Ball_and_Plate_MicroLabBox_student_M->childSfunctions[4];
    sfcnTerminate(rts);
  }

  /* Terminate for S-Function (dlowpass1): '<S42>/Dct1lowpass3' */
  /* Level2 S-Function Block: '<S42>/Dct1lowpass3' (dlowpass1) */
  {
    SimStruct *rts = Ball_and_Plate_MicroLabBox_student_M->childSfunctions[5];
    sfcnTerminate(rts);
  }

  /* Terminate for Atomic SubSystem: '<S12>/Outputs to Amplifier' */
  /* Terminate for S-Function (rti_commonblock): '<S52>/S-Function1' */

  /* --- Ball_and_Plate_MicroLabBox_student/Innerloop_Actuator/Motor_C/Outputs to Amplifier/DAC_CLASS1_BL1 --- */
  /* --- [RTI120X, DAC C1] - Channel: 3 --- */

  /* All channel outputs are set to high impedance state */
  DacCl1AnalogOut_setOutputHighZ(pRTIDacC1AnalogOut_Ch_3, DAC_CLASS1_HIGH_Z_ON);

  /* Deactivates AnalogOut functionality */
  DacCl1AnalogOut_stop(pRTIDacC1AnalogOut_Ch_3);

  /* End of Terminate for SubSystem: '<S12>/Outputs to Amplifier' */
}
