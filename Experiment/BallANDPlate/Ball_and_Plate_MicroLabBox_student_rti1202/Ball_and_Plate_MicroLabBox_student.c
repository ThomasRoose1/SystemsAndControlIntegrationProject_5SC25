/*
 * Ball_and_Plate_MicroLabBox_student.c
 *
 * Academic License - for use in teaching, academic research, and meeting
 * course requirements at degree granting institutions only.  Not for
 * government, commercial, or other organizational use.
 *
 * Code generation for model "Ball_and_Plate_MicroLabBox_student".
 *
 * Model version              : 1.91
 * Simulink Coder version : 9.2 (R2019b) 18-Jul-2019
 * C source code generated on : Tue Jun 16 16:16:30 2026
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

/* Named constants for MATLAB Function: '<S92>/optimizer' */
#define Ball_and_Plate_MicroLabBox_s_ny (4.0)

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
static void Ball_and_Plate_MicroLa_mldivide(const real_T A[36], real_T B[6]);

/* Forward declaration for local functions */
static real_T Ball_and_Plate_MicroLabBox_norm(const real_T x[3]);
static void Ball_and_Plate_Mi_Unconstrained(const real_T b_Hinv[49], const
  real_T f[7], real_T x[7], int16_T n);
static real_T Ball_and_Plate_MicroLabB_norm_i(const real_T x[7]);
static void Ball_and_Plate_MicroLabBox__abs(const real_T x[7], real_T y[7]);
static void Ball_and_Plate_MicroLabBo_abs_f(const real_T x[212], real_T y[212]);
static real_T Ball_and_Plate_MicroLabBo_xnrm2(int32_T n, const real_T x[49],
  int32_T ix0);
static void Ball_and_Plate_MicroLabBo_xgemv(int32_T m, int32_T n, const real_T
  b_A[49], int32_T ia0, const real_T x[49], int32_T ix0, real_T y[7]);
static void Ball_and_Plate_MicroLabBo_xgerc(int32_T m, int32_T n, real_T alpha1,
  int32_T ix0, const real_T y[7], real_T b_A[49], int32_T ia0);
static void Ball_and_Plate_MicroLabBox_s_qr(const real_T b_A[49], real_T Q[49],
  real_T R[49]);
static real_T Ball_and_Plate_Micro_KWIKfactor(const real_T b_Ac[1484], const
  int16_T iC[212], int16_T nA, const real_T b_Linv[49], real_T RLinv[49], real_T
  D[49], real_T b_H[49], int16_T n);
static void Ball_and_Plate_M_DropConstraint(int16_T kDrop, int16_T iA[212],
  int16_T *nA, int16_T iC[212]);
static void Ball_and_Plate_MicroLabB_qpkwik(const real_T b_Linv[49], const
  real_T b_Hinv[49], const real_T f[7], const real_T b_Ac[1484], const real_T b
  [212], int16_T iA[212], int16_T b_maxiter, real_T FeasTol, real_T x[7], real_T
  lambda[212], real_T *status);
static void Ball_and_Plate_Micr_mpc_solveQP(const real_T xQP[4], const real_T
  b_Kx[24], const real_T b_Kr[600], const real_T rseq[100], const real_T b_Ku1
  [12], const real_T old_u[2], const real_T b_Kv[156], const real_T vseq[26],
  const real_T b_Kut[300], const real_T b_utarget[50], const real_T b_Linv[49],
  const real_T b_Hinv[49], const real_T b_Ac[1484], const real_T Bc[212],
  boolean_T iA[212], real_T zopt[7], real_T f[7], real_T *status);
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
  if ((Ball_and_Plate_MicroLabBox_student_M->Timing.TaskCounters.TID[2]) > 19) {/* Sample time: [0.02s, 0.0s] */
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
 *    '<S25>/MATLAB Function'
 *    '<S40>/MATLAB Function'
 *    '<S55>/MATLAB Function'
 */
void Ball_and_Pl_MATLABFunction_Init(DW_MATLABFunction_Ball_and_Pl_T *localDW)
{
  localDW->index = 1.0;
  localDW->previous_enable = 0.0;
}

/*
 * Output and update for atomic system:
 *    '<S25>/MATLAB Function'
 *    '<S40>/MATLAB Function'
 *    '<S55>/MATLAB Function'
 */
void Ball_and_Plate_M_MATLABFunction(real_T rtu_enable, const real_T rtu_u[3001],
  B_MATLABFunction_Ball_and_Pla_T *localB, DW_MATLABFunction_Ball_and_Pl_T
  *localDW)
{
  /* MATLAB Function 'Innerloop_Actuator/Motor_A/Init_motion/MATLAB Function': '<S31>:1' */
  /* '<S31>:1:14' */
  if ((rtu_enable != 0.0) && (!(localDW->previous_enable != 0.0))) {
    /* '<S31>:1:17' */
    /* '<S31>:1:18' */
    localDW->index = 1.0;
  }

  if (rtu_enable != 0.0) {
    if (localDW->index <= 3001.0) {
      /* '<S31>:1:25' */
      /* '<S31>:1:26' */
      localB->y = rtu_u[(int32_T)localDW->index - 1];

      /* '<S31>:1:27' */
      localDW->index++;
    } else {
      /* '<S31>:1:31' */
      localB->y = 0.0;
    }
  } else {
    /* '<S31>:1:36' */
    localB->y = -0.0289;
  }

  /* '<S31>:1:39' */
  localDW->previous_enable = rtu_enable;
}

/*
 * Output and update for atomic system:
 *    '<S25>/MATLAB Function2'
 *    '<S40>/MATLAB Function2'
 *    '<S55>/MATLAB Function2'
 */
void Ball_and_Plate__MATLABFunction2(real_T rtu_y, real_T rtu_start, real_T
  rtu_init_value, B_MATLABFunction2_Ball_and_Pl_T *localB)
{
  /* MATLAB Function 'Innerloop_Actuator/Motor_A/Init_motion/MATLAB Function2': '<S32>:1' */
  if (rtu_start > 0.5) {
    /* '<S32>:1:3' */
    /* '<S32>:1:4' */
    localB->path = rtu_init_value;
  } else if (rtu_start < 0.5) {
    /* '<S32>:1:5' */
    /* '<S32>:1:6' */
    localB->path = rtu_y;
  } else {
    /* '<S32>:1:8' */
    localB->path = 0.0;
  }
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

/* Function for MATLAB Function: '<S21>/MATLAB Function1' */
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

/*
 * System initialize for atomic system:
 *    '<S21>/MATLAB Function1'
 *    '<S22>/MATLAB Function2'
 *    '<S23>/MATLAB Function1'
 */
void Ball_and_P_MATLABFunction1_Init(DW_MATLABFunction1_Ball_and_P_T *localDW)
{
  int32_T i;
  localDW->prev_ref_end_not_empty = false;
  for (i = 0; i < 6; i++) {
    localDW->coeffs[i] = 0.0;
  }

  localDW->t_elapsed = 0.0;
  localDW->tf_internal = 0.0;
}

/*
 * Output and update for atomic system:
 *    '<S21>/MATLAB Function1'
 *    '<S22>/MATLAB Function2'
 *    '<S23>/MATLAB Function1'
 */
void Ball_and_Plate__MATLABFunction1(real_T rtu_reference_end, real_T
  rtu_end_time, real_T rtu_Ts, B_MATLABFunction1_Ball_and_Pl_T *localB,
  DW_MATLABFunction1_Ball_and_P_T *localDW)
{
  real_T path;
  real_T b[36];
  real_T tmp;
  real_T tmp_0;
  real_T tmp_1;
  real_T tmp_2;
  real_T tmp_3;
  int32_T i;
  static const int8_T b_0[6] = { 1, 0, 0, 0, 0, 0 };

  static const int8_T c[6] = { 0, 1, 0, 0, 0, 0 };

  static const int8_T d[6] = { 0, 0, 2, 0, 0, 0 };

  /* MATLAB Function 'Innerloop_Actuator/Motor_A/MATLAB Function1': '<S26>:1' */
  /* '<S26>:1:36' */
  /* '<S26>:1:37' */
  /* '<S26>:1:38' */
  if (!localDW->prev_ref_end_not_empty) {
    /* '<S26>:1:13' */
    /* '<S26>:1:14' */
    localDW->prev_ref_end = rtu_reference_end;
    localDW->prev_ref_end_not_empty = true;
  }

  if (rtu_reference_end != localDW->prev_ref_end) {
    /* '<S26>:1:21' */
    /* '<S26>:1:23' */
    localDW->t_elapsed = 0.0;

    /* '<S26>:1:24' */
    localDW->tf_internal = rtu_end_time;

    /* '<S26>:1:36' */
    /* '<S26>:1:43' */
    /* '<S26>:1:46' */
    localDW->coeffs[0] = localDW->prev_ref_end;
    localDW->coeffs[1] = 0.0;
    localDW->coeffs[2] = 0.0;
    localDW->coeffs[3] = rtu_reference_end;
    localDW->coeffs[4] = 0.0;
    localDW->coeffs[5] = 0.0;
    path = rt_powd_snf(localDW->tf_internal, 3.0);
    tmp = rt_powd_snf(localDW->tf_internal, 4.0);
    tmp_0 = rt_powd_snf(localDW->tf_internal, 5.0);
    tmp_1 = rt_powd_snf(localDW->tf_internal, 3.0);
    tmp_2 = rt_powd_snf(localDW->tf_internal, 4.0);
    tmp_3 = rt_powd_snf(localDW->tf_internal, 3.0);
    for (i = 0; i < 6; i++) {
      b[6 * i] = b_0[i];
      b[6 * i + 1] = c[i];
      b[6 * i + 2] = d[i];
    }

    b[3] = 1.0;
    b[9] = localDW->tf_internal;
    b[15] = localDW->tf_internal * localDW->tf_internal;
    b[21] = path;
    b[27] = tmp;
    b[33] = tmp_0;
    b[4] = 0.0;
    b[10] = 1.0;
    b[16] = 2.0 * localDW->tf_internal;
    b[22] = localDW->tf_internal * localDW->tf_internal * 3.0;
    b[28] = 4.0 * tmp_1;
    b[34] = 5.0 * tmp_2;
    b[5] = 0.0;
    b[11] = 0.0;
    b[17] = 2.0;
    b[23] = 6.0 * localDW->tf_internal;
    b[29] = localDW->tf_internal * localDW->tf_internal * 12.0;
    b[35] = 20.0 * tmp_3;
    Ball_and_Plate_MicroLa_mldivide(b, localDW->coeffs);
  }

  if (localDW->t_elapsed <= localDW->tf_internal) {
    /* '<S26>:1:52' */
    /* '<S26>:1:54' */
    path = ((((localDW->coeffs[1] * localDW->t_elapsed + localDW->coeffs[0]) +
              localDW->t_elapsed * localDW->t_elapsed * localDW->coeffs[2]) +
             localDW->coeffs[3] * rt_powd_snf(localDW->t_elapsed, 3.0)) +
            localDW->coeffs[4] * rt_powd_snf(localDW->t_elapsed, 4.0)) +
      localDW->coeffs[5] * rt_powd_snf(localDW->t_elapsed, 5.0);

    /* '<S26>:1:57' */
    localDW->t_elapsed += rtu_Ts;
  } else {
    /* '<S26>:1:60' */
    path = rtu_reference_end;
  }

  /* '<S26>:1:64' */
  localDW->prev_ref_end = rtu_reference_end;
  localB->path = path;
}

/*
 * System initialize for atomic system:
 *    '<S27>/MATLAB Function'
 *    '<S42>/MATLAB Function'
 *    '<S57>/MATLAB Function'
 *    '<S58>/MATLAB Function'
 */
void Ball_and__MATLABFunction_o_Init(DW_MATLABFunction_Ball_and__j_T *localDW)
{
  localDW->index = 1.0;
  localDW->previous_enable = 0.0;
}

/*
 * Output and update for atomic system:
 *    '<S27>/MATLAB Function'
 *    '<S42>/MATLAB Function'
 *    '<S57>/MATLAB Function'
 *    '<S58>/MATLAB Function'
 */
void Ball_and_Plate_MATLABFunction_e(real_T rtu_enable, const real_T rtu_u
  [200000], B_MATLABFunction_Ball_and_P_m_T *localB,
  DW_MATLABFunction_Ball_and__j_T *localDW)
{
  /* MATLAB Function 'Innerloop_Actuator/Motor_A/Multisine/MATLAB Function': '<S33>:1' */
  /* '<S33>:1:14' */
  if ((rtu_enable != 0.0) && (!(localDW->previous_enable != 0.0))) {
    /* '<S33>:1:17' */
    /* '<S33>:1:18' */
    localDW->index = 1.0;
  }

  if (rtu_enable != 0.0) {
    if (localDW->index <= 200000.0) {
      /* '<S33>:1:25' */
      /* '<S33>:1:26' */
      localB->y = rtu_u[(int32_T)localDW->index - 1];

      /* '<S33>:1:27' */
      localDW->index++;
    } else {
      /* '<S33>:1:31' */
      localB->y = 0.0;
    }
  } else {
    /* '<S33>:1:36' */
    localB->y = 0.0;
  }

  /* '<S33>:1:39' */
  localDW->previous_enable = rtu_enable;
}

/*
 * System initialize for enable system:
 *    '<S37>/Enabled Subsystem'
 *    '<S52>/Enabled Subsystem'
 *    '<S69>/Enabled Subsystem'
 */
void Ball_and__EnabledSubsystem_Init(B_EnabledSubsystem_Ball_and_P_T *localB,
  P_EnabledSubsystem_Ball_and_P_T *localP)
{
  /* SystemInitialize for Outport: '<S38>/Out1' */
  localB->OutportBufferForOut1 = localP->Out1_Y0;
}

/*
 * Disable for enable system:
 *    '<S37>/Enabled Subsystem'
 *    '<S52>/Enabled Subsystem'
 *    '<S69>/Enabled Subsystem'
 */
void Ball_a_EnabledSubsystem_Disable(B_EnabledSubsystem_Ball_and_P_T *localB,
  DW_EnabledSubsystem_Ball_and__T *localDW, P_EnabledSubsystem_Ball_and_P_T
  *localP)
{
  /* Disable for Outport: '<S38>/Out1' */
  localB->OutportBufferForOut1 = localP->Out1_Y0;
  localDW->EnabledSubsystem_MODE = false;
}

/*
 * Start for enable system:
 *    '<S37>/Enabled Subsystem'
 *    '<S52>/Enabled Subsystem'
 *    '<S69>/Enabled Subsystem'
 */
void Ball_and_EnabledSubsystem_Start(DW_EnabledSubsystem_Ball_and__T *localDW)
{
  localDW->EnabledSubsystem_MODE = false;
}

/*
 * Output and update for enable system:
 *    '<S37>/Enabled Subsystem'
 *    '<S52>/Enabled Subsystem'
 *    '<S69>/Enabled Subsystem'
 */
void Ball_and_Plate_EnabledSubsystem
  (RT_MODEL_Ball_and_Plate_MicroLabBox_student_T * const
   Ball_and_Plate_MicroLabBox_student_M, real_T rtu_Enable,
   B_EnabledSubsystem_Ball_and_P_T *localB, DW_EnabledSubsystem_Ball_and__T
   *localDW, P_EnabledSubsystem_Ball_and_P_T *localP)
{
  /* Outputs for Enabled SubSystem: '<S37>/Enabled Subsystem' incorporates:
   *  EnablePort: '<S38>/Enable'
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
    /* SignalConversion generated from: '<S38>/Out1' incorporates:
     *  Constant: '<S38>/Constant'
     */
    localB->OutportBufferForOut1 = localP->Constant_Value;
  }

  /* End of Outputs for SubSystem: '<S37>/Enabled Subsystem' */
}

/*
 * System initialize for enable system:
 *    '<S37>/Enabled Subsystem1'
 *    '<S52>/Enabled Subsystem1'
 *    '<S69>/Enabled Subsystem1'
 */
void Ball_and_EnabledSubsystem1_Init(B_EnabledSubsystem1_Ball_and__T *localB,
  P_EnabledSubsystem1_Ball_and__T *localP)
{
  /* SystemInitialize for Outport: '<S39>/Out1' */
  localB->In1 = localP->Out1_Y0;
}

/*
 * Disable for enable system:
 *    '<S37>/Enabled Subsystem1'
 *    '<S52>/Enabled Subsystem1'
 *    '<S69>/Enabled Subsystem1'
 */
void Ball__EnabledSubsystem1_Disable(DW_EnabledSubsystem1_Ball_and_T *localDW)
{
  localDW->EnabledSubsystem1_MODE = false;
}

/*
 * Start for enable system:
 *    '<S37>/Enabled Subsystem1'
 *    '<S52>/Enabled Subsystem1'
 *    '<S69>/Enabled Subsystem1'
 */
void Ball_an_EnabledSubsystem1_Start(DW_EnabledSubsystem1_Ball_and_T *localDW)
{
  localDW->EnabledSubsystem1_MODE = false;
}

/*
 * Output and update for enable system:
 *    '<S37>/Enabled Subsystem1'
 *    '<S52>/Enabled Subsystem1'
 *    '<S69>/Enabled Subsystem1'
 */
void Ball_and_Plat_EnabledSubsystem1
  (RT_MODEL_Ball_and_Plate_MicroLabBox_student_T * const
   Ball_and_Plate_MicroLabBox_student_M, boolean_T rtu_Enable, real_T rtu_In1,
   B_EnabledSubsystem1_Ball_and__T *localB, DW_EnabledSubsystem1_Ball_and_T
   *localDW)
{
  /* Outputs for Enabled SubSystem: '<S37>/Enabled Subsystem1' incorporates:
   *  EnablePort: '<S39>/Enable'
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
    /* Inport: '<S39>/In1' */
    localB->In1 = rtu_In1;
  }

  /* End of Outputs for SubSystem: '<S37>/Enabled Subsystem1' */
}

/* Function for MATLAB Function: '<S5>/AngleToPos ' */
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

/* Function for MATLAB Function: '<S92>/optimizer' */
static void Ball_and_Plate_Mi_Unconstrained(const real_T b_Hinv[49], const
  real_T f[7], real_T x[7], int16_T n)
{
  int32_T i;
  real_T b_Hinv_0;
  int32_T i_0;
  int32_T i_1;
  for (i = 1; i - 1 < n; i++) {
    i_0 = (int16_T)i;
    b_Hinv_0 = 0.0;
    for (i_1 = 0; i_1 < 7; i_1++) {
      b_Hinv_0 += -b_Hinv[(7 * i_1 + i_0) - 1] * f[i_1];
    }

    x[(int16_T)i - 1] = b_Hinv_0;
  }
}

/* Function for MATLAB Function: '<S92>/optimizer' */
static real_T Ball_and_Plate_MicroLabB_norm_i(const real_T x[7])
{
  real_T y;
  real_T scale;
  real_T absxk;
  real_T t;
  int32_T k;
  y = 0.0;
  scale = 3.3121686421112381E-170;
  for (k = 0; k < 7; k++) {
    absxk = fabs(x[k]);
    if (absxk > scale) {
      t = scale / absxk;
      y = y * t * t + 1.0;
      scale = absxk;
    } else {
      t = absxk / scale;
      y += t * t;
    }
  }

  return scale * sqrt(y);
}

/* Function for MATLAB Function: '<S92>/optimizer' */
static void Ball_and_Plate_MicroLabBox__abs(const real_T x[7], real_T y[7])
{
  int32_T k;
  for (k = 0; k < 7; k++) {
    y[k] = fabs(x[k]);
  }
}

/* Function for MATLAB Function: '<S92>/optimizer' */
static void Ball_and_Plate_MicroLabBo_abs_f(const real_T x[212], real_T y[212])
{
  int32_T k;
  for (k = 0; k < 212; k++) {
    y[k] = fabs(x[k]);
  }
}

/* Function for MATLAB Function: '<S92>/optimizer' */
static real_T Ball_and_Plate_MicroLabBo_xnrm2(int32_T n, const real_T x[49],
  int32_T ix0)
{
  real_T y;
  real_T scale;
  int32_T kend;
  real_T absxk;
  real_T t;
  int32_T k;
  y = 0.0;
  if (n >= 1) {
    if (n == 1) {
      y = fabs(x[ix0 - 1]);
    } else {
      scale = 3.3121686421112381E-170;
      kend = (ix0 + n) - 1;
      for (k = ix0; k <= kend; k++) {
        absxk = fabs(x[k - 1]);
        if (absxk > scale) {
          t = scale / absxk;
          y = y * t * t + 1.0;
          scale = absxk;
        } else {
          t = absxk / scale;
          y += t * t;
        }
      }

      y = scale * sqrt(y);
    }
  }

  return y;
}

real_T rt_hypotd_snf(real_T u0, real_T u1)
{
  real_T y;
  real_T a;
  a = fabs(u0);
  y = fabs(u1);
  if (a < y) {
    a /= y;
    y *= sqrt(a * a + 1.0);
  } else if (a > y) {
    y /= a;
    y = sqrt(y * y + 1.0) * a;
  } else {
    if (!rtIsNaN(y)) {
      y = a * 1.4142135623730951;
    }
  }

  return y;
}

/* Function for MATLAB Function: '<S92>/optimizer' */
static void Ball_and_Plate_MicroLabBo_xgemv(int32_T m, int32_T n, const real_T
  b_A[49], int32_T ia0, const real_T x[49], int32_T ix0, real_T y[7])
{
  int32_T ix;
  real_T c;
  int32_T b_iy;
  int32_T b;
  int32_T iac;
  int32_T d;
  int32_T ia;
  if ((m != 0) && (n != 0)) {
    for (b_iy = 0; b_iy < n; b_iy++) {
      y[b_iy] = 0.0;
    }

    b_iy = 0;
    b = (n - 1) * 7 + ia0;
    for (iac = ia0; iac <= b; iac += 7) {
      ix = ix0;
      c = 0.0;
      d = (iac + m) - 1;
      for (ia = iac; ia <= d; ia++) {
        c += b_A[ia - 1] * x[ix - 1];
        ix++;
      }

      y[b_iy] += c;
      b_iy++;
    }
  }
}

/* Function for MATLAB Function: '<S92>/optimizer' */
static void Ball_and_Plate_MicroLabBo_xgerc(int32_T m, int32_T n, real_T alpha1,
  int32_T ix0, const real_T y[7], real_T b_A[49], int32_T ia0)
{
  int32_T jA;
  int32_T jy;
  real_T temp;
  int32_T ix;
  int32_T j;
  int32_T b;
  int32_T ijA;
  if (!(alpha1 == 0.0)) {
    jA = ia0 - 1;
    jy = 0;
    for (j = 0; j < n; j++) {
      if (y[jy] != 0.0) {
        temp = y[jy] * alpha1;
        ix = ix0;
        b = m + jA;
        for (ijA = jA; ijA < b; ijA++) {
          b_A[ijA] += b_A[ix - 1] * temp;
          ix++;
        }
      }

      jy++;
      jA += 7;
    }
  }
}

/* Function for MATLAB Function: '<S92>/optimizer' */
static void Ball_and_Plate_MicroLabBox_s_qr(const real_T b_A[49], real_T Q[49],
  real_T R[49])
{
  real_T c_A[49];
  real_T tau[7];
  real_T work[7];
  int32_T ii;
  real_T b;
  real_T b_atmp;
  real_T xnorm;
  int32_T knt;
  int32_T lastc;
  int32_T coltop;
  int32_T b_coltop;
  int32_T b_ia;
  int32_T i;
  int32_T exitg1;
  boolean_T exitg2;
  for (i = 0; i < 7; i++) {
    tau[i] = 0.0;
  }

  memcpy(&c_A[0], &b_A[0], 49U * sizeof(real_T));
  for (i = 0; i < 7; i++) {
    work[i] = 0.0;
  }

  for (i = 0; i < 7; i++) {
    ii = i * 7 + i;
    if (i + 1 < 7) {
      b_atmp = c_A[ii];
      b = 0.0;
      xnorm = Ball_and_Plate_MicroLabBo_xnrm2(6 - i, c_A, ii + 2);
      if (xnorm != 0.0) {
        xnorm = rt_hypotd_snf(c_A[ii], xnorm);
        if (c_A[ii] >= 0.0) {
          xnorm = -xnorm;
        }

        if (fabs(xnorm) < 1.0020841800044864E-292) {
          knt = -1;
          lastc = ii - i;
          do {
            knt++;
            for (coltop = ii + 1; coltop < lastc + 7; coltop++) {
              c_A[coltop] *= 9.9792015476736E+291;
            }

            xnorm *= 9.9792015476736E+291;
            b_atmp *= 9.9792015476736E+291;
          } while (!(fabs(xnorm) >= 1.0020841800044864E-292));

          xnorm = rt_hypotd_snf(b_atmp, Ball_and_Plate_MicroLabBo_xnrm2(6 - i,
            c_A, ii + 2));
          if (b_atmp >= 0.0) {
            xnorm = -xnorm;
          }

          b = (xnorm - b_atmp) / xnorm;
          b_atmp = 1.0 / (b_atmp - xnorm);
          lastc = ii - i;
          for (coltop = ii + 1; coltop < lastc + 7; coltop++) {
            c_A[coltop] *= b_atmp;
          }

          for (lastc = 0; lastc <= knt; lastc++) {
            xnorm *= 1.0020841800044864E-292;
          }

          b_atmp = xnorm;
        } else {
          b = (xnorm - c_A[ii]) / xnorm;
          b_atmp = 1.0 / (c_A[ii] - xnorm);
          knt = ii - i;
          for (lastc = ii + 1; lastc < knt + 7; lastc++) {
            c_A[lastc] *= b_atmp;
          }

          b_atmp = xnorm;
        }
      }

      tau[i] = b;
      c_A[ii] = b_atmp;
      b_atmp = c_A[ii];
      c_A[ii] = 1.0;
      if (tau[i] != 0.0) {
        knt = 7 - i;
        lastc = ii - i;
        while ((knt > 0) && (c_A[lastc + 6] == 0.0)) {
          knt--;
          lastc--;
        }

        lastc = 6 - i;
        exitg2 = false;
        while ((!exitg2) && (lastc > 0)) {
          coltop = ((lastc - 1) * 7 + ii) + 7;
          b_coltop = coltop;
          do {
            exitg1 = 0;
            if (b_coltop + 1 <= coltop + knt) {
              if (c_A[b_coltop] != 0.0) {
                exitg1 = 1;
              } else {
                b_coltop++;
              }
            } else {
              lastc--;
              exitg1 = 2;
            }
          } while (exitg1 == 0);

          if (exitg1 == 1) {
            exitg2 = true;
          }
        }
      } else {
        knt = 0;
        lastc = 0;
      }

      if (knt > 0) {
        Ball_and_Plate_MicroLabBo_xgemv(knt, lastc, c_A, ii + 8, c_A, ii + 1,
          work);
        Ball_and_Plate_MicroLabBo_xgerc(knt, lastc, -tau[i], ii + 1, work, c_A,
          ii + 8);
      }

      c_A[ii] = b_atmp;
    } else {
      tau[6] = 0.0;
    }
  }

  for (i = 0; i < 7; i++) {
    for (ii = 0; ii <= i; ii++) {
      R[ii + 7 * i] = c_A[7 * i + ii];
    }

    for (ii = i + 1; ii + 1 < 8; ii++) {
      R[ii + 7 * i] = 0.0;
    }

    work[i] = 0.0;
  }

  i = 6;
  for (ii = 6; ii >= 0; ii--) {
    knt = (ii * 7 + ii) + 8;
    if (ii + 1 < 7) {
      c_A[knt - 8] = 1.0;
      if (tau[i] != 0.0) {
        lastc = 7 - ii;
        coltop = knt - ii;
        while ((lastc > 0) && (c_A[coltop - 2] == 0.0)) {
          lastc--;
          coltop--;
        }

        coltop = 6 - ii;
        exitg2 = false;
        while ((!exitg2) && (coltop > 0)) {
          b_coltop = (coltop - 1) * 7 + knt;
          b_ia = b_coltop;
          do {
            exitg1 = 0;
            if (b_ia <= (b_coltop + lastc) - 1) {
              if (c_A[b_ia - 1] != 0.0) {
                exitg1 = 1;
              } else {
                b_ia++;
              }
            } else {
              coltop--;
              exitg1 = 2;
            }
          } while (exitg1 == 0);

          if (exitg1 == 1) {
            exitg2 = true;
          }
        }
      } else {
        lastc = 0;
        coltop = 0;
      }

      if (lastc > 0) {
        Ball_and_Plate_MicroLabBo_xgemv(lastc, coltop, c_A, knt, c_A, knt - 7,
          work);
        Ball_and_Plate_MicroLabBo_xgerc(lastc, coltop, -tau[i], knt - 7, work,
          c_A, knt);
      }

      lastc = (knt - ii) - 1;
      for (coltop = knt - 7; coltop < lastc; coltop++) {
        c_A[coltop] *= -tau[i];
      }
    }

    c_A[knt - 8] = 1.0 - tau[i];
    for (lastc = 0; lastc < ii; lastc++) {
      c_A[(knt - lastc) - 9] = 0.0;
    }

    i--;
  }

  for (i = 0; i < 7; i++) {
    for (ii = 0; ii < 7; ii++) {
      Q[ii + 7 * i] = c_A[7 * i + ii];
    }
  }
}

/* Function for MATLAB Function: '<S92>/optimizer' */
static real_T Ball_and_Plate_Micro_KWIKfactor(const real_T b_Ac[1484], const
  int16_T iC[212], int16_T nA, const real_T b_Linv[49], real_T RLinv[49], real_T
  D[49], real_T b_H[49], int16_T n)
{
  real_T Status;
  real_T TL[49];
  real_T QQ[49];
  real_T RR[49];
  int32_T i;
  int16_T b_j;
  int16_T c_k;
  int32_T f_i;
  real_T b_Linv_0;
  int32_T i_0;
  int32_T f_i_0;
  int32_T i_1;
  int32_T exitg1;
  Status = 1.0;
  memset(&RLinv[0], 0, 49U * sizeof(real_T));
  for (i = 1; i - 1 < nA; i++) {
    f_i = iC[(int16_T)i - 1];
    i_0 = (int16_T)i - 1;
    for (i_1 = 0; i_1 < 7; i_1++) {
      RLinv[i_1 + 7 * i_0] = 0.0;
      for (f_i_0 = 0; f_i_0 < 7; f_i_0++) {
        RLinv[i_1 + 7 * i_0] += b_Ac[(212 * f_i_0 + f_i) - 1] * b_Linv[7 * f_i_0
          + i_1];
      }
    }
  }

  Ball_and_Plate_MicroLabBox_s_qr(RLinv, QQ, RR);
  i = 1;
  do {
    exitg1 = 0;
    if (i - 1 <= nA - 1) {
      if (fabs(RR[(((int16_T)i - 1) * 7 + (int16_T)i) - 1]) < 1.0E-12) {
        Status = -2.0;
        exitg1 = 1;
      } else {
        i++;
      }
    } else {
      for (i = 1; i - 1 < n; i++) {
        for (f_i = 1; f_i - 1 < n; f_i++) {
          i_0 = (int16_T)i;
          f_i_0 = (int16_T)f_i;
          b_Linv_0 = 0.0;
          for (i_1 = 0; i_1 < 7; i_1++) {
            b_Linv_0 += b_Linv[(i_0 - 1) * 7 + i_1] * QQ[(f_i_0 - 1) * 7 + i_1];
          }

          TL[((int16_T)i + 7 * ((int16_T)f_i - 1)) - 1] = b_Linv_0;
        }
      }

      memset(&RLinv[0], 0, 49U * sizeof(real_T));
      for (b_j = nA; b_j > 0; b_j--) {
        RLinv[(b_j + 7 * (b_j - 1)) - 1] = 1.0;
        for (c_k = b_j; c_k <= nA; c_k++) {
          RLinv[(b_j + 7 * (c_k - 1)) - 1] /= RR[((b_j - 1) * 7 + b_j) - 1];
        }

        if (b_j > 1) {
          for (i = 1; i - 1 <= b_j - 2; i++) {
            for (c_k = b_j; c_k <= nA; c_k++) {
              RLinv[((int16_T)i + 7 * (c_k - 1)) - 1] -= RR[((b_j - 1) * 7 +
                (int16_T)i) - 1] * RLinv[((c_k - 1) * 7 + b_j) - 1];
            }
          }
        }
      }

      for (i = 1; i - 1 < n; i++) {
        for (b_j = (int16_T)i; b_j <= n; b_j++) {
          b_H[((int16_T)i + 7 * (b_j - 1)) - 1] = 0.0;
          i_1 = nA + 1;
          if (i_1 > 32767) {
            i_1 = 32767;
          }

          for (c_k = (int16_T)i_1; c_k <= n; c_k++) {
            b_H[((int16_T)i + 7 * (b_j - 1)) - 1] -= TL[((c_k - 1) * 7 +
              (int16_T)i) - 1] * TL[((c_k - 1) * 7 + b_j) - 1];
          }

          b_H[(b_j + 7 * ((int16_T)i - 1)) - 1] = b_H[((b_j - 1) * 7 + (int16_T)
            i) - 1];
        }
      }

      for (i = 1; i - 1 < nA; i++) {
        for (f_i = 1; f_i - 1 < n; f_i++) {
          D[((int16_T)f_i + 7 * ((int16_T)i - 1)) - 1] = 0.0;
          for (b_j = (int16_T)i; b_j <= nA; b_j++) {
            D[((int16_T)f_i + 7 * ((int16_T)i - 1)) - 1] += TL[((b_j - 1) * 7 +
              (int16_T)f_i) - 1] * RLinv[((b_j - 1) * 7 + (int16_T)i) - 1];
          }
        }
      }

      exitg1 = 1;
    }
  } while (exitg1 == 0);

  return Status;
}

/* Function for MATLAB Function: '<S92>/optimizer' */
static void Ball_and_Plate_M_DropConstraint(int16_T kDrop, int16_T iA[212],
  int16_T *nA, int16_T iC[212])
{
  int16_T b;
  int16_T i;
  int32_T tmp;
  iA[iC[kDrop - 1] - 1] = 0;
  if (kDrop < *nA) {
    tmp = *nA - 1;
    if (tmp < -32768) {
      tmp = -32768;
    }

    b = (int16_T)tmp;
    for (i = kDrop; i <= b; i++) {
      iC[i - 1] = iC[i];
    }
  }

  iC[*nA - 1] = 0;
  tmp = *nA - 1;
  if (tmp < -32768) {
    tmp = -32768;
  }

  *nA = (int16_T)tmp;
}

/* Function for MATLAB Function: '<S92>/optimizer' */
static void Ball_and_Plate_MicroLabB_qpkwik(const real_T b_Linv[49], const
  real_T b_Hinv[49], const real_T f[7], const real_T b_Ac[1484], const real_T b
  [212], int16_T iA[212], int16_T b_maxiter, real_T FeasTol, real_T x[7], real_T
  lambda[212], real_T *status)
{
  real_T r[7];
  real_T rMin;
  real_T RLinv[49];
  real_T D[49];
  real_T b_H[49];
  real_T U[49];
  real_T cTol[212];
  boolean_T cTolComputed;
  int16_T iC[212];
  int16_T nA;
  real_T Opt[14];
  real_T Rhs[14];
  boolean_T DualFeasible;
  boolean_T ColdReset;
  int16_T kDrop;
  real_T Xnorm0;
  real_T cMin;
  int16_T kNext;
  real_T cVal;
  real_T AcRow[7];
  real_T z[7];
  real_T t;
  int16_T iSave;
  real_T varargin_1[212];
  int32_T idx;
  uint16_T q;
  uint16_T b_x;
  int32_T e_k;
  int32_T i;
  real_T b_Ac_0[7];
  int32_T i_0;
  int32_T tmp;
  int32_T exitg1;
  int32_T exitg2;
  int32_T exitg3;
  boolean_T exitg4;
  boolean_T guard1 = false;
  boolean_T guard2 = false;
  *status = 1.0;
  memset(&lambda[0], 0, 212U * sizeof(real_T));
  for (i = 0; i < 7; i++) {
    x[i] = 0.0;
    r[i] = 0.0;
  }

  rMin = 0.0;
  cTolComputed = false;
  for (i = 0; i < 212; i++) {
    cTol[i] = 1.0;
    iC[i] = 0;
  }

  nA = 0;
  for (i = 0; i < 212; i++) {
    if (iA[i] == 1) {
      e_k = nA + 1;
      if (e_k > 32767) {
        e_k = 32767;
      }

      nA = (int16_T)e_k;
      iC[nA - 1] = (int16_T)(i + 1);
    }
  }

  guard1 = false;
  if (nA > 0) {
    memset(&Opt[0], 0, 14U * sizeof(real_T));
    for (i = 0; i < 7; i++) {
      Rhs[i] = f[i];
      Rhs[i + 7] = 0.0;
    }

    DualFeasible = false;
    e_k = 3 * nA;
    if (e_k > 32767) {
      e_k = 32767;
    }

    kNext = (int16_T)e_k;
    if (kNext <= 50) {
      kNext = 50;
    }

    q = (uint16_T)(kNext / 10U);
    b_x = (uint16_T)((uint32_T)kNext - q * 10);
    if ((b_x > 0) && (b_x >= 5)) {
      q++;
    }

    ColdReset = false;
    do {
      exitg3 = 0;
      if ((!DualFeasible) && (nA > 0) && ((int32_T)*status <= b_maxiter)) {
        Xnorm0 = Ball_and_Plate_Micro_KWIKfactor(b_Ac, iC, nA, b_Linv, RLinv, D,
          b_H, 7);
        if (Xnorm0 < 0.0) {
          if (ColdReset) {
            *status = -2.0;
            exitg3 = 2;
          } else {
            nA = 0;
            memset(&iA[0], 0, 212U * sizeof(int16_T));
            memset(&iC[0], 0, 212U * sizeof(int16_T));
            ColdReset = true;
          }
        } else {
          for (i = 1; i - 1 < nA; i++) {
            e_k = (int16_T)i + 7;
            if (e_k > 32767) {
              e_k = 32767;
            }

            Rhs[e_k - 1] = b[iC[(int16_T)i - 1] - 1];
            for (kNext = (int16_T)i; kNext <= nA; kNext++) {
              U[(kNext + 7 * ((int16_T)i - 1)) - 1] = 0.0;
              for (idx = 1; idx - 1 < nA; idx++) {
                U[(kNext + 7 * ((int16_T)i - 1)) - 1] += RLinv[(((int16_T)idx -
                  1) * 7 + kNext) - 1] * RLinv[(((int16_T)idx - 1) * 7 +
                  (int16_T)i) - 1];
              }

              U[((int16_T)i + 7 * (kNext - 1)) - 1] = U[(((int16_T)i - 1) * 7 +
                kNext) - 1];
            }
          }

          for (i = 0; i < 7; i++) {
            idx = i + 1;
            Xnorm0 = 0.0;
            for (e_k = 0; e_k < 7; e_k++) {
              Xnorm0 += b_H[(7 * e_k + idx) - 1] * Rhs[e_k];
            }

            Opt[i] = Xnorm0;
            for (idx = 1; idx - 1 < nA; idx++) {
              e_k = (int16_T)idx + 7;
              if (e_k > 32767) {
                e_k = 32767;
              }

              Opt[i] += D[((int16_T)idx - 1) * 7 + i] * Rhs[e_k - 1];
            }
          }

          for (i = 1; i - 1 < nA; i++) {
            idx = (int16_T)i;
            Xnorm0 = 0.0;
            for (e_k = 0; e_k < 7; e_k++) {
              Xnorm0 += D[(idx - 1) * 7 + e_k] * Rhs[e_k];
            }

            e_k = (int16_T)i + 7;
            if (e_k > 32767) {
              e_k = 32767;
            }

            Opt[e_k - 1] = Xnorm0;
            for (idx = 1; idx - 1 < nA; idx++) {
              e_k = (int16_T)i + 7;
              if (e_k > 32767) {
                e_k = 32767;
              }

              i_0 = (int16_T)i + 7;
              if (i_0 > 32767) {
                i_0 = 32767;
              }

              tmp = (int16_T)idx + 7;
              if (tmp > 32767) {
                tmp = 32767;
              }

              Opt[e_k - 1] = U[(((int16_T)idx - 1) * 7 + (int16_T)i) - 1] *
                Rhs[tmp - 1] + Opt[i_0 - 1];
            }
          }

          Xnorm0 = -1.0E-12;
          kDrop = 0;
          for (i = 1; i - 1 < nA; i++) {
            e_k = (int16_T)i + 7;
            if (e_k > 32767) {
              e_k = 32767;
            }

            lambda[iC[(int16_T)i - 1] - 1] = Opt[e_k - 1];
            e_k = (int16_T)i + 7;
            if (e_k > 32767) {
              e_k = 32767;
            }

            if ((Opt[e_k - 1] < Xnorm0) && ((int16_T)i <= nA)) {
              kDrop = (int16_T)i;
              e_k = (int16_T)i + 7;
              if (e_k > 32767) {
                e_k = 32767;
              }

              Xnorm0 = Opt[e_k - 1];
            }
          }

          if (kDrop <= 0) {
            DualFeasible = true;
            for (i = 0; i < 7; i++) {
              x[i] = Opt[i];
            }
          } else {
            (*status)++;
            if ((int32_T)*status > q) {
              nA = 0;
              memset(&iA[0], 0, 212U * sizeof(int16_T));
              memset(&iC[0], 0, 212U * sizeof(int16_T));
              ColdReset = true;
            } else {
              lambda[iC[kDrop - 1] - 1] = 0.0;
              Ball_and_Plate_M_DropConstraint(kDrop, iA, &nA, iC);
            }
          }
        }
      } else {
        if (nA <= 0) {
          memset(&lambda[0], 0, 212U * sizeof(real_T));
          Ball_and_Plate_Mi_Unconstrained(b_Hinv, f, x, 7);
        }

        exitg3 = 1;
      }
    } while (exitg3 == 0);

    if (exitg3 == 1) {
      guard1 = true;
    }
  } else {
    Ball_and_Plate_Mi_Unconstrained(b_Hinv, f, x, 7);
    guard1 = true;
  }

  if (guard1) {
    Xnorm0 = Ball_and_Plate_MicroLabB_norm_i(x);
    do {
      exitg2 = 0;
      if ((int32_T)*status <= b_maxiter) {
        cMin = -FeasTol;
        kNext = 0;
        for (i = 0; i < 212; i++) {
          if (!cTolComputed) {
            idx = i + 1;
            for (e_k = 0; e_k < 7; e_k++) {
              b_Ac_0[e_k] = b_Ac[(212 * e_k + idx) - 1] * x[e_k];
            }

            Ball_and_Plate_MicroLabBox__abs(b_Ac_0, AcRow);
            if (!rtIsNaN(AcRow[0])) {
              idx = 1;
            } else {
              idx = 0;
              e_k = 2;
              exitg4 = false;
              while ((!exitg4) && (e_k < 8)) {
                if (!rtIsNaN(AcRow[e_k - 1])) {
                  idx = e_k;
                  exitg4 = true;
                } else {
                  e_k++;
                }
              }
            }

            if (idx == 0) {
              cVal = AcRow[0];
            } else {
              cVal = AcRow[idx - 1];
              while (idx + 1 <= 7) {
                if (cVal < AcRow[idx]) {
                  cVal = AcRow[idx];
                }

                idx++;
              }
            }

            t = cTol[i];
            if ((t > cVal) || rtIsNaN(cVal)) {
              cVal = t;
            }

            cTol[i] = cVal;
          }

          if (iA[i] == 0) {
            idx = i + 1;
            cVal = 0.0;
            for (e_k = 0; e_k < 7; e_k++) {
              cVal += b_Ac[(212 * e_k + idx) - 1] * x[e_k];
            }

            cVal = (cVal - b[i]) / cTol[i];
            if (cVal < cMin) {
              cMin = cVal;
              kNext = (int16_T)(i + 1);
            }
          }
        }

        cTolComputed = true;
        if (kNext <= 0) {
          exitg2 = 1;
        } else {
          do {
            exitg1 = 0;
            if ((kNext > 0) && ((int32_T)*status <= b_maxiter)) {
              i = kNext;
              for (e_k = 0; e_k < 7; e_k++) {
                AcRow[e_k] = b_Ac[(212 * e_k + i) - 1];
              }

              guard2 = false;
              if (nA == 0) {
                i = kNext;
                for (e_k = 0; e_k < 7; e_k++) {
                  z[e_k] = 0.0;
                  for (i_0 = 0; i_0 < 7; i_0++) {
                    cMin = z[e_k];
                    cMin += b_Ac[(212 * i_0 + i) - 1] * b_Hinv[7 * i_0 + e_k];
                    z[e_k] = cMin;
                  }
                }

                guard2 = true;
              } else {
                cMin = Ball_and_Plate_Micro_KWIKfactor(b_Ac, iC, nA, b_Linv,
                  RLinv, D, b_H, 7);
                if (cMin <= 0.0) {
                  *status = -2.0;
                  exitg1 = 1;
                } else {
                  for (e_k = 0; e_k < 49; e_k++) {
                    U[e_k] = -b_H[e_k];
                  }

                  for (e_k = 0; e_k < 7; e_k++) {
                    z[e_k] = 0.0;
                    for (i_0 = 0; i_0 < 7; i_0++) {
                      cMin = z[e_k];
                      cMin += U[7 * i_0 + e_k] * AcRow[i_0];
                      z[e_k] = cMin;
                    }
                  }

                  for (i = 1; i - 1 < nA; i++) {
                    idx = (int16_T)i;
                    t = 0.0;
                    for (e_k = 0; e_k < 7; e_k++) {
                      t += D[(idx - 1) * 7 + e_k] * AcRow[e_k];
                    }

                    r[(int16_T)i - 1] = t;
                  }

                  guard2 = true;
                }
              }

              if (guard2) {
                kDrop = 0;
                cMin = 0.0;
                DualFeasible = true;
                ColdReset = true;
                if (nA > 0) {
                  i = 0;
                  exitg4 = false;
                  while ((!exitg4) && (i <= nA - 1)) {
                    if (r[i] >= 1.0E-12) {
                      ColdReset = false;
                      exitg4 = true;
                    } else {
                      i++;
                    }
                  }
                }

                ColdReset = ((nA == 0) || ColdReset);
                if (!ColdReset) {
                  for (i = 1; i - 1 < nA; i++) {
                    if (r[(int16_T)i - 1] > 1.0E-12) {
                      cVal = lambda[iC[(int16_T)i - 1] - 1] / r[(int16_T)i - 1];
                      if ((kDrop == 0) || (cVal < rMin)) {
                        rMin = cVal;
                        kDrop = (int16_T)i;
                      }
                    }
                  }

                  if (kDrop > 0) {
                    cMin = rMin;
                    DualFeasible = false;
                  }
                }

                cVal = 0.0;
                for (e_k = 0; e_k < 7; e_k++) {
                  cVal += z[e_k] * AcRow[e_k];
                }

                if (cVal <= 0.0) {
                  cVal = 0.0;
                  ColdReset = true;
                } else {
                  t = 0.0;
                  for (e_k = 0; e_k < 7; e_k++) {
                    t += AcRow[e_k] * x[e_k];
                  }

                  cVal = (b[kNext - 1] - t) / cVal;
                  ColdReset = false;
                }

                if (DualFeasible && ColdReset) {
                  *status = -1.0;
                  exitg1 = 1;
                } else {
                  if (ColdReset) {
                    t = cMin;
                  } else if (DualFeasible) {
                    t = cVal;
                  } else if ((cMin < cVal) || rtIsNaN(cVal)) {
                    t = cMin;
                  } else {
                    t = cVal;
                  }

                  for (i = 1; i - 1 < nA; i++) {
                    lambda[iC[(int16_T)i - 1] - 1] -= r[(int16_T)i - 1] * t;
                    if ((iC[(int16_T)i - 1] <= 212) && (lambda[iC[(int16_T)i - 1]
                         - 1] < 0.0)) {
                      lambda[iC[(int16_T)i - 1] - 1] = 0.0;
                    }
                  }

                  lambda[kNext - 1] += t;
                  if (t == cMin) {
                    Ball_and_Plate_M_DropConstraint(kDrop, iA, &nA, iC);
                  }

                  if (!ColdReset) {
                    for (e_k = 0; e_k < 7; e_k++) {
                      cMin = x[e_k];
                      cMin += t * z[e_k];
                      x[e_k] = cMin;
                    }

                    if (t == cVal) {
                      if (nA == 7) {
                        *status = -1.0;
                        exitg1 = 1;
                      } else {
                        e_k = nA + 1;
                        if (e_k > 32767) {
                          e_k = 32767;
                        }

                        nA = (int16_T)e_k;
                        iC[nA - 1] = kNext;
                        kDrop = nA;
                        while ((kDrop > 1) && (iC[kDrop - 1] <= iC[kDrop - 2]))
                        {
                          iSave = iC[kDrop - 1];
                          iC[kDrop - 1] = iC[kDrop - 2];
                          iC[kDrop - 2] = iSave;
                          kDrop--;
                        }

                        iA[kNext - 1] = 1;
                        kNext = 0;
                        (*status)++;
                      }
                    } else {
                      (*status)++;
                    }
                  } else {
                    (*status)++;
                  }
                }
              }
            } else {
              cMin = Ball_and_Plate_MicroLabB_norm_i(x);
              if (fabs(cMin - Xnorm0) > 0.001) {
                Xnorm0 = cMin;
                Ball_and_Plate_MicroLabBo_abs_f(b, varargin_1);
                for (i = 0; i < 212; i++) {
                  t = varargin_1[i];
                  if (!(t > 1.0)) {
                    t = 1.0;
                  }

                  cTol[i] = t;
                }

                cTolComputed = false;
              }

              exitg1 = 2;
            }
          } while (exitg1 == 0);

          if (exitg1 == 1) {
            exitg2 = 1;
          }
        }
      } else {
        *status = 0.0;
        exitg2 = 1;
      }
    } while (exitg2 == 0);
  }
}

/* Function for MATLAB Function: '<S92>/optimizer' */
static void Ball_and_Plate_Micr_mpc_solveQP(const real_T xQP[4], const real_T
  b_Kx[24], const real_T b_Kr[600], const real_T rseq[100], const real_T b_Ku1
  [12], const real_T old_u[2], const real_T b_Kv[156], const real_T vseq[26],
  const real_T b_Kut[300], const real_T b_utarget[50], const real_T b_Linv[49],
  const real_T b_Hinv[49], const real_T b_Ac[1484], const real_T Bc[212],
  boolean_T iA[212], real_T zopt[7], real_T f[7], real_T *status)
{
  real_T unusedU0[212];
  int16_T iAnew[212];
  int32_T i;
  real_T b_Kx_0;
  real_T b_Kr_0;
  real_T b_Ku1_0;
  real_T b_Kv_0;
  real_T b_Kut_0;
  int32_T i_0;
  for (i = 0; i < 7; i++) {
    f[i] = 0.0;
  }

  for (i = 0; i < 6; i++) {
    b_Kx_0 = b_Kx[i << 2] * xQP[0];
    b_Kx_0 += b_Kx[(i << 2) + 1] * xQP[1];
    b_Kx_0 += b_Kx[(i << 2) + 2] * xQP[2];
    b_Kx_0 += b_Kx[(i << 2) + 3] * xQP[3];
    b_Kr_0 = 0.0;
    for (i_0 = 0; i_0 < 100; i_0++) {
      b_Kr_0 += b_Kr[100 * i + i_0] * rseq[i_0];
    }

    b_Ku1_0 = b_Ku1[i << 1] * old_u[0];
    b_Ku1_0 += b_Ku1[(i << 1) + 1] * old_u[1];
    b_Kv_0 = 0.0;
    for (i_0 = 0; i_0 < 26; i_0++) {
      b_Kv_0 += b_Kv[26 * i + i_0] * vseq[i_0];
    }

    b_Kut_0 = 0.0;
    for (i_0 = 0; i_0 < 50; i_0++) {
      b_Kut_0 += b_Kut[50 * i + i_0] * b_utarget[i_0];
    }

    f[i] = (((b_Kx_0 + b_Kr_0) + b_Ku1_0) + b_Kv_0) + b_Kut_0;
  }

  for (i = 0; i < 212; i++) {
    iAnew[i] = iA[i];
  }

  Ball_and_Plate_MicroLabB_qpkwik(b_Linv, b_Hinv, f, b_Ac, Bc, iAnew, 876,
    1.0E-6, zopt, unusedU0, status);
  for (i = 0; i < 212; i++) {
    iA[i] = (iAnew[i] != 0);
  }

  if ((*status < 0.0) || (*status == 0.0)) {
    for (i = 0; i < 7; i++) {
      zopt[i] = 0.0;
    }
  }
}

/* Model output function */
void Ball_and_Plate_MicroLabBox_student_output(void)
{
  real_T P1_global[3];
  int32_T z;
  real_T Ad[16];
  real_T x_pred[4];
  real_T P_pred[16];
  real_T S[4];
  real_T K[8];
  real_T y[8];
  int32_T r1;
  int32_T r2;
  real_T a21;
  boolean_T rEQ0;
  real_T old_u[2];
  real_T rseq[100];
  real_T vseq[26];
  real_T zopt[7];
  real_T f[7];
  real_T R_BtoP[9];
  ZCEventType zcEvent;
  real_T T[3];
  real_T T_0[3];
  real_T tmp[156];
  real_T tmp_0[50];
  real_T b_Mlim[212];
  real_T tmp_1;
  real_T tmp_2;
  real_T tmp_3;
  real_T tmp_4[9];
  real_T tmp_5[9];
  real_T tmp_6[9];
  real_T e[9];
  real_T Ad_0[16];
  real_T tmp_7[2];
  real_T b_Mlim_0[212];
  real_T uX_idx_2;
  real_T uX_idx_1;
  real_T uX_idx_0;
  real_T P3_global_idx_2;
  static const int8_T e_0[3] = { 1, 0, 0 };

  static const real_T T_1[3] = { 0.0, 0.0, 0.322 };

  static const real_T b[3] = { 0.17, 0.0, 0.0 };

  static const int8_T b_b[8] = { 1, 0, 0, 0, 0, 0, 1, 0 };

  static const int8_T a[8] = { 1, 0, 0, 0, 0, 1, 0, 0 };

  static const real_T c[3] = { -0.085000000000000075, -0.14722431864335456, 0.0
  };

  static const real_T d[3] = { -0.084999999999999964, 0.14722431864335458, 0.0 };

  static const real_T b_Mx[848] = { -1.0, -0.0, -0.0, -0.0, -1.0, -0.0, -0.0,
    -0.0, -1.0, -0.0, -0.0, -0.0, -1.0, -0.0, -0.0, -0.0, -1.0, -0.0, -0.0, -0.0,
    -1.0, -0.0, -0.0, -0.0, -1.0, -0.0, -0.0, -0.0, -1.0, -0.0, -0.0, -0.0, -1.0,
    -0.0, -0.0, -0.0, -1.0, -0.0, -0.0, -0.0, -1.0, -0.0, -0.0, -0.0, -1.0, -0.0,
    -0.0, -0.0, -1.0, -0.0, -0.0, -0.0, -1.0, -0.0, -0.0, -0.0, -1.0, -0.0, -0.0,
    -0.0, -1.0, -0.0, -0.0, -0.0, -1.0, -0.0, -0.0, -0.0, -1.0, -0.0, -0.0, -0.0,
    -1.0, -0.0, -0.0, -0.0, -1.0, -0.0, -0.0, -0.0, -1.0, -0.0, -0.0, -0.0, -1.0,
    -0.0, -0.0, -0.0, -1.0, -0.0, -0.0, -0.0, -1.0, -0.0, -0.0, -0.0, -1.0, -0.0,
    -0.0, -0.0, 1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0,
    0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0,
    1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0,
    0.0, 1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0, 0.0,
    0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0,
    0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0,
    1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, -0.02, -1.0, -0.0, -0.0, -0.04,
    -1.0, -0.0, -0.0, -0.06, -1.0, -0.0, -0.0, -0.08, -1.0, -0.0, -0.0, -0.1,
    -1.0, -0.0, -0.0, -0.12000000000000001, -1.0, -0.0, -0.0, -0.14, -1.0, -0.0,
    -0.0, -0.16, -1.0, -0.0, -0.0, -0.18, -1.0, -0.0, -0.0, -0.19999999999999998,
    -1.0, -0.0, -0.0, -0.21999999999999997, -1.0, -0.0, -0.0,
    -0.23999999999999996, -1.0, -0.0, -0.0, -0.25999999999999995, -1.0, -0.0,
    -0.0, -0.27999999999999997, -1.0, -0.0, -0.0, -0.3, -1.0, -0.0, -0.0, -0.32,
    -1.0, -0.0, -0.0, -0.34, -1.0, -0.0, -0.0, -0.36000000000000004, -1.0, -0.0,
    -0.0, -0.38000000000000006, -1.0, -0.0, -0.0, -0.40000000000000008, -1.0,
    -0.0, -0.0, -0.4200000000000001, -1.0, -0.0, -0.0, -0.44000000000000011,
    -1.0, -0.0, -0.0, -0.46000000000000013, -1.0, -0.0, -0.0,
    -0.48000000000000015, -1.0, -0.0, -0.0, -0.50000000000000011, -1.0, -0.0,
    -0.0, 0.02, 1.0, 0.0, 0.0, 0.04, 1.0, 0.0, 0.0, 0.06, 1.0, 0.0, 0.0, 0.08,
    1.0, 0.0, 0.0, 0.1, 1.0, 0.0, 0.0, 0.12000000000000001, 1.0, 0.0, 0.0, 0.14,
    1.0, 0.0, 0.0, 0.16, 1.0, 0.0, 0.0, 0.18, 1.0, 0.0, 0.0, 0.19999999999999998,
    1.0, 0.0, 0.0, 0.21999999999999997, 1.0, 0.0, 0.0, 0.23999999999999996, 1.0,
    0.0, 0.0, 0.25999999999999995, 1.0, 0.0, 0.0, 0.27999999999999997, 1.0, 0.0,
    0.0, 0.3, 1.0, 0.0, 0.0, 0.32, 1.0, 0.0, 0.0, 0.34, 1.0, 0.0, 0.0,
    0.36000000000000004, 1.0, 0.0, 0.0, 0.38000000000000006, 1.0, 0.0, 0.0,
    0.40000000000000008, 1.0, 0.0, 0.0, 0.4200000000000001, 1.0, 0.0, 0.0,
    0.44000000000000011, 1.0, 0.0, 0.0, 0.46000000000000013, 1.0, 0.0, 0.0,
    0.48000000000000015, 1.0, 0.0, 0.0, 0.50000000000000011, 1.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, -0.0, -0.0, -1.0,
    -0.0, -0.0, -0.0, -1.0, -0.0, -0.0, -0.0, -1.0, -0.0, -0.0, -0.0, -1.0, -0.0,
    -0.0, -0.0, -1.0, -0.0, -0.0, -0.0, -1.0, -0.0, -0.0, -0.0, -1.0, -0.0, -0.0,
    -0.0, -1.0, -0.0, -0.0, -0.0, -1.0, -0.0, -0.0, -0.0, -1.0, -0.0, -0.0, -0.0,
    -1.0, -0.0, -0.0, -0.0, -1.0, -0.0, -0.0, -0.0, -1.0, -0.0, -0.0, -0.0, -1.0,
    -0.0, -0.0, -0.0, -1.0, -0.0, -0.0, -0.0, -1.0, -0.0, -0.0, -0.0, -1.0, -0.0,
    -0.0, -0.0, -1.0, -0.0, -0.0, -0.0, -1.0, -0.0, -0.0, -0.0, -1.0, -0.0, -0.0,
    -0.0, -1.0, -0.0, -0.0, -0.0, -1.0, -0.0, -0.0, -0.0, -1.0, -0.0, -0.0, -0.0,
    -1.0, -0.0, -0.0, -0.0, -1.0, -0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0, 0.0,
    0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0,
    0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0,
    1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0,
    0.0, 1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0, 0.0,
    0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0,
    0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0,
    1.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, -0.0,
    -0.0, -0.02, -1.0, -0.0, -0.0, -0.04, -1.0, -0.0, -0.0, -0.06, -1.0, -0.0,
    -0.0, -0.08, -1.0, -0.0, -0.0, -0.1, -1.0, -0.0, -0.0, -0.12000000000000001,
    -1.0, -0.0, -0.0, -0.14, -1.0, -0.0, -0.0, -0.16, -1.0, -0.0, -0.0, -0.18,
    -1.0, -0.0, -0.0, -0.19999999999999998, -1.0, -0.0, -0.0,
    -0.21999999999999997, -1.0, -0.0, -0.0, -0.23999999999999996, -1.0, -0.0,
    -0.0, -0.25999999999999995, -1.0, -0.0, -0.0, -0.27999999999999997, -1.0,
    -0.0, -0.0, -0.3, -1.0, -0.0, -0.0, -0.32, -1.0, -0.0, -0.0, -0.34, -1.0,
    -0.0, -0.0, -0.36000000000000004, -1.0, -0.0, -0.0, -0.38000000000000006,
    -1.0, -0.0, -0.0, -0.40000000000000008, -1.0, -0.0, -0.0,
    -0.4200000000000001, -1.0, -0.0, -0.0, -0.44000000000000011, -1.0, -0.0,
    -0.0, -0.46000000000000013, -1.0, -0.0, -0.0, -0.48000000000000015, -1.0,
    -0.0, -0.0, -0.50000000000000011, -1.0, 0.0, 0.0, 0.02, 1.0, 0.0, 0.0, 0.04,
    1.0, 0.0, 0.0, 0.06, 1.0, 0.0, 0.0, 0.08, 1.0, 0.0, 0.0, 0.1, 1.0, 0.0, 0.0,
    0.12000000000000001, 1.0, 0.0, 0.0, 0.14, 1.0, 0.0, 0.0, 0.16, 1.0, 0.0, 0.0,
    0.18, 1.0, 0.0, 0.0, 0.19999999999999998, 1.0, 0.0, 0.0, 0.21999999999999997,
    1.0, 0.0, 0.0, 0.23999999999999996, 1.0, 0.0, 0.0, 0.25999999999999995, 1.0,
    0.0, 0.0, 0.27999999999999997, 1.0, 0.0, 0.0, 0.3, 1.0, 0.0, 0.0, 0.32, 1.0,
    0.0, 0.0, 0.34, 1.0, 0.0, 0.0, 0.36000000000000004, 1.0, 0.0, 0.0,
    0.38000000000000006, 1.0, 0.0, 0.0, 0.40000000000000008, 1.0, 0.0, 0.0,
    0.4200000000000001, 1.0, 0.0, 0.0, 0.44000000000000011, 1.0, 0.0, 0.0,
    0.46000000000000013, 1.0, 0.0, 0.0, 0.48000000000000015, 1.0, 0.0, 0.0,
    0.50000000000000011, 1.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0 };

  static const real_T b_Kx[24] = { 0.0, 0.0, -77428.928571428551,
    -34159.821428571435, 77428.928571428551, 34159.821428571435, 0.0, 0.0, 0.0,
    0.0, -68669.999999999985, -30803.400000000005, 68669.999999999985,
    30803.400000000005, 0.0, 0.0, 0.0, 0.0, -60597.771428571417,
    -27642.898285714287, 60597.771428571417, 27642.898285714287, 0.0, 0.0 };

  static const real_T b_Kr[600] = { -0.0, -0.0, 14.014285714285714,
    14.014285714285716, -0.0, -0.0, 56.057142857142857, 28.028571428571432, -0.0,
    -0.0, 126.12857142857145, 42.042857142857152, -0.0, -0.0, 224.22857142857143,
    56.057142857142864, -0.0, -0.0, 350.35714285714289, 70.071428571428569, -0.0,
    -0.0, 504.51428571428579, 84.085714285714289, -0.0, -0.0, 686.7, 98.1, -0.0,
    -0.0, 896.91428571428594, 112.11428571428573, -0.0, -0.0, 1135.1571428571431,
    126.12857142857143, -0.0, -0.0, 1401.4285714285716, 140.14285714285714, -0.0,
    -0.0, 1695.7285714285713, 154.15714285714287, -0.0, -0.0, 2018.0571428571429,
    168.17142857142858, -0.0, -0.0, 2368.4142857142856, 182.18571428571428, -0.0,
    -0.0, 2746.7999999999997, 196.2, -0.0, -0.0, 3153.2142857142853,
    210.21428571428572, -0.0, -0.0, 3587.6571428571424, 224.22857142857146, -0.0,
    -0.0, 4050.1285714285714, 238.24285714285716, -0.0, -0.0, 4540.6285714285714,
    252.25714285714287, -0.0, -0.0, 5059.1571428571424, 266.2714285714286, -0.0,
    -0.0, 5605.7142857142853, 280.28571428571428, -0.0, -0.0, 6180.2999999999993,
    294.3, -0.0, -0.0, 6782.9142857142851, 308.31428571428575, -0.0, -0.0,
    7413.5571428571438, 322.32857142857142, -0.0, -0.0, 8072.2285714285726,
    336.34285714285716, -0.0, -0.0, 8758.9285714285725, 350.35714285714283,
    -14.014285714285714, -14.014285714285716, -0.0, -0.0, -56.057142857142857,
    -28.028571428571432, -0.0, -0.0, -126.12857142857145, -42.042857142857152,
    -0.0, -0.0, -224.22857142857143, -56.057142857142864, -0.0, -0.0,
    -350.35714285714289, -70.071428571428569, -0.0, -0.0, -504.51428571428579,
    -84.085714285714289, -0.0, -0.0, -686.7, -98.1, -0.0, -0.0,
    -896.91428571428594, -112.11428571428573, -0.0, -0.0, -1135.1571428571431,
    -126.12857142857143, -0.0, -0.0, -1401.4285714285716, -140.14285714285714,
    -0.0, -0.0, -1695.7285714285713, -154.15714285714287, -0.0, -0.0,
    -2018.0571428571429, -168.17142857142858, -0.0, -0.0, -2368.4142857142856,
    -182.18571428571428, -0.0, -0.0, -2746.7999999999997, -196.2, -0.0, -0.0,
    -3153.2142857142853, -210.21428571428572, -0.0, -0.0, -3587.6571428571424,
    -224.22857142857146, -0.0, -0.0, -4050.1285714285714, -238.24285714285716,
    -0.0, -0.0, -4540.6285714285714, -252.25714285714287, -0.0, -0.0,
    -5059.1571428571424, -266.2714285714286, -0.0, -0.0, -5605.7142857142853,
    -280.28571428571428, -0.0, -0.0, -6180.2999999999993, -294.3, -0.0, -0.0,
    -6782.9142857142851, -308.31428571428575, -0.0, -0.0, -7413.5571428571438,
    -322.32857142857142, -0.0, -0.0, -8072.2285714285726, -336.34285714285716,
    -0.0, -0.0, -8758.9285714285725, -350.35714285714283, -0.0, -0.0, -0.0, -0.0,
    -0.0, -0.0, -0.0, -0.0, 14.014285714285714, 14.014285714285716, -0.0, -0.0,
    56.057142857142857, 28.028571428571432, -0.0, -0.0, 126.12857142857145,
    42.042857142857152, -0.0, -0.0, 224.22857142857143, 56.057142857142864, -0.0,
    -0.0, 350.35714285714289, 70.071428571428569, -0.0, -0.0, 504.51428571428579,
    84.085714285714289, -0.0, -0.0, 686.7, 98.1, -0.0, -0.0, 896.91428571428594,
    112.11428571428573, -0.0, -0.0, 1135.1571428571431, 126.12857142857143, -0.0,
    -0.0, 1401.4285714285716, 140.14285714285714, -0.0, -0.0, 1695.7285714285713,
    154.15714285714287, -0.0, -0.0, 2018.0571428571429, 168.17142857142858, -0.0,
    -0.0, 2368.4142857142856, 182.18571428571428, -0.0, -0.0, 2746.7999999999997,
    196.2, -0.0, -0.0, 3153.2142857142853, 210.21428571428572, -0.0, -0.0,
    3587.6571428571424, 224.22857142857146, -0.0, -0.0, 4050.1285714285714,
    238.24285714285716, -0.0, -0.0, 4540.6285714285714, 252.25714285714287, -0.0,
    -0.0, 5059.1571428571424, 266.2714285714286, -0.0, -0.0, 5605.7142857142853,
    280.28571428571428, -0.0, -0.0, 6180.2999999999993, 294.3, -0.0, -0.0,
    6782.9142857142851, 308.31428571428575, -0.0, -0.0, 7413.5571428571438,
    322.32857142857142, -0.0, -0.0, 8072.2285714285726, 336.34285714285716, -0.0,
    -0.0, -0.0, -0.0, -14.014285714285714, -14.014285714285716, -0.0, -0.0,
    -56.057142857142857, -28.028571428571432, -0.0, -0.0, -126.12857142857145,
    -42.042857142857152, -0.0, -0.0, -224.22857142857143, -56.057142857142864,
    -0.0, -0.0, -350.35714285714289, -70.071428571428569, -0.0, -0.0,
    -504.51428571428579, -84.085714285714289, -0.0, -0.0, -686.7, -98.1, -0.0,
    -0.0, -896.91428571428594, -112.11428571428573, -0.0, -0.0,
    -1135.1571428571431, -126.12857142857143, -0.0, -0.0, -1401.4285714285716,
    -140.14285714285714, -0.0, -0.0, -1695.7285714285713, -154.15714285714287,
    -0.0, -0.0, -2018.0571428571429, -168.17142857142858, -0.0, -0.0,
    -2368.4142857142856, -182.18571428571428, -0.0, -0.0, -2746.7999999999997,
    -196.2, -0.0, -0.0, -3153.2142857142853, -210.21428571428572, -0.0, -0.0,
    -3587.6571428571424, -224.22857142857146, -0.0, -0.0, -4050.1285714285714,
    -238.24285714285716, -0.0, -0.0, -4540.6285714285714, -252.25714285714287,
    -0.0, -0.0, -5059.1571428571424, -266.2714285714286, -0.0, -0.0,
    -5605.7142857142853, -280.28571428571428, -0.0, -0.0, -6180.2999999999993,
    -294.3, -0.0, -0.0, -6782.9142857142851, -308.31428571428575, -0.0, -0.0,
    -7413.5571428571438, -322.32857142857142, -0.0, -0.0, -8072.2285714285726,
    -336.34285714285716, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0,
    -0.0, -0.0, -0.0, 14.014285714285714, 14.014285714285716, -0.0, -0.0,
    56.057142857142857, 28.028571428571432, -0.0, -0.0, 126.12857142857145,
    42.042857142857152, -0.0, -0.0, 224.22857142857143, 56.057142857142864, -0.0,
    -0.0, 350.35714285714289, 70.071428571428569, -0.0, -0.0, 504.51428571428579,
    84.085714285714289, -0.0, -0.0, 686.7, 98.1, -0.0, -0.0, 896.91428571428594,
    112.11428571428573, -0.0, -0.0, 1135.1571428571431, 126.12857142857143, -0.0,
    -0.0, 1401.4285714285716, 140.14285714285714, -0.0, -0.0, 1695.7285714285713,
    154.15714285714287, -0.0, -0.0, 2018.0571428571429, 168.17142857142858, -0.0,
    -0.0, 2368.4142857142856, 182.18571428571428, -0.0, -0.0, 2746.7999999999997,
    196.2, -0.0, -0.0, 3153.2142857142853, 210.21428571428572, -0.0, -0.0,
    3587.6571428571424, 224.22857142857146, -0.0, -0.0, 4050.1285714285714,
    238.24285714285716, -0.0, -0.0, 4540.6285714285714, 252.25714285714287, -0.0,
    -0.0, 5059.1571428571424, 266.2714285714286, -0.0, -0.0, 5605.7142857142853,
    280.28571428571428, -0.0, -0.0, 6180.2999999999993, 294.3, -0.0, -0.0,
    6782.9142857142851, 308.31428571428575, -0.0, -0.0, 7413.5571428571438,
    322.32857142857142, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0,
    -14.014285714285714, -14.014285714285716, -0.0, -0.0, -56.057142857142857,
    -28.028571428571432, -0.0, -0.0, -126.12857142857145, -42.042857142857152,
    -0.0, -0.0, -224.22857142857143, -56.057142857142864, -0.0, -0.0,
    -350.35714285714289, -70.071428571428569, -0.0, -0.0, -504.51428571428579,
    -84.085714285714289, -0.0, -0.0, -686.7, -98.1, -0.0, -0.0,
    -896.91428571428594, -112.11428571428573, -0.0, -0.0, -1135.1571428571431,
    -126.12857142857143, -0.0, -0.0, -1401.4285714285716, -140.14285714285714,
    -0.0, -0.0, -1695.7285714285713, -154.15714285714287, -0.0, -0.0,
    -2018.0571428571429, -168.17142857142858, -0.0, -0.0, -2368.4142857142856,
    -182.18571428571428, -0.0, -0.0, -2746.7999999999997, -196.2, -0.0, -0.0,
    -3153.2142857142853, -210.21428571428572, -0.0, -0.0, -3587.6571428571424,
    -224.22857142857146, -0.0, -0.0, -4050.1285714285714, -238.24285714285716,
    -0.0, -0.0, -4540.6285714285714, -252.25714285714287, -0.0, -0.0,
    -5059.1571428571424, -266.2714285714286, -0.0, -0.0, -5605.7142857142853,
    -280.28571428571428, -0.0, -0.0, -6180.2999999999993, -294.3, -0.0, -0.0,
    -6782.9142857142851, -308.31428571428575, -0.0, -0.0, -7413.5571428571438,
    -322.32857142857142, -0.0, -0.0 };

  static const real_T b_Ku1[12] = { 303148.743027449, 0.0, 0.0, 303148.743027449,
    288469.99916571431, 0.0, 0.0, 288469.99916571431, 274010.22189151018, 0.0,
    0.0, 274010.22189151018 };

  static const real_T b_Kut[300] = { -10000.0, -0.0, -10000.0, -0.0, -10000.0,
    -0.0, -10000.0, -0.0, -10000.0, -0.0, -10000.0, -0.0, -10000.0, -0.0,
    -10000.0, -0.0, -10000.0, -0.0, -10000.0, -0.0, -10000.0, -0.0, -10000.0,
    -0.0, -10000.0, -0.0, -10000.0, -0.0, -10000.0, -0.0, -10000.0, -0.0,
    -10000.0, -0.0, -10000.0, -0.0, -10000.0, -0.0, -10000.0, -0.0, -10000.0,
    -0.0, -10000.0, -0.0, -10000.0, -0.0, -10000.0, -0.0, -10000.0, -0.0, -0.0,
    -10000.0, -0.0, -10000.0, -0.0, -10000.0, -0.0, -10000.0, -0.0, -10000.0,
    -0.0, -10000.0, -0.0, -10000.0, -0.0, -10000.0, -0.0, -10000.0, -0.0,
    -10000.0, -0.0, -10000.0, -0.0, -10000.0, -0.0, -10000.0, -0.0, -10000.0,
    -0.0, -10000.0, -0.0, -10000.0, -0.0, -10000.0, -0.0, -10000.0, -0.0,
    -10000.0, -0.0, -10000.0, -0.0, -10000.0, -0.0, -10000.0, -0.0, -10000.0,
    -0.0, -10000.0, -0.0, -10000.0, -0.0, -0.0, -10000.0, -0.0, -10000.0, -0.0,
    -10000.0, -0.0, -10000.0, -0.0, -10000.0, -0.0, -10000.0, -0.0, -10000.0,
    -0.0, -10000.0, -0.0, -10000.0, -0.0, -10000.0, -0.0, -10000.0, -0.0,
    -10000.0, -0.0, -10000.0, -0.0, -10000.0, -0.0, -10000.0, -0.0, -10000.0,
    -0.0, -10000.0, -0.0, -10000.0, -0.0, -10000.0, -0.0, -10000.0, -0.0,
    -10000.0, -0.0, -10000.0, -0.0, -10000.0, -0.0, -10000.0, -0.0, -0.0, -0.0,
    -0.0, -10000.0, -0.0, -10000.0, -0.0, -10000.0, -0.0, -10000.0, -0.0,
    -10000.0, -0.0, -10000.0, -0.0, -10000.0, -0.0, -10000.0, -0.0, -10000.0,
    -0.0, -10000.0, -0.0, -10000.0, -0.0, -10000.0, -0.0, -10000.0, -0.0,
    -10000.0, -0.0, -10000.0, -0.0, -10000.0, -0.0, -10000.0, -0.0, -10000.0,
    -0.0, -10000.0, -0.0, -10000.0, -0.0, -10000.0, -0.0, -10000.0, -0.0,
    -10000.0, -0.0, -10000.0, -0.0, -0.0, -0.0, -0.0, -10000.0, -0.0, -10000.0,
    -0.0, -10000.0, -0.0, -10000.0, -0.0, -10000.0, -0.0, -10000.0, -0.0,
    -10000.0, -0.0, -10000.0, -0.0, -10000.0, -0.0, -10000.0, -0.0, -10000.0,
    -0.0, -10000.0, -0.0, -10000.0, -0.0, -10000.0, -0.0, -10000.0, -0.0,
    -10000.0, -0.0, -10000.0, -0.0, -10000.0, -0.0, -10000.0, -0.0, -10000.0,
    -0.0, -10000.0, -0.0, -10000.0, -0.0, -10000.0, -0.0, -0.0, -0.0, -0.0, -0.0,
    -0.0, -10000.0, -0.0, -10000.0, -0.0, -10000.0, -0.0, -10000.0, -0.0,
    -10000.0, -0.0, -10000.0, -0.0, -10000.0, -0.0, -10000.0, -0.0, -10000.0,
    -0.0, -10000.0, -0.0, -10000.0, -0.0, -10000.0, -0.0, -10000.0, -0.0,
    -10000.0, -0.0, -10000.0, -0.0, -10000.0, -0.0, -10000.0, -0.0, -10000.0,
    -0.0, -10000.0, -0.0, -10000.0, -0.0, -10000.0, -0.0, -10000.0, -0.0,
    -10000.0 };

  static const real_T b_Linv[49] = { 0.0018162353075412718, 0.0,
    -0.0096383282143209913, 0.0, 0.0002179822254296405, 0.0, 0.0, 0.0,
    0.0018162353075412718, 0.0, -0.0096383282143209913, 0.0,
    0.0002179822254296405, 0.0, 0.0, 0.0, 0.010128772806557685, 0.0,
    -0.0098676884454988577, 0.0, 0.0, 0.0, 0.0, 0.0, 0.010128772806557685, 0.0,
    -0.0098676884454988577, 0.0, 0.0, 0.0, 0.0, 0.0, 0.01014725297766496, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.01014725297766496, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.003162277660168379 };

  static const real_T b_Hinv[49] = { 9.6243597709938869E-5, 0.0,
    -9.977541740508833E-5, 0.0, 2.2119207860689542E-6, 0.0, 0.0, 0.0,
    9.6243597709938869E-5, 0.0, -9.977541740508833E-5, 0.0,
    2.2119207860689542E-6, 0.0, -9.977541740508833E-5, 0.0,
    0.0001999633138242941, 0.0, -0.0001001299309612584, 0.0, 0.0, 0.0,
    -9.977541740508833E-5, 0.0, 0.0001999633138242941, 0.0,
    -0.0001001299309612584, 0.0, 2.2119207860689542E-6, 0.0,
    -0.0001001299309612584, 0.0, 0.0001029667429927304, 0.0, 0.0, 0.0,
    2.2119207860689542E-6, 0.0, -0.0001001299309612584, 0.0,
    0.0001029667429927304, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    9.9999999999999974E-6 };

  static const real_T b_Ac[1484] = { -0.0, -0.0, 0.0014014285714285715,
    0.14014285714285715, -0.0, -0.0, 0.0056057142857142859, 0.2802857142857143,
    -0.0, -0.0, 0.012612857142857144, 0.42042857142857148, -0.0, -0.0,
    0.022422857142857144, 0.56057142857142861, -0.0, -0.0, 0.035035714285714288,
    0.70071428571428573, -0.0, -0.0, 0.050451428571428578, 0.84085714285714286,
    -0.0, -0.0, 0.068670000000000009, 0.981, -0.0, -0.0, 0.089691428571428589,
    1.1211428571428572, -0.0, -0.0, 0.1135157142857143, 1.2612857142857143, -0.0,
    -0.0, 0.14014285714285715, 1.4014285714285715, -0.0, -0.0,
    0.16957285714285714, 1.5415714285714286, -0.0, -0.0, 0.20180571428571428,
    1.6817142857142857, -0.0, -0.0, 0.23684142857142856, 1.8218571428571428,
    -0.0, -0.0, 0.27468, 1.962, -0.0, -0.0, 0.31532142857142853,
    2.1021428571428573, -0.0, -0.0, 0.35876571428571424, 2.2422857142857144,
    -0.0, -0.0, 0.40501285714285712, 2.3824285714285716, -0.0, -0.0,
    0.4540628571428571, 2.5225714285714287, -0.0, -0.0, 0.50591571428571425,
    2.6627142857142858, -0.0, -0.0, 0.5605714285714285, 2.8028571428571429, -0.0,
    -0.0, 0.61803, 2.943, -0.0, -0.0, 0.67829142857142855, 3.0831428571428572,
    -0.0, -0.0, 0.74135571428571434, 3.2232857142857143, -0.0, -0.0,
    0.80722285714285724, 3.3634285714285714, -0.0, -0.0, 0.87589285714285725,
    3.5035714285714286, 0.0, 0.0, -0.0014014285714285715, -0.14014285714285715,
    0.0, 0.0, -0.0056057142857142859, -0.2802857142857143, 0.0, 0.0,
    -0.012612857142857144, -0.42042857142857148, 0.0, 0.0, -0.022422857142857144,
    -0.56057142857142861, 0.0, 0.0, -0.035035714285714288, -0.70071428571428573,
    0.0, 0.0, -0.050451428571428578, -0.84085714285714286, 0.0, 0.0,
    -0.068670000000000009, -0.981, 0.0, 0.0, -0.089691428571428589,
    -1.1211428571428572, 0.0, 0.0, -0.1135157142857143, -1.2612857142857143, 0.0,
    0.0, -0.14014285714285715, -1.4014285714285715, 0.0, 0.0,
    -0.16957285714285714, -1.5415714285714286, 0.0, 0.0, -0.20180571428571428,
    -1.6817142857142857, 0.0, 0.0, -0.23684142857142856, -1.8218571428571428,
    0.0, 0.0, -0.27468, -1.962, 0.0, 0.0, -0.31532142857142853,
    -2.1021428571428573, 0.0, 0.0, -0.35876571428571424, -2.2422857142857144,
    0.0, 0.0, -0.40501285714285712, -2.3824285714285716, 0.0, 0.0,
    -0.4540628571428571, -2.5225714285714287, 0.0, 0.0, -0.50591571428571425,
    -2.6627142857142858, 0.0, 0.0, -0.5605714285714285, -2.8028571428571429, 0.0,
    0.0, -0.61803, -2.943, 0.0, 0.0, -0.67829142857142855, -3.0831428571428572,
    0.0, 0.0, -0.74135571428571434, -3.2232857142857143, 0.0, 0.0,
    -0.80722285714285724, -3.3634285714285714, 0.0, 0.0, -0.87589285714285725,
    -3.5035714285714286, -1.0, -0.0, -1.0, -0.0, -1.0, -0.0, 1.0, 0.0, 1.0, 0.0,
    1.0, 0.0, -0.0014014285714285715, -0.14014285714285715, -0.0, -0.0,
    -0.0056057142857142859, -0.2802857142857143, -0.0, -0.0,
    -0.012612857142857144, -0.42042857142857148, -0.0, -0.0,
    -0.022422857142857144, -0.56057142857142861, -0.0, -0.0,
    -0.035035714285714288, -0.70071428571428573, -0.0, -0.0,
    -0.050451428571428578, -0.84085714285714286, -0.0, -0.0,
    -0.068670000000000009, -0.981, -0.0, -0.0, -0.089691428571428589,
    -1.1211428571428572, -0.0, -0.0, -0.1135157142857143, -1.2612857142857143,
    -0.0, -0.0, -0.14014285714285715, -1.4014285714285715, -0.0, -0.0,
    -0.16957285714285714, -1.5415714285714286, -0.0, -0.0, -0.20180571428571428,
    -1.6817142857142857, -0.0, -0.0, -0.23684142857142856, -1.8218571428571428,
    -0.0, -0.0, -0.27468, -1.962, -0.0, -0.0, -0.31532142857142853,
    -2.1021428571428573, -0.0, -0.0, -0.35876571428571424, -2.2422857142857144,
    -0.0, -0.0, -0.40501285714285712, -2.3824285714285716, -0.0, -0.0,
    -0.4540628571428571, -2.5225714285714287, -0.0, -0.0, -0.50591571428571425,
    -2.6627142857142858, -0.0, -0.0, -0.5605714285714285, -2.8028571428571429,
    -0.0, -0.0, -0.61803, -2.943, -0.0, -0.0, -0.67829142857142855,
    -3.0831428571428572, -0.0, -0.0, -0.74135571428571434, -3.2232857142857143,
    -0.0, -0.0, -0.80722285714285724, -3.3634285714285714, -0.0, -0.0,
    -0.87589285714285725, -3.5035714285714286, -0.0, -0.0, 0.0014014285714285715,
    0.14014285714285715, 0.0, 0.0, 0.0056057142857142859, 0.2802857142857143,
    0.0, 0.0, 0.012612857142857144, 0.42042857142857148, 0.0, 0.0,
    0.022422857142857144, 0.56057142857142861, 0.0, 0.0, 0.035035714285714288,
    0.70071428571428573, 0.0, 0.0, 0.050451428571428578, 0.84085714285714286,
    0.0, 0.0, 0.068670000000000009, 0.981, 0.0, 0.0, 0.089691428571428589,
    1.1211428571428572, 0.0, 0.0, 0.1135157142857143, 1.2612857142857143, 0.0,
    0.0, 0.14014285714285715, 1.4014285714285715, 0.0, 0.0, 0.16957285714285714,
    1.5415714285714286, 0.0, 0.0, 0.20180571428571428, 1.6817142857142857, 0.0,
    0.0, 0.23684142857142856, 1.8218571428571428, 0.0, 0.0, 0.27468, 1.962, 0.0,
    0.0, 0.31532142857142853, 2.1021428571428573, 0.0, 0.0, 0.35876571428571424,
    2.2422857142857144, 0.0, 0.0, 0.40501285714285712, 2.3824285714285716, 0.0,
    0.0, 0.4540628571428571, 2.5225714285714287, 0.0, 0.0, 0.50591571428571425,
    2.6627142857142858, 0.0, 0.0, 0.5605714285714285, 2.8028571428571429, 0.0,
    0.0, 0.61803, 2.943, 0.0, 0.0, 0.67829142857142855, 3.0831428571428572, 0.0,
    0.0, 0.74135571428571434, 3.2232857142857143, 0.0, 0.0, 0.80722285714285724,
    3.3634285714285714, 0.0, 0.0, 0.87589285714285725, 3.5035714285714286, 0.0,
    0.0, -0.0, -1.0, -0.0, -1.0, -0.0, -1.0, 0.0, 1.0, 0.0, 1.0, 0.0, 1.0, -0.0,
    -0.0, -0.0, -0.0, -0.0, -0.0, 0.0014014285714285715, 0.14014285714285715,
    -0.0, -0.0, 0.0056057142857142859, 0.2802857142857143, -0.0, -0.0,
    0.012612857142857144, 0.42042857142857148, -0.0, -0.0, 0.022422857142857144,
    0.56057142857142861, -0.0, -0.0, 0.035035714285714288, 0.70071428571428573,
    -0.0, -0.0, 0.050451428571428578, 0.84085714285714286, -0.0, -0.0,
    0.068670000000000009, 0.981, -0.0, -0.0, 0.089691428571428589,
    1.1211428571428572, -0.0, -0.0, 0.1135157142857143, 1.2612857142857143, -0.0,
    -0.0, 0.14014285714285715, 1.4014285714285715, -0.0, -0.0,
    0.16957285714285714, 1.5415714285714286, -0.0, -0.0, 0.20180571428571428,
    1.6817142857142857, -0.0, -0.0, 0.23684142857142856, 1.8218571428571428,
    -0.0, -0.0, 0.27468, 1.962, -0.0, -0.0, 0.31532142857142853,
    2.1021428571428573, -0.0, -0.0, 0.35876571428571424, 2.2422857142857144,
    -0.0, -0.0, 0.40501285714285712, 2.3824285714285716, -0.0, -0.0,
    0.4540628571428571, 2.5225714285714287, -0.0, -0.0, 0.50591571428571425,
    2.6627142857142858, -0.0, -0.0, 0.5605714285714285, 2.8028571428571429, -0.0,
    -0.0, 0.61803, 2.943, -0.0, -0.0, 0.67829142857142855, 3.0831428571428572,
    -0.0, -0.0, 0.74135571428571434, 3.2232857142857143, -0.0, -0.0,
    0.80722285714285724, 3.3634285714285714, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    -0.0014014285714285715, -0.14014285714285715, 0.0, 0.0,
    -0.0056057142857142859, -0.2802857142857143, 0.0, 0.0, -0.012612857142857144,
    -0.42042857142857148, 0.0, 0.0, -0.022422857142857144, -0.56057142857142861,
    0.0, 0.0, -0.035035714285714288, -0.70071428571428573, 0.0, 0.0,
    -0.050451428571428578, -0.84085714285714286, 0.0, 0.0, -0.068670000000000009,
    -0.981, 0.0, 0.0, -0.089691428571428589, -1.1211428571428572, 0.0, 0.0,
    -0.1135157142857143, -1.2612857142857143, 0.0, 0.0, -0.14014285714285715,
    -1.4014285714285715, 0.0, 0.0, -0.16957285714285714, -1.5415714285714286,
    0.0, 0.0, -0.20180571428571428, -1.6817142857142857, 0.0, 0.0,
    -0.23684142857142856, -1.8218571428571428, 0.0, 0.0, -0.27468, -1.962, 0.0,
    0.0, -0.31532142857142853, -2.1021428571428573, 0.0, 0.0,
    -0.35876571428571424, -2.2422857142857144, 0.0, 0.0, -0.40501285714285712,
    -2.3824285714285716, 0.0, 0.0, -0.4540628571428571, -2.5225714285714287, 0.0,
    0.0, -0.50591571428571425, -2.6627142857142858, 0.0, 0.0,
    -0.5605714285714285, -2.8028571428571429, 0.0, 0.0, -0.61803, -2.943, 0.0,
    0.0, -0.67829142857142855, -3.0831428571428572, 0.0, 0.0,
    -0.74135571428571434, -3.2232857142857143, 0.0, 0.0, -0.80722285714285724,
    -3.3634285714285714, -0.0, -0.0, -1.0, -0.0, -1.0, -0.0, 0.0, 0.0, 1.0, 0.0,
    1.0, 0.0, -0.0, -0.0, -0.0, -0.0, -0.0014014285714285715,
    -0.14014285714285715, -0.0, -0.0, -0.0056057142857142859,
    -0.2802857142857143, -0.0, -0.0, -0.012612857142857144, -0.42042857142857148,
    -0.0, -0.0, -0.022422857142857144, -0.56057142857142861, -0.0, -0.0,
    -0.035035714285714288, -0.70071428571428573, -0.0, -0.0,
    -0.050451428571428578, -0.84085714285714286, -0.0, -0.0,
    -0.068670000000000009, -0.981, -0.0, -0.0, -0.089691428571428589,
    -1.1211428571428572, -0.0, -0.0, -0.1135157142857143, -1.2612857142857143,
    -0.0, -0.0, -0.14014285714285715, -1.4014285714285715, -0.0, -0.0,
    -0.16957285714285714, -1.5415714285714286, -0.0, -0.0, -0.20180571428571428,
    -1.6817142857142857, -0.0, -0.0, -0.23684142857142856, -1.8218571428571428,
    -0.0, -0.0, -0.27468, -1.962, -0.0, -0.0, -0.31532142857142853,
    -2.1021428571428573, -0.0, -0.0, -0.35876571428571424, -2.2422857142857144,
    -0.0, -0.0, -0.40501285714285712, -2.3824285714285716, -0.0, -0.0,
    -0.4540628571428571, -2.5225714285714287, -0.0, -0.0, -0.50591571428571425,
    -2.6627142857142858, -0.0, -0.0, -0.5605714285714285, -2.8028571428571429,
    -0.0, -0.0, -0.61803, -2.943, -0.0, -0.0, -0.67829142857142855,
    -3.0831428571428572, -0.0, -0.0, -0.74135571428571434, -3.2232857142857143,
    -0.0, -0.0, -0.80722285714285724, -3.3634285714285714, -0.0, -0.0, 0.0, 0.0,
    0.0, 0.0, 0.0014014285714285715, 0.14014285714285715, 0.0, 0.0,
    0.0056057142857142859, 0.2802857142857143, 0.0, 0.0, 0.012612857142857144,
    0.42042857142857148, 0.0, 0.0, 0.022422857142857144, 0.56057142857142861,
    0.0, 0.0, 0.035035714285714288, 0.70071428571428573, 0.0, 0.0,
    0.050451428571428578, 0.84085714285714286, 0.0, 0.0, 0.068670000000000009,
    0.981, 0.0, 0.0, 0.089691428571428589, 1.1211428571428572, 0.0, 0.0,
    0.1135157142857143, 1.2612857142857143, 0.0, 0.0, 0.14014285714285715,
    1.4014285714285715, 0.0, 0.0, 0.16957285714285714, 1.5415714285714286, 0.0,
    0.0, 0.20180571428571428, 1.6817142857142857, 0.0, 0.0, 0.23684142857142856,
    1.8218571428571428, 0.0, 0.0, 0.27468, 1.962, 0.0, 0.0, 0.31532142857142853,
    2.1021428571428573, 0.0, 0.0, 0.35876571428571424, 2.2422857142857144, 0.0,
    0.0, 0.40501285714285712, 2.3824285714285716, 0.0, 0.0, 0.4540628571428571,
    2.5225714285714287, 0.0, 0.0, 0.50591571428571425, 2.6627142857142858, 0.0,
    0.0, 0.5605714285714285, 2.8028571428571429, 0.0, 0.0, 0.61803, 2.943, 0.0,
    0.0, 0.67829142857142855, 3.0831428571428572, 0.0, 0.0, 0.74135571428571434,
    3.2232857142857143, 0.0, 0.0, 0.80722285714285724, 3.3634285714285714, 0.0,
    0.0, -0.0, -0.0, -0.0, -1.0, -0.0, -1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 1.0, -0.0,
    -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, 0.0014014285714285715,
    0.14014285714285715, -0.0, -0.0, 0.0056057142857142859, 0.2802857142857143,
    -0.0, -0.0, 0.012612857142857144, 0.42042857142857148, -0.0, -0.0,
    0.022422857142857144, 0.56057142857142861, -0.0, -0.0, 0.035035714285714288,
    0.70071428571428573, -0.0, -0.0, 0.050451428571428578, 0.84085714285714286,
    -0.0, -0.0, 0.068670000000000009, 0.981, -0.0, -0.0, 0.089691428571428589,
    1.1211428571428572, -0.0, -0.0, 0.1135157142857143, 1.2612857142857143, -0.0,
    -0.0, 0.14014285714285715, 1.4014285714285715, -0.0, -0.0,
    0.16957285714285714, 1.5415714285714286, -0.0, -0.0, 0.20180571428571428,
    1.6817142857142857, -0.0, -0.0, 0.23684142857142856, 1.8218571428571428,
    -0.0, -0.0, 0.27468, 1.962, -0.0, -0.0, 0.31532142857142853,
    2.1021428571428573, -0.0, -0.0, 0.35876571428571424, 2.2422857142857144,
    -0.0, -0.0, 0.40501285714285712, 2.3824285714285716, -0.0, -0.0,
    0.4540628571428571, 2.5225714285714287, -0.0, -0.0, 0.50591571428571425,
    2.6627142857142858, -0.0, -0.0, 0.5605714285714285, 2.8028571428571429, -0.0,
    -0.0, 0.61803, 2.943, -0.0, -0.0, 0.67829142857142855, 3.0831428571428572,
    -0.0, -0.0, 0.74135571428571434, 3.2232857142857143, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, -0.0014014285714285715, -0.14014285714285715, 0.0,
    0.0, -0.0056057142857142859, -0.2802857142857143, 0.0, 0.0,
    -0.012612857142857144, -0.42042857142857148, 0.0, 0.0, -0.022422857142857144,
    -0.56057142857142861, 0.0, 0.0, -0.035035714285714288, -0.70071428571428573,
    0.0, 0.0, -0.050451428571428578, -0.84085714285714286, 0.0, 0.0,
    -0.068670000000000009, -0.981, 0.0, 0.0, -0.089691428571428589,
    -1.1211428571428572, 0.0, 0.0, -0.1135157142857143, -1.2612857142857143, 0.0,
    0.0, -0.14014285714285715, -1.4014285714285715, 0.0, 0.0,
    -0.16957285714285714, -1.5415714285714286, 0.0, 0.0, -0.20180571428571428,
    -1.6817142857142857, 0.0, 0.0, -0.23684142857142856, -1.8218571428571428,
    0.0, 0.0, -0.27468, -1.962, 0.0, 0.0, -0.31532142857142853,
    -2.1021428571428573, 0.0, 0.0, -0.35876571428571424, -2.2422857142857144,
    0.0, 0.0, -0.40501285714285712, -2.3824285714285716, 0.0, 0.0,
    -0.4540628571428571, -2.5225714285714287, 0.0, 0.0, -0.50591571428571425,
    -2.6627142857142858, 0.0, 0.0, -0.5605714285714285, -2.8028571428571429, 0.0,
    0.0, -0.61803, -2.943, 0.0, 0.0, -0.67829142857142855, -3.0831428571428572,
    0.0, 0.0, -0.74135571428571434, -3.2232857142857143, -0.0, -0.0, -0.0, -0.0,
    -1.0, -0.0, 0.0, 0.0, 0.0, 0.0, 1.0, 0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0,
    -0.0, -0.0, -0.0014014285714285715, -0.14014285714285715, -0.0, -0.0,
    -0.0056057142857142859, -0.2802857142857143, -0.0, -0.0,
    -0.012612857142857144, -0.42042857142857148, -0.0, -0.0,
    -0.022422857142857144, -0.56057142857142861, -0.0, -0.0,
    -0.035035714285714288, -0.70071428571428573, -0.0, -0.0,
    -0.050451428571428578, -0.84085714285714286, -0.0, -0.0,
    -0.068670000000000009, -0.981, -0.0, -0.0, -0.089691428571428589,
    -1.1211428571428572, -0.0, -0.0, -0.1135157142857143, -1.2612857142857143,
    -0.0, -0.0, -0.14014285714285715, -1.4014285714285715, -0.0, -0.0,
    -0.16957285714285714, -1.5415714285714286, -0.0, -0.0, -0.20180571428571428,
    -1.6817142857142857, -0.0, -0.0, -0.23684142857142856, -1.8218571428571428,
    -0.0, -0.0, -0.27468, -1.962, -0.0, -0.0, -0.31532142857142853,
    -2.1021428571428573, -0.0, -0.0, -0.35876571428571424, -2.2422857142857144,
    -0.0, -0.0, -0.40501285714285712, -2.3824285714285716, -0.0, -0.0,
    -0.4540628571428571, -2.5225714285714287, -0.0, -0.0, -0.50591571428571425,
    -2.6627142857142858, -0.0, -0.0, -0.5605714285714285, -2.8028571428571429,
    -0.0, -0.0, -0.61803, -2.943, -0.0, -0.0, -0.67829142857142855,
    -3.0831428571428572, -0.0, -0.0, -0.74135571428571434, -3.2232857142857143,
    -0.0, -0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0014014285714285715,
    0.14014285714285715, 0.0, 0.0, 0.0056057142857142859, 0.2802857142857143,
    0.0, 0.0, 0.012612857142857144, 0.42042857142857148, 0.0, 0.0,
    0.022422857142857144, 0.56057142857142861, 0.0, 0.0, 0.035035714285714288,
    0.70071428571428573, 0.0, 0.0, 0.050451428571428578, 0.84085714285714286,
    0.0, 0.0, 0.068670000000000009, 0.981, 0.0, 0.0, 0.089691428571428589,
    1.1211428571428572, 0.0, 0.0, 0.1135157142857143, 1.2612857142857143, 0.0,
    0.0, 0.14014285714285715, 1.4014285714285715, 0.0, 0.0, 0.16957285714285714,
    1.5415714285714286, 0.0, 0.0, 0.20180571428571428, 1.6817142857142857, 0.0,
    0.0, 0.23684142857142856, 1.8218571428571428, 0.0, 0.0, 0.27468, 1.962, 0.0,
    0.0, 0.31532142857142853, 2.1021428571428573, 0.0, 0.0, 0.35876571428571424,
    2.2422857142857144, 0.0, 0.0, 0.40501285714285712, 2.3824285714285716, 0.0,
    0.0, 0.4540628571428571, 2.5225714285714287, 0.0, 0.0, 0.50591571428571425,
    2.6627142857142858, 0.0, 0.0, 0.5605714285714285, 2.8028571428571429, 0.0,
    0.0, 0.61803, 2.943, 0.0, 0.0, 0.67829142857142855, 3.0831428571428572, 0.0,
    0.0, 0.74135571428571434, 3.2232857142857143, 0.0, 0.0, -0.0, -0.0, -0.0,
    -0.0, -0.0, -1.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0,
    1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0,
    1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0,
    1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0,
    1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0,
    1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0,
    1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0,
    1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0,
    1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0,
    1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0,
    1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0,
    1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0,
    1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0,
    1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0 };

  static const real_T b_Mlim_1[212] = { 0.15, 1.0, 0.15, 1.0, 0.15, 1.0, 0.15,
    1.0, 0.15, 1.0, 0.15, 1.0, 0.15, 1.0, 0.15, 1.0, 0.15, 1.0, 0.15, 1.0, 0.15,
    1.0, 0.15, 1.0, 0.15, 1.0, 0.15, 1.0, 0.15, 1.0, 0.15, 1.0, 0.15, 1.0, 0.15,
    1.0, 0.15, 1.0, 0.15, 1.0, 0.15, 1.0, 0.15, 1.0, 0.15, 1.0, 0.15, 1.0, 0.15,
    1.0, 0.15, 1.0, 0.15, 1.0, 0.15, 1.0, 0.15, 1.0, 0.15, 1.0, 0.15, 1.0, 0.15,
    1.0, 0.15, 1.0, 0.15, 1.0, 0.15, 1.0, 0.15, 1.0, 0.15, 1.0, 0.15, 1.0, 0.15,
    1.0, 0.15, 1.0, 0.15, 1.0, 0.15, 1.0, 0.15, 1.0, 0.15, 1.0, 0.15, 1.0, 0.15,
    1.0, 0.15, 1.0, 0.15, 1.0, 0.15, 1.0, 0.15, 1.0, 0.15, 1.0, 0.15, 1.0, 0.15,
    1.0, 0.15, 1.0, 0.15, 1.0, 0.15, 1.0, 0.15, 1.0, 0.15, 1.0, 0.15, 1.0, 0.15,
    1.0, 0.15, 1.0, 0.15, 1.0, 0.15, 1.0, 0.15, 1.0, 0.15, 1.0, 0.15, 1.0, 0.15,
    1.0, 0.15, 1.0, 0.15, 1.0, 0.15, 1.0, 0.15, 1.0, 0.15, 1.0, 0.15, 1.0, 0.15,
    1.0, 0.15, 1.0, 0.15, 1.0, 0.15, 1.0, 0.15, 1.0, 0.15, 1.0, 0.15, 1.0, 0.15,
    1.0, 0.15, 1.0, 0.15, 1.0, 0.15, 1.0, 0.15, 1.0, 0.15, 1.0, 0.15, 1.0, 0.15,
    1.0, 0.15, 1.0, 0.15, 1.0, 0.15, 1.0, 0.15, 1.0, 0.15, 1.0, 0.15, 1.0, 0.15,
    1.0, 0.15, 1.0, 0.15, 1.0, 0.15, 1.0, 0.15, 1.0, 0.15, 1.0,
    0.17453292519943295, 0.17453292519943295, 0.17453292519943295,
    0.17453292519943295, 0.17453292519943295, 0.17453292519943295,
    0.17453292519943295, 0.17453292519943295, 0.17453292519943295,
    0.17453292519943295, 0.17453292519943295, 0.17453292519943295 };

  static const real_T b_Mu1[424] = { -0.0, -0.0, 0.0014014285714285715,
    0.14014285714285715, -0.0, -0.0, 0.0056057142857142859, 0.2802857142857143,
    -0.0, -0.0, 0.012612857142857144, 0.42042857142857148, -0.0, -0.0,
    0.022422857142857144, 0.56057142857142861, -0.0, -0.0, 0.035035714285714288,
    0.70071428571428573, -0.0, -0.0, 0.050451428571428578, 0.84085714285714286,
    -0.0, -0.0, 0.068670000000000009, 0.981, -0.0, -0.0, 0.089691428571428589,
    1.1211428571428572, -0.0, -0.0, 0.1135157142857143, 1.2612857142857143, -0.0,
    -0.0, 0.14014285714285715, 1.4014285714285715, -0.0, -0.0,
    0.16957285714285714, 1.5415714285714286, -0.0, -0.0, 0.20180571428571428,
    1.6817142857142857, -0.0, -0.0, 0.23684142857142856, 1.8218571428571428,
    -0.0, -0.0, 0.27468, 1.962, -0.0, -0.0, 0.31532142857142853,
    2.1021428571428573, -0.0, -0.0, 0.35876571428571424, 2.2422857142857144,
    -0.0, -0.0, 0.40501285714285712, 2.3824285714285716, -0.0, -0.0,
    0.4540628571428571, 2.5225714285714287, -0.0, -0.0, 0.50591571428571425,
    2.6627142857142858, -0.0, -0.0, 0.5605714285714285, 2.8028571428571429, -0.0,
    -0.0, 0.61803, 2.943, -0.0, -0.0, 0.67829142857142855, 3.0831428571428572,
    -0.0, -0.0, 0.74135571428571434, 3.2232857142857143, -0.0, -0.0,
    0.80722285714285724, 3.3634285714285714, -0.0, -0.0, 0.87589285714285725,
    3.5035714285714286, 0.0, 0.0, -0.0014014285714285715, -0.14014285714285715,
    0.0, 0.0, -0.0056057142857142859, -0.2802857142857143, 0.0, 0.0,
    -0.012612857142857144, -0.42042857142857148, 0.0, 0.0, -0.022422857142857144,
    -0.56057142857142861, 0.0, 0.0, -0.035035714285714288, -0.70071428571428573,
    0.0, 0.0, -0.050451428571428578, -0.84085714285714286, 0.0, 0.0,
    -0.068670000000000009, -0.981, 0.0, 0.0, -0.089691428571428589,
    -1.1211428571428572, 0.0, 0.0, -0.1135157142857143, -1.2612857142857143, 0.0,
    0.0, -0.14014285714285715, -1.4014285714285715, 0.0, 0.0,
    -0.16957285714285714, -1.5415714285714286, 0.0, 0.0, -0.20180571428571428,
    -1.6817142857142857, 0.0, 0.0, -0.23684142857142856, -1.8218571428571428,
    0.0, 0.0, -0.27468, -1.962, 0.0, 0.0, -0.31532142857142853,
    -2.1021428571428573, 0.0, 0.0, -0.35876571428571424, -2.2422857142857144,
    0.0, 0.0, -0.40501285714285712, -2.3824285714285716, 0.0, 0.0,
    -0.4540628571428571, -2.5225714285714287, 0.0, 0.0, -0.50591571428571425,
    -2.6627142857142858, 0.0, 0.0, -0.5605714285714285, -2.8028571428571429, 0.0,
    0.0, -0.61803, -2.943, 0.0, 0.0, -0.67829142857142855, -3.0831428571428572,
    0.0, 0.0, -0.74135571428571434, -3.2232857142857143, 0.0, 0.0,
    -0.80722285714285724, -3.3634285714285714, 0.0, 0.0, -0.87589285714285725,
    -3.5035714285714286, -1.0, -0.0, -1.0, -0.0, -1.0, -0.0, 1.0, 0.0, 1.0, 0.0,
    1.0, 0.0, -0.0014014285714285715, -0.14014285714285715, -0.0, -0.0,
    -0.0056057142857142859, -0.2802857142857143, -0.0, -0.0,
    -0.012612857142857144, -0.42042857142857148, -0.0, -0.0,
    -0.022422857142857144, -0.56057142857142861, -0.0, -0.0,
    -0.035035714285714288, -0.70071428571428573, -0.0, -0.0,
    -0.050451428571428578, -0.84085714285714286, -0.0, -0.0,
    -0.068670000000000009, -0.981, -0.0, -0.0, -0.089691428571428589,
    -1.1211428571428572, -0.0, -0.0, -0.1135157142857143, -1.2612857142857143,
    -0.0, -0.0, -0.14014285714285715, -1.4014285714285715, -0.0, -0.0,
    -0.16957285714285714, -1.5415714285714286, -0.0, -0.0, -0.20180571428571428,
    -1.6817142857142857, -0.0, -0.0, -0.23684142857142856, -1.8218571428571428,
    -0.0, -0.0, -0.27468, -1.962, -0.0, -0.0, -0.31532142857142853,
    -2.1021428571428573, -0.0, -0.0, -0.35876571428571424, -2.2422857142857144,
    -0.0, -0.0, -0.40501285714285712, -2.3824285714285716, -0.0, -0.0,
    -0.4540628571428571, -2.5225714285714287, -0.0, -0.0, -0.50591571428571425,
    -2.6627142857142858, -0.0, -0.0, -0.5605714285714285, -2.8028571428571429,
    -0.0, -0.0, -0.61803, -2.943, -0.0, -0.0, -0.67829142857142855,
    -3.0831428571428572, -0.0, -0.0, -0.74135571428571434, -3.2232857142857143,
    -0.0, -0.0, -0.80722285714285724, -3.3634285714285714, -0.0, -0.0,
    -0.87589285714285725, -3.5035714285714286, -0.0, -0.0, 0.0014014285714285715,
    0.14014285714285715, 0.0, 0.0, 0.0056057142857142859, 0.2802857142857143,
    0.0, 0.0, 0.012612857142857144, 0.42042857142857148, 0.0, 0.0,
    0.022422857142857144, 0.56057142857142861, 0.0, 0.0, 0.035035714285714288,
    0.70071428571428573, 0.0, 0.0, 0.050451428571428578, 0.84085714285714286,
    0.0, 0.0, 0.068670000000000009, 0.981, 0.0, 0.0, 0.089691428571428589,
    1.1211428571428572, 0.0, 0.0, 0.1135157142857143, 1.2612857142857143, 0.0,
    0.0, 0.14014285714285715, 1.4014285714285715, 0.0, 0.0, 0.16957285714285714,
    1.5415714285714286, 0.0, 0.0, 0.20180571428571428, 1.6817142857142857, 0.0,
    0.0, 0.23684142857142856, 1.8218571428571428, 0.0, 0.0, 0.27468, 1.962, 0.0,
    0.0, 0.31532142857142853, 2.1021428571428573, 0.0, 0.0, 0.35876571428571424,
    2.2422857142857144, 0.0, 0.0, 0.40501285714285712, 2.3824285714285716, 0.0,
    0.0, 0.4540628571428571, 2.5225714285714287, 0.0, 0.0, 0.50591571428571425,
    2.6627142857142858, 0.0, 0.0, 0.5605714285714285, 2.8028571428571429, 0.0,
    0.0, 0.61803, 2.943, 0.0, 0.0, 0.67829142857142855, 3.0831428571428572, 0.0,
    0.0, 0.74135571428571434, 3.2232857142857143, 0.0, 0.0, 0.80722285714285724,
    3.3634285714285714, 0.0, 0.0, 0.87589285714285725, 3.5035714285714286, 0.0,
    0.0, -0.0, -1.0, -0.0, -1.0, -0.0, -1.0, 0.0, 1.0, 0.0, 1.0, 0.0, 1.0 };

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
    /* DataTypeConversion: '<S4>/Data Type Conversion' incorporates:
     *  Constant: '<S4>/Constant'
     */
    Ball_and_Plate_MicroLabBox_student_B.DataTypeConversion =
      (Ball_and_Plate_MicroLabBox_student_P.Constant_Value_i != 0.0);

    /* S-Function (rti_commonblock): '<S17>/S-Function1' incorporates:
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
                           12U,
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
    a21 = 1.0 / Ball_and_Plate_MicroLabBox_student_P.Ts_Outer;
    Ball_and_Plate_MicroLabBox_student_B.Gain = a21 *
      Ball_and_Plate_MicroLabBox_student_B.Diff;

    /* Outport: '<Root>/FrameRate ' */
    Ball_and_Plate_MicroLabBox_student_Y.FrameRate =
      Ball_and_Plate_MicroLabBox_student_B.Gain;
  }

  /* FirstOrderHold: '<Root>/First Order Hold' */
  Ball_and_Plate_MicroLabBox_student_B.FirstOrderHold =
    Ball_and_Plate_MicroLabBox_student_DW.Ck;
  if (Ball_and_Plate_MicroLabBox_student_DW.Tk != (rtInf)) {
    a21 = Ball_and_Plate_MicroLabBox_student_M->Timing.t[0] -
      Ball_and_Plate_MicroLabBox_student_DW.Tk;
    Ball_and_Plate_MicroLabBox_student_B.FirstOrderHold +=
      Ball_and_Plate_MicroLabBox_student_DW.Mk * a21;
  }

  /* End of FirstOrderHold: '<Root>/First Order Hold' */

  /* Gain: '<S10>/Gain1' */
  Ball_and_Plate_MicroLabBox_student_B.Gain1 =
    Ball_and_Plate_MicroLabBox_student_P.Gain1_Gain *
    Ball_and_Plate_MicroLabBox_student_B.FirstOrderHold;
  if (rtmIsMajorTimeStep(Ball_and_Plate_MicroLabBox_student_M) &&
      Ball_and_Plate_MicroLabBox_student_M->Timing.TaskCounters.TID[1] == 0) {
    /* S-Function (dlowpass1): '<S10>/Dct1lowpass2' */

    /* Level2 S-Function Block: '<S10>/Dct1lowpass2' (dlowpass1) */
    {
      SimStruct *rts = Ball_and_Plate_MicroLabBox_student_M->childSfunctions[0];
      sfcnOutputs(rts,1);
    }
  }

  /* FirstOrderHold: '<Root>/First Order Hold1' */
  Ball_and_Plate_MicroLabBox_student_B.FirstOrderHold1 =
    Ball_and_Plate_MicroLabBox_student_DW.Ck_h;
  if (Ball_and_Plate_MicroLabBox_student_DW.Tk_c != (rtInf)) {
    a21 = Ball_and_Plate_MicroLabBox_student_M->Timing.t[0] -
      Ball_and_Plate_MicroLabBox_student_DW.Tk_c;
    Ball_and_Plate_MicroLabBox_student_B.FirstOrderHold1 +=
      Ball_and_Plate_MicroLabBox_student_DW.Mk_g * a21;
  }

  /* End of FirstOrderHold: '<Root>/First Order Hold1' */

  /* Gain: '<S11>/Gain1' */
  Ball_and_Plate_MicroLabBox_student_B.Gain1_m =
    Ball_and_Plate_MicroLabBox_student_P.Gain1_Gain_b *
    Ball_and_Plate_MicroLabBox_student_B.FirstOrderHold1;
  if (rtmIsMajorTimeStep(Ball_and_Plate_MicroLabBox_student_M) &&
      Ball_and_Plate_MicroLabBox_student_M->Timing.TaskCounters.TID[1] == 0) {
    /* S-Function (dlowpass1): '<S11>/Dct1lowpass2' */

    /* Level2 S-Function Block: '<S11>/Dct1lowpass2' (dlowpass1) */
    {
      SimStruct *rts = Ball_and_Plate_MicroLabBox_student_M->childSfunctions[1];
      sfcnOutputs(rts,1);
    }
  }

  /* Outputs for Atomic SubSystem: '<S21>/Position Measurement' */
  if (rtmIsMajorTimeStep(Ball_and_Plate_MicroLabBox_student_M) &&
      Ball_and_Plate_MicroLabBox_student_M->Timing.TaskCounters.TID[1] == 0) {
    /* Outputs for Triggered SubSystem: '<S29>/EMC_ENCODER_POS_SET_BL1' incorporates:
     *  TriggerPort: '<S36>/Trigger'
     */
    if (rtmIsMajorTimeStep(Ball_and_Plate_MicroLabBox_student_M)) {
      /* Constant: '<S5>/Init  ' */
      zcEvent = rt_ZCFcn(RISING_ZERO_CROSSING,
                         &Ball_and_Plate_MicroLabBox_student_PrevZCX.EMC_ENCODER_POS_SET_BL1_Trig_fh,
                         (Ball_and_Plate_MicroLabBox_student_P.Init_Value));
      if (zcEvent != NO_ZCEVENT) {
        /* S-Function (rti_commonblock): '<S36>/S-Function1' incorporates:
         *  Constant: '<S29>/Constant'
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

    /* End of Outputs for SubSystem: '<S29>/EMC_ENCODER_POS_SET_BL1' */

    /* S-Function (rti_commonblock): '<S35>/S-Function1' */
    /* This comment workarounds a code generation problem */

    /* Gain: '<S29>/Inc2Pos' */
    Ball_and_Plate_MicroLabBox_student_B.Inc2Pos_p =
      Ball_and_Plate_MicroLabBox_student_P.Inc2Pos_Gain *
      Ball_and_Plate_MicroLabBox_student_B.SFunction1_o1_p;

    /* Sum: '<S29>/AbsPosition' incorporates:
     *  Constant: '<S29>/Pos_offset'
     *  Constant: '<S29>/ZP'
     */
    Ball_and_Plate_MicroLabBox_student_B.AbsPosition_m =
      (Ball_and_Plate_MicroLabBox_student_B.Inc2Pos_p -
       Ball_and_Plate_MicroLabBox_student_P.ZP_Value) -
      Ball_and_Plate_MicroLabBox_student_P.Pos_offset_Value;

    /* Abs: '<S37>/Abs' */
    Ball_and_Plate_MicroLabBox_student_B.Abs_h = fabs
      (Ball_and_Plate_MicroLabBox_student_B.SFunction1_o2_d);

    /* Outputs for Enabled SubSystem: '<S37>/Enabled Subsystem' */
    /* Constant: '<S5>/Init  ' */
    Ball_and_Plate_EnabledSubsystem(Ball_and_Plate_MicroLabBox_student_M,
      Ball_and_Plate_MicroLabBox_student_P.Init_Value,
      &Ball_and_Plate_MicroLabBox_student_B.EnabledSubsystem,
      &Ball_and_Plate_MicroLabBox_student_DW.EnabledSubsystem,
      &Ball_and_Plate_MicroLabBox_student_P.EnabledSubsystem);

    /* End of Outputs for SubSystem: '<S37>/Enabled Subsystem' */

    /* RelationalOperator: '<S37>/Relational Operator1' incorporates:
     *  Constant: '<S37>/Const'
     */
    Ball_and_Plate_MicroLabBox_student_B.RelationalOperator1_f =
      (Ball_and_Plate_MicroLabBox_student_B.Abs_h <=
       Ball_and_Plate_MicroLabBox_student_P.Const_Value);
  }

  /* TransferFcn: '<S37>/Transfer Fcn' */
  Ball_and_Plate_MicroLabBox_student_B.TransferFcn_f = 0.0;
  Ball_and_Plate_MicroLabBox_student_B.TransferFcn_f +=
    Ball_and_Plate_MicroLabBox_student_P.TransferFcn_C *
    Ball_and_Plate_MicroLabBox_student_X.TransferFcn_CSTATE_l;

  /* RelationalOperator: '<S37>/Relational Operator' incorporates:
   *  Constant: '<S37>/Const1'
   */
  Ball_and_Plate_MicroLabBox_student_B.RelationalOperator_a =
    (Ball_and_Plate_MicroLabBox_student_P.Const1_Value >=
     Ball_and_Plate_MicroLabBox_student_B.TransferFcn_f);

  /* Logic: '<S37>/Logical Operator2' incorporates:
   *  Constant: '<S5>/Init  '
   */
  Ball_and_Plate_MicroLabBox_student_B.LogicalOperator2_a =
    (Ball_and_Plate_MicroLabBox_student_B.RelationalOperator1_f &&
     Ball_and_Plate_MicroLabBox_student_B.RelationalOperator_a &&
     (Ball_and_Plate_MicroLabBox_student_P.Init_Value != 0.0));

  /* Outputs for Enabled SubSystem: '<S37>/Enabled Subsystem1' */
  Ball_and_Plat_EnabledSubsystem1(Ball_and_Plate_MicroLabBox_student_M,
    Ball_and_Plate_MicroLabBox_student_B.LogicalOperator2_a,
    Ball_and_Plate_MicroLabBox_student_B.Inc2Pos_p,
    &Ball_and_Plate_MicroLabBox_student_B.EnabledSubsystem1,
    &Ball_and_Plate_MicroLabBox_student_DW.EnabledSubsystem1);

  /* End of Outputs for SubSystem: '<S37>/Enabled Subsystem1' */
  if (rtmIsMajorTimeStep(Ball_and_Plate_MicroLabBox_student_M) &&
      Ball_and_Plate_MicroLabBox_student_M->Timing.TaskCounters.TID[1] == 0) {
    /* Gain: '<S29>/mm2m' */
    Ball_and_Plate_MicroLabBox_student_B.mm2m_po =
      Ball_and_Plate_MicroLabBox_student_P.mm2m_Gain *
      Ball_and_Plate_MicroLabBox_student_B.AbsPosition_m;
  }

  /* End of Outputs for SubSystem: '<S21>/Position Measurement' */

  /* Outputs for Atomic SubSystem: '<S22>/Position Measurement' */
  if (rtmIsMajorTimeStep(Ball_and_Plate_MicroLabBox_student_M) &&
      Ball_and_Plate_MicroLabBox_student_M->Timing.TaskCounters.TID[1] == 0) {
    /* Outputs for Triggered SubSystem: '<S44>/EMC_ENCODER_POS_SET_BL1' incorporates:
     *  TriggerPort: '<S51>/Trigger'
     */
    if (rtmIsMajorTimeStep(Ball_and_Plate_MicroLabBox_student_M)) {
      /* Constant: '<S5>/Init  ' */
      zcEvent = rt_ZCFcn(RISING_ZERO_CROSSING,
                         &Ball_and_Plate_MicroLabBox_student_PrevZCX.EMC_ENCODER_POS_SET_BL1_Trig__f,
                         (Ball_and_Plate_MicroLabBox_student_P.Init_Value));
      if (zcEvent != NO_ZCEVENT) {
        /* S-Function (rti_commonblock): '<S51>/S-Function1' incorporates:
         *  Constant: '<S44>/Constant'
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

    /* End of Outputs for SubSystem: '<S44>/EMC_ENCODER_POS_SET_BL1' */

    /* S-Function (rti_commonblock): '<S50>/S-Function1' */
    /* This comment workarounds a code generation problem */

    /* Gain: '<S44>/Inc2Pos' */
    Ball_and_Plate_MicroLabBox_student_B.Inc2Pos_e =
      Ball_and_Plate_MicroLabBox_student_P.Inc2Pos_Gain_k *
      Ball_and_Plate_MicroLabBox_student_B.SFunction1_o1_i;

    /* Sum: '<S44>/AbsPosition' incorporates:
     *  Constant: '<S44>/Pos_offset'
     *  Constant: '<S44>/ZP'
     */
    Ball_and_Plate_MicroLabBox_student_B.AbsPosition_i =
      (Ball_and_Plate_MicroLabBox_student_B.Inc2Pos_e -
       Ball_and_Plate_MicroLabBox_student_P.ZP_Value_m) -
      Ball_and_Plate_MicroLabBox_student_P.Pos_offset_Value_k;

    /* Abs: '<S52>/Abs' */
    Ball_and_Plate_MicroLabBox_student_B.Abs_j = fabs
      (Ball_and_Plate_MicroLabBox_student_B.SFunction1_o2_p);

    /* Outputs for Enabled SubSystem: '<S52>/Enabled Subsystem' */
    /* Constant: '<S5>/Init  ' */
    Ball_and_Plate_EnabledSubsystem(Ball_and_Plate_MicroLabBox_student_M,
      Ball_and_Plate_MicroLabBox_student_P.Init_Value,
      &Ball_and_Plate_MicroLabBox_student_B.EnabledSubsystem_a,
      &Ball_and_Plate_MicroLabBox_student_DW.EnabledSubsystem_a,
      &Ball_and_Plate_MicroLabBox_student_P.EnabledSubsystem_a);

    /* End of Outputs for SubSystem: '<S52>/Enabled Subsystem' */

    /* RelationalOperator: '<S52>/Relational Operator1' incorporates:
     *  Constant: '<S52>/Const'
     */
    Ball_and_Plate_MicroLabBox_student_B.RelationalOperator1_c =
      (Ball_and_Plate_MicroLabBox_student_B.Abs_j <=
       Ball_and_Plate_MicroLabBox_student_P.Const_Value_g);
  }

  /* TransferFcn: '<S52>/Transfer Fcn' */
  Ball_and_Plate_MicroLabBox_student_B.TransferFcn_o = 0.0;
  Ball_and_Plate_MicroLabBox_student_B.TransferFcn_o +=
    Ball_and_Plate_MicroLabBox_student_P.TransferFcn_C_e *
    Ball_and_Plate_MicroLabBox_student_X.TransferFcn_CSTATE_h;

  /* RelationalOperator: '<S52>/Relational Operator' incorporates:
   *  Constant: '<S52>/Const1'
   */
  Ball_and_Plate_MicroLabBox_student_B.RelationalOperator_n =
    (Ball_and_Plate_MicroLabBox_student_P.Const1_Value_a >=
     Ball_and_Plate_MicroLabBox_student_B.TransferFcn_o);

  /* Logic: '<S52>/Logical Operator2' incorporates:
   *  Constant: '<S5>/Init  '
   */
  Ball_and_Plate_MicroLabBox_student_B.LogicalOperator2_l =
    (Ball_and_Plate_MicroLabBox_student_B.RelationalOperator1_c &&
     Ball_and_Plate_MicroLabBox_student_B.RelationalOperator_n &&
     (Ball_and_Plate_MicroLabBox_student_P.Init_Value != 0.0));

  /* Outputs for Enabled SubSystem: '<S52>/Enabled Subsystem1' */
  Ball_and_Plat_EnabledSubsystem1(Ball_and_Plate_MicroLabBox_student_M,
    Ball_and_Plate_MicroLabBox_student_B.LogicalOperator2_l,
    Ball_and_Plate_MicroLabBox_student_B.Inc2Pos_e,
    &Ball_and_Plate_MicroLabBox_student_B.EnabledSubsystem1_f,
    &Ball_and_Plate_MicroLabBox_student_DW.EnabledSubsystem1_f);

  /* End of Outputs for SubSystem: '<S52>/Enabled Subsystem1' */
  if (rtmIsMajorTimeStep(Ball_and_Plate_MicroLabBox_student_M) &&
      Ball_and_Plate_MicroLabBox_student_M->Timing.TaskCounters.TID[1] == 0) {
    /* Gain: '<S44>/mm2m' */
    Ball_and_Plate_MicroLabBox_student_B.mm2m_p =
      Ball_and_Plate_MicroLabBox_student_P.mm2m_Gain_f *
      Ball_and_Plate_MicroLabBox_student_B.AbsPosition_i;
  }

  /* End of Outputs for SubSystem: '<S22>/Position Measurement' */

  /* Outputs for Atomic SubSystem: '<S23>/Position Measurement' */
  if (rtmIsMajorTimeStep(Ball_and_Plate_MicroLabBox_student_M) &&
      Ball_and_Plate_MicroLabBox_student_M->Timing.TaskCounters.TID[1] == 0) {
    /* Outputs for Triggered SubSystem: '<S60>/EMC_ENCODER_POS_SET_BL1' incorporates:
     *  TriggerPort: '<S68>/Trigger'
     */
    if (rtmIsMajorTimeStep(Ball_and_Plate_MicroLabBox_student_M)) {
      /* Constant: '<S5>/Init  ' */
      zcEvent = rt_ZCFcn(RISING_ZERO_CROSSING,
                         &Ball_and_Plate_MicroLabBox_student_PrevZCX.EMC_ENCODER_POS_SET_BL1_Trig_ZC,
                         (Ball_and_Plate_MicroLabBox_student_P.Init_Value));
      if (zcEvent != NO_ZCEVENT) {
        /* S-Function (rti_commonblock): '<S68>/S-Function1' incorporates:
         *  Constant: '<S60>/Constant'
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

    /* End of Outputs for SubSystem: '<S60>/EMC_ENCODER_POS_SET_BL1' */

    /* S-Function (rti_commonblock): '<S67>/S-Function1' */
    /* This comment workarounds a code generation problem */

    /* Gain: '<S60>/Inc2Pos' */
    Ball_and_Plate_MicroLabBox_student_B.Inc2Pos =
      Ball_and_Plate_MicroLabBox_student_P.Inc2Pos_Gain_h *
      Ball_and_Plate_MicroLabBox_student_B.SFunction1_o1;

    /* Sum: '<S60>/AbsPosition' incorporates:
     *  Constant: '<S60>/Pos_offset'
     *  Constant: '<S60>/ZP'
     */
    Ball_and_Plate_MicroLabBox_student_B.AbsPosition =
      (Ball_and_Plate_MicroLabBox_student_B.Inc2Pos -
       Ball_and_Plate_MicroLabBox_student_P.ZP_Value_p) -
      Ball_and_Plate_MicroLabBox_student_P.Pos_offset_Value_f;

    /* Abs: '<S69>/Abs' */
    Ball_and_Plate_MicroLabBox_student_B.Abs = fabs
      (Ball_and_Plate_MicroLabBox_student_B.SFunction1_o2);

    /* Outputs for Enabled SubSystem: '<S69>/Enabled Subsystem' */
    /* Constant: '<S5>/Init  ' */
    Ball_and_Plate_EnabledSubsystem(Ball_and_Plate_MicroLabBox_student_M,
      Ball_and_Plate_MicroLabBox_student_P.Init_Value,
      &Ball_and_Plate_MicroLabBox_student_B.EnabledSubsystem_n,
      &Ball_and_Plate_MicroLabBox_student_DW.EnabledSubsystem_n,
      &Ball_and_Plate_MicroLabBox_student_P.EnabledSubsystem_n);

    /* End of Outputs for SubSystem: '<S69>/Enabled Subsystem' */

    /* RelationalOperator: '<S69>/Relational Operator1' incorporates:
     *  Constant: '<S69>/Const'
     */
    Ball_and_Plate_MicroLabBox_student_B.RelationalOperator1 =
      (Ball_and_Plate_MicroLabBox_student_B.Abs <=
       Ball_and_Plate_MicroLabBox_student_P.Const_Value_m);
  }

  /* TransferFcn: '<S69>/Transfer Fcn' */
  Ball_and_Plate_MicroLabBox_student_B.TransferFcn = 0.0;
  Ball_and_Plate_MicroLabBox_student_B.TransferFcn +=
    Ball_and_Plate_MicroLabBox_student_P.TransferFcn_C_c *
    Ball_and_Plate_MicroLabBox_student_X.TransferFcn_CSTATE;

  /* RelationalOperator: '<S69>/Relational Operator' incorporates:
   *  Constant: '<S69>/Const1'
   */
  Ball_and_Plate_MicroLabBox_student_B.RelationalOperator =
    (Ball_and_Plate_MicroLabBox_student_P.Const1_Value_p >=
     Ball_and_Plate_MicroLabBox_student_B.TransferFcn);

  /* Logic: '<S69>/Logical Operator2' incorporates:
   *  Constant: '<S5>/Init  '
   */
  Ball_and_Plate_MicroLabBox_student_B.LogicalOperator2 =
    (Ball_and_Plate_MicroLabBox_student_B.RelationalOperator1 &&
     Ball_and_Plate_MicroLabBox_student_B.RelationalOperator &&
     (Ball_and_Plate_MicroLabBox_student_P.Init_Value != 0.0));

  /* Outputs for Enabled SubSystem: '<S69>/Enabled Subsystem1' */
  Ball_and_Plat_EnabledSubsystem1(Ball_and_Plate_MicroLabBox_student_M,
    Ball_and_Plate_MicroLabBox_student_B.LogicalOperator2,
    Ball_and_Plate_MicroLabBox_student_B.Inc2Pos,
    &Ball_and_Plate_MicroLabBox_student_B.EnabledSubsystem1_b,
    &Ball_and_Plate_MicroLabBox_student_DW.EnabledSubsystem1_b);

  /* End of Outputs for SubSystem: '<S69>/Enabled Subsystem1' */
  if (rtmIsMajorTimeStep(Ball_and_Plate_MicroLabBox_student_M) &&
      Ball_and_Plate_MicroLabBox_student_M->Timing.TaskCounters.TID[1] == 0) {
    /* Gain: '<S60>/mm2m' */
    Ball_and_Plate_MicroLabBox_student_B.mm2m =
      Ball_and_Plate_MicroLabBox_student_P.mm2m_Gain_o *
      Ball_and_Plate_MicroLabBox_student_B.AbsPosition;
  }

  /* End of Outputs for SubSystem: '<S23>/Position Measurement' */
  if (rtmIsMajorTimeStep(Ball_and_Plate_MicroLabBox_student_M) &&
      Ball_and_Plate_MicroLabBox_student_M->Timing.TaskCounters.TID[1] == 0) {
    /* MATLAB Function: '<S5>/PosToAngle ' */
    /* MATLAB Function 'Innerloop_Actuator/PosToAngle ': '<S24>:1' */
    /* '<S24>:1:6' */
    /* '<S24>:1:7' */
    /* '<S24>:1:13' */
    P1_global[2] = Ball_and_Plate_MicroLabBox_student_B.mm2m_po + 0.322;

    /* '<S24>:1:14' */
    /* '<S24>:1:15' */
    P3_global_idx_2 = Ball_and_Plate_MicroLabBox_student_B.mm2m + 0.322;

    /* '<S24>:1:19' */
    a21 = (((Ball_and_Plate_MicroLabBox_student_B.mm2m_p + 0.322) + P1_global[2])
           + P3_global_idx_2) / 3.0;

    /* '<S24>:1:23' */
    /* '<S24>:1:25' */
    P1_global[0] = 0.16999999999999998;
    P1_global[1] = 0.0;
    uX_idx_0 = P1_global[2];
    uX_idx_0 -= a21;
    P3_global_idx_2 -= a21;
    P1_global[2] = uX_idx_0;

    /* '<S24>:1:28' */
    a21 = Ball_and_Plate_MicroLabBox_norm(P1_global);
    uX_idx_0 = 0.16999999999999998 / a21;
    uX_idx_1 = 0.0 / a21;
    uX_idx_2 = P1_global[2] / a21;

    /* '<S24>:1:29' */
    P1_global[0] = uX_idx_1 * P3_global_idx_2 - uX_idx_2 * -0.14722431864335458;
    P1_global[1] = uX_idx_2 * -0.084999999999999992 - uX_idx_0 * P3_global_idx_2;
    P1_global[2] = uX_idx_0 * -0.14722431864335458 - uX_idx_1 *
      -0.084999999999999992;

    /* '<S24>:1:30' */
    a21 = Ball_and_Plate_MicroLabBox_norm(P1_global);
    P3_global_idx_2 = P1_global[0] / a21;
    a21 = P1_global[1] / a21;

    /* '<S24>:1:31' */
    /* '<S24>:1:34' */
    Ball_and_Plate_MicroLabBox_student_B.beta = -uX_idx_2;
    Ball_and_Plate_MicroLabBox_student_B.beta = asin
      (Ball_and_Plate_MicroLabBox_student_B.beta);

    /* '<S24>:1:35' */
    Ball_and_Plate_MicroLabBox_student_B.alpha = (P3_global_idx_2 * uX_idx_1 -
      a21 * uX_idx_0) / sqrt(1.0 - uX_idx_2 * uX_idx_2);
    Ball_and_Plate_MicroLabBox_student_B.alpha = asin
      (Ball_and_Plate_MicroLabBox_student_B.alpha);

    /* '<S24>:1:36' */
    Ball_and_Plate_MicroLabBox_student_B.psi = uX_idx_1 / sqrt(1.0 - uX_idx_2 *
      uX_idx_2);
    Ball_and_Plate_MicroLabBox_student_B.psi = asin
      (Ball_and_Plate_MicroLabBox_student_B.psi);

    /* MATLAB Function: '<S4>/MATLAB Function' */
    /* MATLAB Function 'Ethernet communication/MATLAB Function': '<S19>:1' */
    /* '<S19>:1:2' */
    r1 = (Ball_and_Plate_MicroLabBox_student_B.SFunction1_o1_c[1] << 8) +
      Ball_and_Plate_MicroLabBox_student_B.SFunction1_o1_c[0];

    /* '<S19>:1:3' */
    r2 = (Ball_and_Plate_MicroLabBox_student_B.SFunction1_o1_c[5] << 8) +
      Ball_and_Plate_MicroLabBox_student_B.SFunction1_o1_c[4];

    /* '<S19>:1:4' */
    z = (Ball_and_Plate_MicroLabBox_student_B.SFunction1_o1_c[9] << 8) +
      Ball_and_Plate_MicroLabBox_student_B.SFunction1_o1_c[8];

    /* '<S19>:1:5' */
    Ball_and_Plate_MicroLabBox_student_B.flag =
      Ball_and_Plate_MicroLabBox_student_B.SFunction1_o1_c[10];
    if (r1 > 32767) {
      /* '<S19>:1:7' */
      /* '<S19>:1:8' */
      r1 -= 65536;
    }

    if (r2 > 32767) {
      /* '<S19>:1:10' */
      /* '<S19>:1:11' */
      r2 -= 65536;
    }

    if (z > 32767) {
      /* '<S19>:1:13' */
      /* '<S19>:1:14' */
      z -= 65536;
    }

    Ball_and_Plate_MicroLabBox_student_B.x = r1;
    Ball_and_Plate_MicroLabBox_student_B.y = r2;
    Ball_and_Plate_MicroLabBox_student_B.z = z;

    /* End of MATLAB Function: '<S4>/MATLAB Function' */

    /* SignalConversion generated from: '<S6>/ SFunction ' incorporates:
     *  MATLAB Function: '<Root>/MATLAB Function'
     */
    Ball_and_Plate_MicroLabBox_student_B.TmpSignalConversionAtSFunctionI[0] =
      Ball_and_Plate_MicroLabBox_student_B.Dct1lowpass2;
    Ball_and_Plate_MicroLabBox_student_B.TmpSignalConversionAtSFunctionI[1] =
      Ball_and_Plate_MicroLabBox_student_B.Dct1lowpass2_b;

    /* SignalConversion generated from: '<S6>/ SFunction ' incorporates:
     *  MATLAB Function: '<Root>/MATLAB Function'
     */
    Ball_and_Plate_MicroLabBox_student_B.TmpSignalConversionAtSFunctio_e[0] =
      Ball_and_Plate_MicroLabBox_student_B.alpha;
    Ball_and_Plate_MicroLabBox_student_B.TmpSignalConversionAtSFunctio_e[1] =
      Ball_and_Plate_MicroLabBox_student_B.beta;

    /* MATLAB Function: '<Root>/MATLAB Function' incorporates:
     *  Constant: '<Root>/Constant'
     *  Constant: '<Root>/Constant1'
     *  Constant: '<Root>/Constant2'
     */
    /* MATLAB Function 'MATLAB Function': '<S6>:1' */
    /* '<S6>:1:5' */
    /* '<S6>:1:7' */
    /* '<S6>:1:34' */
    /* '<S6>:1:3' */
    /* '<S6>:1:4' */
    Ad[0] = 1.0;
    Ad[4] = Ball_and_Plate_MicroLabBox_student_P.Ts_Inner;
    Ad[8] = 0.0;
    Ad[12] = 0.0;
    Ad[2] = 0.0;
    Ad[6] = 0.0;
    Ad[10] = 1.0;
    Ad[14] = Ball_and_Plate_MicroLabBox_student_P.Ts_Inner;
    Ad[1] = 0.0;
    Ad[3] = 0.0;
    Ad[5] = 1.0;
    Ad[7] = 0.0;
    Ad[9] = 0.0;
    Ad[11] = 0.0;
    Ad[13] = 0.0;
    Ad[15] = 1.0;

    /* '<S6>:1:9' */
    /* '<S6>:1:28' */
    y[0] = Ball_and_Plate_MicroLabBox_student_P.Ts_Inner *
      Ball_and_Plate_MicroLabBox_student_P.Ts_Inner * 3.503571428571429;
    y[4] = 0.0;
    y[1] = 7.007142857142858 * Ball_and_Plate_MicroLabBox_student_P.Ts_Inner;
    y[5] = 0.0;
    y[2] = 0.0;
    y[6] = Ball_and_Plate_MicroLabBox_student_P.Ts_Inner *
      Ball_and_Plate_MicroLabBox_student_P.Ts_Inner * 3.503571428571429;
    y[3] = 0.0;
    y[7] = 7.007142857142858 * Ball_and_Plate_MicroLabBox_student_P.Ts_Inner;

    /* '<S6>:1:29' */
    for (r1 = 0; r1 < 4; r1++) {
      P3_global_idx_2 = Ad[r1] * Ball_and_Plate_MicroLabBox_student_DW.x_hat[0];
      P3_global_idx_2 += Ad[r1 + 4] *
        Ball_and_Plate_MicroLabBox_student_DW.x_hat[1];
      P3_global_idx_2 += Ad[r1 + 8] *
        Ball_and_Plate_MicroLabBox_student_DW.x_hat[2];
      P3_global_idx_2 += Ad[r1 + 12] *
        Ball_and_Plate_MicroLabBox_student_DW.x_hat[3];
      a21 = y[r1] *
        Ball_and_Plate_MicroLabBox_student_B.TmpSignalConversionAtSFunctio_e[0];
      a21 += y[r1 + 4] *
        Ball_and_Plate_MicroLabBox_student_B.TmpSignalConversionAtSFunctio_e[1];
      x_pred[r1] = P3_global_idx_2 + a21;
      for (r2 = 0; r2 < 4; r2++) {
        Ad_0[r1 + (r2 << 2)] = 0.0;
        P3_global_idx_2 = Ad_0[(r2 << 2) + r1];
        P3_global_idx_2 += Ball_and_Plate_MicroLabBox_student_DW.P[r2 << 2] *
          Ad[r1];
        Ad_0[r1 + (r2 << 2)] = P3_global_idx_2;
        P3_global_idx_2 = Ad_0[(r2 << 2) + r1];
        P3_global_idx_2 += Ball_and_Plate_MicroLabBox_student_DW.P[(r2 << 2) + 1]
          * Ad[r1 + 4];
        Ad_0[r1 + (r2 << 2)] = P3_global_idx_2;
        P3_global_idx_2 = Ad_0[(r2 << 2) + r1];
        P3_global_idx_2 += Ball_and_Plate_MicroLabBox_student_DW.P[(r2 << 2) + 2]
          * Ad[r1 + 8];
        Ad_0[r1 + (r2 << 2)] = P3_global_idx_2;
        P3_global_idx_2 = Ad_0[(r2 << 2) + r1];
        P3_global_idx_2 += Ball_and_Plate_MicroLabBox_student_DW.P[(r2 << 2) + 3]
          * Ad[r1 + 12];
        Ad_0[r1 + (r2 << 2)] = P3_global_idx_2;
      }

      for (r2 = 0; r2 < 4; r2++) {
        a21 = Ad_0[r1] * Ad[r2];
        a21 += Ad_0[r1 + 4] * Ad[r2 + 4];
        a21 += Ad_0[r1 + 8] * Ad[r2 + 8];
        a21 += Ad_0[r1 + 12] * Ad[r2 + 12];
        P_pred[r1 + (r2 << 2)] = Ball_and_Plate_MicroLabBox_student_P.Q_kf[(r2 <<
          2) + r1] + a21;
      }
    }

    if (Ball_and_Plate_MicroLabBox_student_B.flag > 0.5) {
      /* '<S6>:1:32' */
      /* '<S6>:1:34' */
      /* '<S6>:1:35' */
      for (r1 = 0; r1 < 2; r1++) {
        for (r2 = 0; r2 < 4; r2++) {
          K[r1 + (r2 << 1)] = 0.0;
          a21 = K[(r2 << 1) + r1];
          a21 += P_pred[r2 << 2] * (real_T)a[r1];
          K[r1 + (r2 << 1)] = a21;
          a21 = K[(r2 << 1) + r1];
          a21 += P_pred[(r2 << 2) + 1] * 0.0;
          K[r1 + (r2 << 1)] = a21;
          a21 = K[(r2 << 1) + r1];
          a21 += P_pred[(r2 << 2) + 2] * (real_T)a[r1 + 4];
          K[r1 + (r2 << 1)] = a21;
          a21 = K[(r2 << 1) + r1];
          a21 += P_pred[(r2 << 2) + 3] * 0.0;
          K[r1 + (r2 << 1)] = a21;
        }

        for (r2 = 0; r2 < 2; r2++) {
          a21 = (real_T)b_b[r2 << 2] * K[r1];
          a21 += K[r1 + 2] * 0.0;
          a21 += (real_T)b_b[(r2 << 2) + 2] * K[r1 + 4];
          a21 += (real_T)b_b[(r2 << 2) + 3] * K[r1 + 6];
          S[r1 + (r2 << 1)] = Ball_and_Plate_MicroLabBox_student_P.R_kf[(r2 << 1)
            + r1] + a21;
        }

        for (r2 = 0; r2 < 4; r2++) {
          y[r2 + (r1 << 2)] = 0.0;
          a21 = y[(r1 << 2) + r2];
          a21 += (real_T)b_b[r1 << 2] * P_pred[r2];
          y[r2 + (r1 << 2)] = a21;
          a21 = y[(r1 << 2) + r2];
          a21 += P_pred[r2 + 4] * 0.0;
          y[r2 + (r1 << 2)] = a21;
          a21 = y[(r1 << 2) + r2];
          a21 += (real_T)b_b[(r1 << 2) + 2] * P_pred[r2 + 8];
          y[r2 + (r1 << 2)] = a21;
          a21 = y[(r1 << 2) + r2];
          a21 += (real_T)b_b[(r1 << 2) + 3] * P_pred[r2 + 12];
          y[r2 + (r1 << 2)] = a21;
        }
      }

      if (fabs(S[1]) > fabs(S[0])) {
        r1 = 1;
        r2 = 0;
      } else {
        r1 = 0;
        r2 = 1;
      }

      a21 = S[r2] / S[r1];
      P3_global_idx_2 = S[r2 + 2] - S[r1 + 2] * a21;
      K[r1 << 2] = y[0] / S[r1];
      K[r2 << 2] = (y[4] - K[r1 << 2] * S[r1 + 2]) / P3_global_idx_2;
      K[r1 << 2] -= K[r2 << 2] * a21;
      K[(r1 << 2) + 1] = y[1] / S[r1];
      K[(r2 << 2) + 1] = (y[5] - K[(r1 << 2) + 1] * S[r1 + 2]) / P3_global_idx_2;
      K[(r1 << 2) + 1] -= K[(r2 << 2) + 1] * a21;
      K[(r1 << 2) + 2] = y[2] / S[r1];
      K[(r2 << 2) + 2] = (y[6] - K[(r1 << 2) + 2] * S[r1 + 2]) / P3_global_idx_2;
      K[(r1 << 2) + 2] -= K[(r2 << 2) + 2] * a21;
      K[(r1 << 2) + 3] = y[3] / S[r1];
      K[(r2 << 2) + 3] = (y[7] - K[(r1 << 2) + 3] * S[r1 + 2]) / P3_global_idx_2;
      K[(r1 << 2) + 3] -= K[(r2 << 2) + 3] * a21;

      /* '<S6>:1:37' */
      for (r1 = 0; r1 < 2; r1++) {
        a21 = (real_T)a[r1] * x_pred[0];
        a21 += 0.0 * x_pred[1];
        a21 += (real_T)a[r1 + 4] * x_pred[2];
        a21 += 0.0 * x_pred[3];
        tmp_7[r1] =
          Ball_and_Plate_MicroLabBox_student_B.TmpSignalConversionAtSFunctionI[r1]
          - a21;
      }

      for (r1 = 0; r1 < 4; r1++) {
        a21 = K[r1] * tmp_7[0];
        a21 += K[r1 + 4] * tmp_7[1];
        Ball_and_Plate_MicroLabBox_student_DW.x_hat[r1] = x_pred[r1] + a21;
      }

      /* '<S6>:1:38' */
      memset(&Ad[0], 0, sizeof(real_T) << 4U);
      Ad[0] = 1.0;
      Ad[5] = 1.0;
      Ad[10] = 1.0;
      Ad[15] = 1.0;
      for (r1 = 0; r1 < 4; r1++) {
        for (r2 = 0; r2 < 4; r2++) {
          a21 = (real_T)a[r2 << 1] * K[r1];
          a21 += (real_T)a[(r2 << 1) + 1] * K[r1 + 4];
          Ad_0[r1 + (r2 << 2)] = Ad[(r2 << 2) + r1] - a21;
        }

        for (r2 = 0; r2 < 4; r2++) {
          Ball_and_Plate_MicroLabBox_student_DW.P[r1 + (r2 << 2)] = 0.0;
          Ball_and_Plate_MicroLabBox_student_DW.P[r1 + (r2 << 2)] += P_pred[r2 <<
            2] * Ad_0[r1];
          Ball_and_Plate_MicroLabBox_student_DW.P[r1 + (r2 << 2)] += P_pred[(r2 <<
            2) + 1] * Ad_0[r1 + 4];
          Ball_and_Plate_MicroLabBox_student_DW.P[r1 + (r2 << 2)] += P_pred[(r2 <<
            2) + 2] * Ad_0[r1 + 8];
          Ball_and_Plate_MicroLabBox_student_DW.P[r1 + (r2 << 2)] += P_pred[(r2 <<
            2) + 3] * Ad_0[r1 + 12];
        }
      }
    } else {
      /* '<S6>:1:41' */
      Ball_and_Plate_MicroLabBox_student_DW.x_hat[0] = x_pred[0];
      Ball_and_Plate_MicroLabBox_student_DW.x_hat[1] = x_pred[1];
      Ball_and_Plate_MicroLabBox_student_DW.x_hat[2] = x_pred[2];
      Ball_and_Plate_MicroLabBox_student_DW.x_hat[3] = x_pred[3];

      /* '<S6>:1:42' */
      memcpy(&Ball_and_Plate_MicroLabBox_student_DW.P[0], &P_pred[0], sizeof
             (real_T) << 4U);
    }

    /* '<S6>:1:46' */
    Ball_and_Plate_MicroLabBox_student_B.x_est[0] =
      Ball_and_Plate_MicroLabBox_student_DW.x_hat[0];
    Ball_and_Plate_MicroLabBox_student_B.x_est[1] =
      Ball_and_Plate_MicroLabBox_student_DW.x_hat[1];
    Ball_and_Plate_MicroLabBox_student_B.x_est[2] =
      Ball_and_Plate_MicroLabBox_student_DW.x_hat[2];
    Ball_and_Plate_MicroLabBox_student_B.x_est[3] =
      Ball_and_Plate_MicroLabBox_student_DW.x_hat[3];

    /* Saturate: '<Root>/Saturation2' */
    a21 = -Ball_and_Plate_MicroLabBox_student_P.ball_pos_sat;
    P3_global_idx_2 = Ball_and_Plate_MicroLabBox_student_B.x_est[0];
    uX_idx_0 = Ball_and_Plate_MicroLabBox_student_P.ball_pos_sat;
    if (P3_global_idx_2 > uX_idx_0) {
      /* Outport: '<Root>/x_obs' */
      Ball_and_Plate_MicroLabBox_student_Y.x_obs = uX_idx_0;
    } else if (P3_global_idx_2 < a21) {
      /* Outport: '<Root>/x_obs' */
      Ball_and_Plate_MicroLabBox_student_Y.x_obs = a21;
    } else {
      /* Outport: '<Root>/x_obs' */
      Ball_and_Plate_MicroLabBox_student_Y.x_obs = P3_global_idx_2;
    }

    /* End of Saturate: '<Root>/Saturation2' */

    /* Gain: '<S14>/Gain1' */
    Ball_and_Plate_MicroLabBox_student_B.Gain1_b =
      Ball_and_Plate_MicroLabBox_student_P.Gain1_Gain_e *
      Ball_and_Plate_MicroLabBox_student_B.x_est[1];

    /* S-Function (dlowpass1): '<S14>/Dct1lowpass2' */

    /* Level2 S-Function Block: '<S14>/Dct1lowpass2' (dlowpass1) */
    {
      SimStruct *rts = Ball_and_Plate_MicroLabBox_student_M->childSfunctions[2];
      sfcnOutputs(rts,1);
    }

    /* Saturate: '<Root>/Saturation1' */
    a21 = -Ball_and_Plate_MicroLabBox_student_P.ball_vel_sat;
    P3_global_idx_2 = Ball_and_Plate_MicroLabBox_student_B.Dct1lowpass2_l;
    uX_idx_0 = Ball_and_Plate_MicroLabBox_student_P.ball_vel_sat;
    if (P3_global_idx_2 > uX_idx_0) {
      /* Outport: '<Root>/x_dot_obs' */
      Ball_and_Plate_MicroLabBox_student_Y.x_dot_obs = uX_idx_0;
    } else if (P3_global_idx_2 < a21) {
      /* Outport: '<Root>/x_dot_obs' */
      Ball_and_Plate_MicroLabBox_student_Y.x_dot_obs = a21;
    } else {
      /* Outport: '<Root>/x_dot_obs' */
      Ball_and_Plate_MicroLabBox_student_Y.x_dot_obs = P3_global_idx_2;
    }

    /* End of Saturate: '<Root>/Saturation1' */

    /* Saturate: '<Root>/Saturation3' */
    a21 = -Ball_and_Plate_MicroLabBox_student_P.ball_pos_sat;
    P3_global_idx_2 = Ball_and_Plate_MicroLabBox_student_B.x_est[2];
    uX_idx_0 = Ball_and_Plate_MicroLabBox_student_P.ball_pos_sat;
    if (P3_global_idx_2 > uX_idx_0) {
      /* Outport: '<Root>/y_obs' */
      Ball_and_Plate_MicroLabBox_student_Y.y_obs = uX_idx_0;
    } else if (P3_global_idx_2 < a21) {
      /* Outport: '<Root>/y_obs' */
      Ball_and_Plate_MicroLabBox_student_Y.y_obs = a21;
    } else {
      /* Outport: '<Root>/y_obs' */
      Ball_and_Plate_MicroLabBox_student_Y.y_obs = P3_global_idx_2;
    }

    /* End of Saturate: '<Root>/Saturation3' */

    /* Gain: '<S15>/Gain1' */
    Ball_and_Plate_MicroLabBox_student_B.Gain1_bh =
      Ball_and_Plate_MicroLabBox_student_P.Gain1_Gain_b0 *
      Ball_and_Plate_MicroLabBox_student_B.x_est[3];

    /* S-Function (dlowpass1): '<S15>/Dct1lowpass2' */

    /* Level2 S-Function Block: '<S15>/Dct1lowpass2' (dlowpass1) */
    {
      SimStruct *rts = Ball_and_Plate_MicroLabBox_student_M->childSfunctions[3];
      sfcnOutputs(rts,1);
    }

    /* Saturate: '<Root>/Saturation' */
    a21 = -Ball_and_Plate_MicroLabBox_student_P.ball_vel_sat;
    P3_global_idx_2 = Ball_and_Plate_MicroLabBox_student_B.Dct1lowpass2_o;
    uX_idx_0 = Ball_and_Plate_MicroLabBox_student_P.ball_vel_sat;
    if (P3_global_idx_2 > uX_idx_0) {
      /* Outport: '<Root>/y_dot_obs' */
      Ball_and_Plate_MicroLabBox_student_Y.y_dot_obs = uX_idx_0;
    } else if (P3_global_idx_2 < a21) {
      /* Outport: '<Root>/y_dot_obs' */
      Ball_and_Plate_MicroLabBox_student_Y.y_dot_obs = a21;
    } else {
      /* Outport: '<Root>/y_dot_obs' */
      Ball_and_Plate_MicroLabBox_student_Y.y_dot_obs = P3_global_idx_2;
    }

    /* End of Saturate: '<Root>/Saturation' */

    /* Switch: '<Root>/Switch' incorporates:
     *  Constant: '<Root>/Constant3'
     *  Constant: '<Root>/Outer_loop_enable'
     *  Outport: '<Root>/x_dot_obs'
     *  Outport: '<Root>/x_obs'
     *  Outport: '<Root>/y_dot_obs'
     *  Outport: '<Root>/y_obs'
     */
    if (Ball_and_Plate_MicroLabBox_student_P.Outer_loop_enable_Value >
        Ball_and_Plate_MicroLabBox_student_P.Switch_Threshold) {
      Ball_and_Plate_MicroLabBox_student_B.Switch[0] =
        Ball_and_Plate_MicroLabBox_student_Y.x_obs;
      Ball_and_Plate_MicroLabBox_student_B.Switch[1] =
        Ball_and_Plate_MicroLabBox_student_Y.x_dot_obs;
      Ball_and_Plate_MicroLabBox_student_B.Switch[2] =
        Ball_and_Plate_MicroLabBox_student_Y.y_obs;
      Ball_and_Plate_MicroLabBox_student_B.Switch[3] =
        Ball_and_Plate_MicroLabBox_student_Y.y_dot_obs;
    } else {
      Ball_and_Plate_MicroLabBox_student_B.Switch[0] =
        Ball_and_Plate_MicroLabBox_student_P.Constant3_Value[0];
      Ball_and_Plate_MicroLabBox_student_B.Switch[1] =
        Ball_and_Plate_MicroLabBox_student_P.Constant3_Value[1];
      Ball_and_Plate_MicroLabBox_student_B.Switch[2] =
        Ball_and_Plate_MicroLabBox_student_P.Constant3_Value[2];
      Ball_and_Plate_MicroLabBox_student_B.Switch[3] =
        Ball_and_Plate_MicroLabBox_student_P.Constant3_Value[3];
    }

    /* End of Switch: '<Root>/Switch' */

    /* Sum: '<Root>/Add' incorporates:
     *  Constant: '<Root>/Constant9'
     */
    Ball_and_Plate_MicroLabBox_student_B.Add[0] =
      Ball_and_Plate_MicroLabBox_student_B.Switch[0] -
      Ball_and_Plate_MicroLabBox_student_P.Constant9_Value[0];
    Ball_and_Plate_MicroLabBox_student_B.Add[1] =
      Ball_and_Plate_MicroLabBox_student_B.Switch[1] -
      Ball_and_Plate_MicroLabBox_student_P.Constant9_Value[1];
    Ball_and_Plate_MicroLabBox_student_B.Add[2] =
      Ball_and_Plate_MicroLabBox_student_B.Switch[2] -
      Ball_and_Plate_MicroLabBox_student_P.Constant9_Value[2];
    Ball_and_Plate_MicroLabBox_student_B.Add[3] =
      Ball_and_Plate_MicroLabBox_student_B.Switch[3] -
      Ball_and_Plate_MicroLabBox_student_P.Constant9_Value[3];

    /* DataTypeConversion: '<S72>/mo or x Conversion' */
    Ball_and_Plate_MicroLabBox_student_B.moorxConversion[0] =
      Ball_and_Plate_MicroLabBox_student_B.Add[0];
    Ball_and_Plate_MicroLabBox_student_B.moorxConversion[1] =
      Ball_and_Plate_MicroLabBox_student_B.Add[1];
    Ball_and_Plate_MicroLabBox_student_B.moorxConversion[2] =
      Ball_and_Plate_MicroLabBox_student_B.Add[2];
    Ball_and_Plate_MicroLabBox_student_B.moorxConversion[3] =
      Ball_and_Plate_MicroLabBox_student_B.Add[3];

    /* Gain: '<Root>/Gain7' incorporates:
     *  Constant: '<Root>/ball_freq'
     */
    Ball_and_Plate_MicroLabBox_student_B.Gain7 =
      Ball_and_Plate_MicroLabBox_student_P.Gain7_Gain *
      Ball_and_Plate_MicroLabBox_student_P.ball_freq_Value;

    /* MATLAB Function: '<Root>/MATLAB Function3' incorporates:
     *  Constant: '<Root>/Constant4'
     *  Constant: '<Root>/enable_circ'
     *  Constant: '<Root>/radius_circ'
     */
    /* MATLAB Function 'MATLAB Function3': '<S7>:1' */
    /* '<S7>:1:17' */
    a21 = Ball_and_Plate_MicroLabBox_student_P.radius_circ_Value / 5.0;
    if (Ball_and_Plate_MicroLabBox_student_P.enable_circ_Value != 0.0) {
      /* '<S7>:1:21' */
      P3_global_idx_2 = Ball_and_Plate_MicroLabBox_student_P.radius_circ_Value;
    } else {
      /* '<S7>:1:23' */
      P3_global_idx_2 = 0.0;
    }

    if (Ball_and_Plate_MicroLabBox_student_DW.current_R < P3_global_idx_2) {
      /* '<S7>:1:27' */
      /* '<S7>:1:28' */
      uX_idx_0 = a21;

      /* '<S7>:1:29' */
      Ball_and_Plate_MicroLabBox_student_DW.current_R += a21 *
        Ball_and_Plate_MicroLabBox_student_P.Ts_Inner;
      if (Ball_and_Plate_MicroLabBox_student_DW.current_R > P3_global_idx_2) {
        /* '<S7>:1:30' */
        /* '<S7>:1:31' */
        Ball_and_Plate_MicroLabBox_student_DW.current_R = P3_global_idx_2;

        /* '<S7>:1:32' */
        uX_idx_0 = 0.0;
      }
    } else if (Ball_and_Plate_MicroLabBox_student_DW.current_R > P3_global_idx_2)
    {
      /* '<S7>:1:34' */
      /* '<S7>:1:35' */
      uX_idx_0 = -a21;

      /* '<S7>:1:36' */
      Ball_and_Plate_MicroLabBox_student_DW.current_R += -a21 *
        Ball_and_Plate_MicroLabBox_student_P.Ts_Inner;
      if (Ball_and_Plate_MicroLabBox_student_DW.current_R < P3_global_idx_2) {
        /* '<S7>:1:37' */
        /* '<S7>:1:38' */
        Ball_and_Plate_MicroLabBox_student_DW.current_R = P3_global_idx_2;

        /* '<S7>:1:39' */
        uX_idx_0 = 0.0;
      }
    } else {
      /* '<S7>:1:42' */
      uX_idx_0 = 0.0;
    }

    /* '<S7>:1:47' */
    /* '<S7>:1:48' */
    /* '<S7>:1:50' */
    /* '<S7>:1:51' */
    /* '<S7>:1:53' */
    Ball_and_Plate_MicroLabBox_student_B.r[0] =
      Ball_and_Plate_MicroLabBox_student_DW.current_R * cos
      (Ball_and_Plate_MicroLabBox_student_DW.theta);
    Ball_and_Plate_MicroLabBox_student_B.r[1] = uX_idx_0 * cos
      (Ball_and_Plate_MicroLabBox_student_DW.theta) -
      Ball_and_Plate_MicroLabBox_student_DW.current_R *
      Ball_and_Plate_MicroLabBox_student_B.Gain7 * sin
      (Ball_and_Plate_MicroLabBox_student_DW.theta);
    Ball_and_Plate_MicroLabBox_student_B.r[2] =
      Ball_and_Plate_MicroLabBox_student_DW.current_R * sin
      (Ball_and_Plate_MicroLabBox_student_DW.theta);
    Ball_and_Plate_MicroLabBox_student_B.r[3] =
      Ball_and_Plate_MicroLabBox_student_DW.current_R *
      Ball_and_Plate_MicroLabBox_student_B.Gain7 * cos
      (Ball_and_Plate_MicroLabBox_student_DW.theta) + uX_idx_0 * sin
      (Ball_and_Plate_MicroLabBox_student_DW.theta);

    /* '<S7>:1:56' */
    a21 = Ball_and_Plate_MicroLabBox_student_B.Gain7 *
      Ball_and_Plate_MicroLabBox_student_P.Ts_Inner +
      Ball_and_Plate_MicroLabBox_student_DW.theta;

    /* '<S7>:1:59' */
    if (rtIsNaN(a21) || rtIsInf(a21)) {
      Ball_and_Plate_MicroLabBox_student_DW.theta = (rtNaN);
    } else if (a21 == 0.0) {
      Ball_and_Plate_MicroLabBox_student_DW.theta = 0.0;
    } else {
      Ball_and_Plate_MicroLabBox_student_DW.theta = fmod(a21, 6.2831853071795862);
      rEQ0 = (Ball_and_Plate_MicroLabBox_student_DW.theta == 0.0);
      if (!rEQ0) {
        P3_global_idx_2 = fabs(a21 / 6.2831853071795862);
        rEQ0 = !(fabs(P3_global_idx_2 - floor(P3_global_idx_2 + 0.5)) >
                 2.2204460492503131E-16 * P3_global_idx_2);
      }

      if (rEQ0) {
        Ball_and_Plate_MicroLabBox_student_DW.theta = 0.0;
      } else {
        if (a21 < 0.0) {
          Ball_and_Plate_MicroLabBox_student_DW.theta += 6.2831853071795862;
        }
      }
    }

    /* End of MATLAB Function: '<Root>/MATLAB Function3' */

    /* DataTypeConversion: '<S72>/Data Type Conversion1' */
    Ball_and_Plate_MicroLabBox_student_B.DataTypeConversion1[0] =
      Ball_and_Plate_MicroLabBox_student_B.r[0];
    Ball_and_Plate_MicroLabBox_student_B.DataTypeConversion1[1] =
      Ball_and_Plate_MicroLabBox_student_B.r[1];
    Ball_and_Plate_MicroLabBox_student_B.DataTypeConversion1[2] =
      Ball_and_Plate_MicroLabBox_student_B.r[2];
    Ball_and_Plate_MicroLabBox_student_B.DataTypeConversion1[3] =
      Ball_and_Plate_MicroLabBox_student_B.r[3];
  }

  if (rtmIsMajorTimeStep(Ball_and_Plate_MicroLabBox_student_M) &&
      Ball_and_Plate_MicroLabBox_student_M->Timing.TaskCounters.TID[2] == 0) {
    /* UnitDelay: '<S72>/last_mv' */
    Ball_and_Plate_MicroLabBox_student_B.last_mv[0] =
      Ball_and_Plate_MicroLabBox_student_DW.last_mv_DSTATE[0];
    Ball_and_Plate_MicroLabBox_student_B.last_mv[1] =
      Ball_and_Plate_MicroLabBox_student_DW.last_mv_DSTATE[1];

    /* DataTypeConversion: '<S72>/Data Type Conversion2' incorporates:
     *  Constant: '<S8>/md_zero'
     */
    Ball_and_Plate_MicroLabBox_student_B.DataTypeConversion2 =
      Ball_and_Plate_MicroLabBox_student_P.md_zero_Value;

    /* DataTypeConversion: '<S72>/Data Type Conversion4' incorporates:
     *  Constant: '<S8>/umin_zero'
     */
    Ball_and_Plate_MicroLabBox_student_B.DataTypeConversion4[0] =
      Ball_and_Plate_MicroLabBox_student_P.umin_zero_Value[0];
    Ball_and_Plate_MicroLabBox_student_B.DataTypeConversion4[1] =
      Ball_and_Plate_MicroLabBox_student_P.umin_zero_Value[1];

    /* Gain: '<S72>/umin_scale' */
    Ball_and_Plate_MicroLabBox_student_B.umin_scale[0] =
      Ball_and_Plate_MicroLabBox_student_P.umin_scale_Gain[0] *
      Ball_and_Plate_MicroLabBox_student_B.DataTypeConversion4[0];
    Ball_and_Plate_MicroLabBox_student_B.umin_scale[1] =
      Ball_and_Plate_MicroLabBox_student_P.umin_scale_Gain[1] *
      Ball_and_Plate_MicroLabBox_student_B.DataTypeConversion4[1];

    /* DataTypeConversion: '<S72>/Data Type Conversion5' incorporates:
     *  Constant: '<S8>/umax_zero'
     */
    Ball_and_Plate_MicroLabBox_student_B.DataTypeConversion5[0] =
      Ball_and_Plate_MicroLabBox_student_P.umax_zero_Value[0];
    Ball_and_Plate_MicroLabBox_student_B.DataTypeConversion5[1] =
      Ball_and_Plate_MicroLabBox_student_P.umax_zero_Value[1];

    /* Gain: '<S72>/umax_scale' */
    Ball_and_Plate_MicroLabBox_student_B.umax_scale[0] =
      Ball_and_Plate_MicroLabBox_student_P.umax_scale_Gain[0] *
      Ball_and_Plate_MicroLabBox_student_B.DataTypeConversion5[0];
    Ball_and_Plate_MicroLabBox_student_B.umax_scale[1] =
      Ball_and_Plate_MicroLabBox_student_P.umax_scale_Gain[1] *
      Ball_and_Plate_MicroLabBox_student_B.DataTypeConversion5[1];

    /* DataTypeConversion: '<S72>/Data Type Conversion6' incorporates:
     *  Constant: '<S8>/ymin_zero'
     */
    Ball_and_Plate_MicroLabBox_student_B.DataTypeConversion6[0] =
      Ball_and_Plate_MicroLabBox_student_P.ymin_zero_Value[0];
    Ball_and_Plate_MicroLabBox_student_B.DataTypeConversion6[1] =
      Ball_and_Plate_MicroLabBox_student_P.ymin_zero_Value[1];
    Ball_and_Plate_MicroLabBox_student_B.DataTypeConversion6[2] =
      Ball_and_Plate_MicroLabBox_student_P.ymin_zero_Value[2];
    Ball_and_Plate_MicroLabBox_student_B.DataTypeConversion6[3] =
      Ball_and_Plate_MicroLabBox_student_P.ymin_zero_Value[3];

    /* Gain: '<S72>/ymin_scale' */
    Ball_and_Plate_MicroLabBox_student_B.ymin_scale[0] =
      Ball_and_Plate_MicroLabBox_student_P.ymin_scale_Gain[0] *
      Ball_and_Plate_MicroLabBox_student_B.DataTypeConversion6[0];
    Ball_and_Plate_MicroLabBox_student_B.ymin_scale[1] =
      Ball_and_Plate_MicroLabBox_student_P.ymin_scale_Gain[1] *
      Ball_and_Plate_MicroLabBox_student_B.DataTypeConversion6[1];
    Ball_and_Plate_MicroLabBox_student_B.ymin_scale[2] =
      Ball_and_Plate_MicroLabBox_student_P.ymin_scale_Gain[2] *
      Ball_and_Plate_MicroLabBox_student_B.DataTypeConversion6[2];
    Ball_and_Plate_MicroLabBox_student_B.ymin_scale[3] =
      Ball_and_Plate_MicroLabBox_student_P.ymin_scale_Gain[3] *
      Ball_and_Plate_MicroLabBox_student_B.DataTypeConversion6[3];

    /* DataTypeConversion: '<S72>/Data Type Conversion7' incorporates:
     *  Constant: '<S8>/ymax_zero'
     */
    Ball_and_Plate_MicroLabBox_student_B.DataTypeConversion7[0] =
      Ball_and_Plate_MicroLabBox_student_P.ymax_zero_Value[0];
    Ball_and_Plate_MicroLabBox_student_B.DataTypeConversion7[1] =
      Ball_and_Plate_MicroLabBox_student_P.ymax_zero_Value[1];
    Ball_and_Plate_MicroLabBox_student_B.DataTypeConversion7[2] =
      Ball_and_Plate_MicroLabBox_student_P.ymax_zero_Value[2];
    Ball_and_Plate_MicroLabBox_student_B.DataTypeConversion7[3] =
      Ball_and_Plate_MicroLabBox_student_P.ymax_zero_Value[3];

    /* Gain: '<S72>/ymax_scale' */
    Ball_and_Plate_MicroLabBox_student_B.ymax_scale[0] =
      Ball_and_Plate_MicroLabBox_student_P.ymax_scale_Gain[0] *
      Ball_and_Plate_MicroLabBox_student_B.DataTypeConversion7[0];
    Ball_and_Plate_MicroLabBox_student_B.ymax_scale[1] =
      Ball_and_Plate_MicroLabBox_student_P.ymax_scale_Gain[1] *
      Ball_and_Plate_MicroLabBox_student_B.DataTypeConversion7[1];
    Ball_and_Plate_MicroLabBox_student_B.ymax_scale[2] =
      Ball_and_Plate_MicroLabBox_student_P.ymax_scale_Gain[2] *
      Ball_and_Plate_MicroLabBox_student_B.DataTypeConversion7[2];
    Ball_and_Plate_MicroLabBox_student_B.ymax_scale[3] =
      Ball_and_Plate_MicroLabBox_student_P.ymax_scale_Gain[3] *
      Ball_and_Plate_MicroLabBox_student_B.DataTypeConversion7[3];

    /* DataTypeConversion: '<S72>/E Conversion' incorporates:
     *  Constant: '<S8>/E_zero'
     */
    Ball_and_Plate_MicroLabBox_student_B.EConversion[0] =
      Ball_and_Plate_MicroLabBox_student_P.E_zero_Value[0];
    Ball_and_Plate_MicroLabBox_student_B.EConversion[1] =
      Ball_and_Plate_MicroLabBox_student_P.E_zero_Value[1];

    /* Gain: '<S72>/umin_scale4' */
    Ball_and_Plate_MicroLabBox_student_B.umin_scale4[0] =
      Ball_and_Plate_MicroLabBox_student_P.umin_scale4_Gain[0] *
      Ball_and_Plate_MicroLabBox_student_B.EConversion[0];

    /* Reshape: '<S72>/Reshape' */
    Ball_and_Plate_MicroLabBox_student_B.Reshape[0] =
      Ball_and_Plate_MicroLabBox_student_B.umin_scale4[0];

    /* Gain: '<S72>/umin_scale4' */
    Ball_and_Plate_MicroLabBox_student_B.umin_scale4[1] =
      Ball_and_Plate_MicroLabBox_student_P.umin_scale4_Gain[1] *
      Ball_and_Plate_MicroLabBox_student_B.EConversion[1];

    /* Reshape: '<S72>/Reshape' */
    Ball_and_Plate_MicroLabBox_student_B.Reshape[1] =
      Ball_and_Plate_MicroLabBox_student_B.umin_scale4[1];

    /* DataTypeConversion: '<S72>/F Conversion' incorporates:
     *  Constant: '<S8>/F_zero'
     */
    Ball_and_Plate_MicroLabBox_student_B.FConversion[0] =
      Ball_and_Plate_MicroLabBox_student_P.F_zero_Value[0];
    Ball_and_Plate_MicroLabBox_student_B.FConversion[1] =
      Ball_and_Plate_MicroLabBox_student_P.F_zero_Value[1];
    Ball_and_Plate_MicroLabBox_student_B.FConversion[2] =
      Ball_and_Plate_MicroLabBox_student_P.F_zero_Value[2];
    Ball_and_Plate_MicroLabBox_student_B.FConversion[3] =
      Ball_and_Plate_MicroLabBox_student_P.F_zero_Value[3];

    /* Gain: '<S72>/ymin_scale1' */
    Ball_and_Plate_MicroLabBox_student_B.ymin_scale1[0] =
      Ball_and_Plate_MicroLabBox_student_P.ymin_scale1_Gain[0] *
      Ball_and_Plate_MicroLabBox_student_B.FConversion[0];

    /* Reshape: '<S72>/Reshape1' */
    Ball_and_Plate_MicroLabBox_student_B.Reshape1[0] =
      Ball_and_Plate_MicroLabBox_student_B.ymin_scale1[0];

    /* Gain: '<S72>/ymin_scale1' */
    Ball_and_Plate_MicroLabBox_student_B.ymin_scale1[1] =
      Ball_and_Plate_MicroLabBox_student_P.ymin_scale1_Gain[1] *
      Ball_and_Plate_MicroLabBox_student_B.FConversion[1];

    /* Reshape: '<S72>/Reshape1' */
    Ball_and_Plate_MicroLabBox_student_B.Reshape1[1] =
      Ball_and_Plate_MicroLabBox_student_B.ymin_scale1[1];

    /* Gain: '<S72>/ymin_scale1' */
    Ball_and_Plate_MicroLabBox_student_B.ymin_scale1[2] =
      Ball_and_Plate_MicroLabBox_student_P.ymin_scale1_Gain[2] *
      Ball_and_Plate_MicroLabBox_student_B.FConversion[2];

    /* Reshape: '<S72>/Reshape1' */
    Ball_and_Plate_MicroLabBox_student_B.Reshape1[2] =
      Ball_and_Plate_MicroLabBox_student_B.ymin_scale1[2];

    /* Gain: '<S72>/ymin_scale1' */
    Ball_and_Plate_MicroLabBox_student_B.ymin_scale1[3] =
      Ball_and_Plate_MicroLabBox_student_P.ymin_scale1_Gain[3] *
      Ball_and_Plate_MicroLabBox_student_B.FConversion[3];

    /* Reshape: '<S72>/Reshape1' */
    Ball_and_Plate_MicroLabBox_student_B.Reshape1[3] =
      Ball_and_Plate_MicroLabBox_student_B.ymin_scale1[3];

    /* DataTypeConversion: '<S72>/G Conversion' incorporates:
     *  Constant: '<S8>/G_zero'
     */
    Ball_and_Plate_MicroLabBox_student_B.GConversion =
      Ball_and_Plate_MicroLabBox_student_P.G_zero_Value;

    /* DataTypeConversion: '<S72>/S Conversion' incorporates:
     *  Constant: '<S8>/S_zero'
     */
    Ball_and_Plate_MicroLabBox_student_B.SConversion =
      Ball_and_Plate_MicroLabBox_student_P.S_zero_Value;

    /* Gain: '<S72>/ymin_scale2' */
    Ball_and_Plate_MicroLabBox_student_B.ymin_scale2 =
      Ball_and_Plate_MicroLabBox_student_P.ymin_scale2_Gain *
      Ball_and_Plate_MicroLabBox_student_B.SConversion;

    /* Reshape: '<S72>/Reshape2' */
    Ball_and_Plate_MicroLabBox_student_B.Reshape2 =
      Ball_and_Plate_MicroLabBox_student_B.ymin_scale2;

    /* DataTypeConversion: '<S72>/Data Type Conversion8' incorporates:
     *  Constant: '<S8>/switch_zero'
     */
    Ball_and_Plate_MicroLabBox_student_B.DataTypeConversion8 =
      Ball_and_Plate_MicroLabBox_student_P.switch_zero_Value;

    /* DataTypeConversion: '<S72>/Data Type Conversion3' incorporates:
     *  Constant: '<S8>/ext.mv_zero'
     */
    Ball_and_Plate_MicroLabBox_student_B.DataTypeConversion3[0] =
      Ball_and_Plate_MicroLabBox_student_P.extmv_zero_Value[0];
    Ball_and_Plate_MicroLabBox_student_B.DataTypeConversion3[1] =
      Ball_and_Plate_MicroLabBox_student_P.extmv_zero_Value[1];

    /* Gain: '<S72>/ext.mv_scale' */
    Ball_and_Plate_MicroLabBox_student_B.extmv_scale[0] =
      Ball_and_Plate_MicroLabBox_student_P.extmv_scale_Gain[0] *
      Ball_and_Plate_MicroLabBox_student_B.DataTypeConversion3[0];
    Ball_and_Plate_MicroLabBox_student_B.extmv_scale[1] =
      Ball_and_Plate_MicroLabBox_student_P.extmv_scale_Gain[1] *
      Ball_and_Plate_MicroLabBox_student_B.DataTypeConversion3[1];

    /* DataTypeConversion: '<S72>/Data Type Conversion13' incorporates:
     *  Constant: '<S8>/mv.target_zero'
     */
    Ball_and_Plate_MicroLabBox_student_B.DataTypeConversion13[0] =
      Ball_and_Plate_MicroLabBox_student_P.mvtarget_zero_Value[0];
    Ball_and_Plate_MicroLabBox_student_B.DataTypeConversion13[1] =
      Ball_and_Plate_MicroLabBox_student_P.mvtarget_zero_Value[1];

    /* Gain: '<S72>/ext.mv_scale1' */
    Ball_and_Plate_MicroLabBox_student_B.extmv_scale1[0] =
      Ball_and_Plate_MicroLabBox_student_P.extmv_scale1_Gain[0] *
      Ball_and_Plate_MicroLabBox_student_B.DataTypeConversion13[0];
    Ball_and_Plate_MicroLabBox_student_B.extmv_scale1[1] =
      Ball_and_Plate_MicroLabBox_student_P.extmv_scale1_Gain[1] *
      Ball_and_Plate_MicroLabBox_student_B.DataTypeConversion13[1];

    /* DataTypeConversion: '<S72>/Data Type Conversion9' incorporates:
     *  Constant: '<S8>/y.wt_zero'
     */
    Ball_and_Plate_MicroLabBox_student_B.DataTypeConversion9[0] =
      Ball_and_Plate_MicroLabBox_student_P.ywt_zero_Value[0];
    Ball_and_Plate_MicroLabBox_student_B.DataTypeConversion9[1] =
      Ball_and_Plate_MicroLabBox_student_P.ywt_zero_Value[1];
    Ball_and_Plate_MicroLabBox_student_B.DataTypeConversion9[2] =
      Ball_and_Plate_MicroLabBox_student_P.ywt_zero_Value[2];
    Ball_and_Plate_MicroLabBox_student_B.DataTypeConversion9[3] =
      Ball_and_Plate_MicroLabBox_student_P.ywt_zero_Value[3];

    /* Math: '<S72>/Math Function' */
    Ball_and_Plate_MicroLabBox_student_B.MathFunction[0] =
      Ball_and_Plate_MicroLabBox_student_B.DataTypeConversion9[0];

    /* Reshape: '<S72>/Reshape3' */
    Ball_and_Plate_MicroLabBox_student_B.Reshape3[0] =
      Ball_and_Plate_MicroLabBox_student_B.MathFunction[0];

    /* Math: '<S72>/Math Function' */
    Ball_and_Plate_MicroLabBox_student_B.MathFunction[1] =
      Ball_and_Plate_MicroLabBox_student_B.DataTypeConversion9[1];

    /* Reshape: '<S72>/Reshape3' */
    Ball_and_Plate_MicroLabBox_student_B.Reshape3[1] =
      Ball_and_Plate_MicroLabBox_student_B.MathFunction[1];

    /* Math: '<S72>/Math Function' */
    Ball_and_Plate_MicroLabBox_student_B.MathFunction[2] =
      Ball_and_Plate_MicroLabBox_student_B.DataTypeConversion9[2];

    /* Reshape: '<S72>/Reshape3' */
    Ball_and_Plate_MicroLabBox_student_B.Reshape3[2] =
      Ball_and_Plate_MicroLabBox_student_B.MathFunction[2];

    /* Math: '<S72>/Math Function' */
    Ball_and_Plate_MicroLabBox_student_B.MathFunction[3] =
      Ball_and_Plate_MicroLabBox_student_B.DataTypeConversion9[3];

    /* Reshape: '<S72>/Reshape3' */
    Ball_and_Plate_MicroLabBox_student_B.Reshape3[3] =
      Ball_and_Plate_MicroLabBox_student_B.MathFunction[3];

    /* DataTypeConversion: '<S72>/Data Type Conversion10' incorporates:
     *  Constant: '<S8>/u.wt_zero'
     */
    Ball_and_Plate_MicroLabBox_student_B.DataTypeConversion10[0] =
      Ball_and_Plate_MicroLabBox_student_P.uwt_zero_Value[0];
    Ball_and_Plate_MicroLabBox_student_B.DataTypeConversion10[1] =
      Ball_and_Plate_MicroLabBox_student_P.uwt_zero_Value[1];

    /* Math: '<S72>/Math Function1' */
    Ball_and_Plate_MicroLabBox_student_B.MathFunction1[0] =
      Ball_and_Plate_MicroLabBox_student_B.DataTypeConversion10[0];

    /* Reshape: '<S72>/Reshape4' */
    Ball_and_Plate_MicroLabBox_student_B.Reshape4[0] =
      Ball_and_Plate_MicroLabBox_student_B.MathFunction1[0];

    /* Math: '<S72>/Math Function1' */
    Ball_and_Plate_MicroLabBox_student_B.MathFunction1[1] =
      Ball_and_Plate_MicroLabBox_student_B.DataTypeConversion10[1];

    /* Reshape: '<S72>/Reshape4' */
    Ball_and_Plate_MicroLabBox_student_B.Reshape4[1] =
      Ball_and_Plate_MicroLabBox_student_B.MathFunction1[1];

    /* DataTypeConversion: '<S72>/Data Type Conversion12' incorporates:
     *  Constant: '<S8>/du.wt_zero'
     */
    Ball_and_Plate_MicroLabBox_student_B.DataTypeConversion12[0] =
      Ball_and_Plate_MicroLabBox_student_P.duwt_zero_Value[0];
    Ball_and_Plate_MicroLabBox_student_B.DataTypeConversion12[1] =
      Ball_and_Plate_MicroLabBox_student_P.duwt_zero_Value[1];

    /* Math: '<S72>/Math Function2' */
    Ball_and_Plate_MicroLabBox_student_B.MathFunction2[0] =
      Ball_and_Plate_MicroLabBox_student_B.DataTypeConversion12[0];

    /* Reshape: '<S72>/Reshape5' */
    Ball_and_Plate_MicroLabBox_student_B.Reshape5[0] =
      Ball_and_Plate_MicroLabBox_student_B.MathFunction2[0];

    /* Math: '<S72>/Math Function2' */
    Ball_and_Plate_MicroLabBox_student_B.MathFunction2[1] =
      Ball_and_Plate_MicroLabBox_student_B.DataTypeConversion12[1];

    /* Reshape: '<S72>/Reshape5' */
    Ball_and_Plate_MicroLabBox_student_B.Reshape5[1] =
      Ball_and_Plate_MicroLabBox_student_B.MathFunction2[1];

    /* DataTypeConversion: '<S72>/Data Type Conversion11' incorporates:
     *  Constant: '<S8>/ecr.wt_zero'
     */
    Ball_and_Plate_MicroLabBox_student_B.DataTypeConversion11 =
      Ball_and_Plate_MicroLabBox_student_P.ecrwt_zero_Value;

    /* Memory: '<S72>/Memory' */
    memcpy(&Ball_and_Plate_MicroLabBox_student_B.Memory[0],
           &Ball_and_Plate_MicroLabBox_student_DW.Memory_PreviousInput[0], 212U *
           sizeof(boolean_T));

    /* MATLAB Function: '<S92>/optimizer' */
    /* MATLAB Function 'MPC Controller/MPC/optimizer/optimizer': '<S93>:1' */
    /* '<S93>:1:53' */
    /* '<S93>:1:139' */
    /* '<S93>:1:140' */
    /* '<S93>:1:143' */
    for (r1 = 0; r1 < 26; r1++) {
      vseq[r1] = 1.0;
    }

    memset(&rseq[0], 0, 100U * sizeof(real_T));
    for (r1 = 0; r1 < 25; r1++) {
      rseq[r1 * (int32_T)Ball_and_Plate_MicroLabBox_s_ny] =
        Ball_and_Plate_MicroLabBox_student_B.DataTypeConversion1[0];
      rseq[r1 * (int32_T)Ball_and_Plate_MicroLabBox_s_ny + 1] =
        Ball_and_Plate_MicroLabBox_student_B.DataTypeConversion1[1];
      rseq[r1 * (int32_T)Ball_and_Plate_MicroLabBox_s_ny + 2] =
        Ball_and_Plate_MicroLabBox_student_B.DataTypeConversion1[2];
      rseq[r1 * (int32_T)Ball_and_Plate_MicroLabBox_s_ny + 3] =
        Ball_and_Plate_MicroLabBox_student_B.DataTypeConversion1[3];
    }

    /* '<S93>:1:58' */
    old_u[0] = Ball_and_Plate_MicroLabBox_student_B.last_mv[0];
    old_u[1] = Ball_and_Plate_MicroLabBox_student_B.last_mv[1];

    /* '<S93>:1:68' */
    Ball_and_Plate_MicroLabBox_student_B.xest[0] =
      Ball_and_Plate_MicroLabBox_student_B.moorxConversion[0];
    Ball_and_Plate_MicroLabBox_student_B.xest[1] =
      Ball_and_Plate_MicroLabBox_student_B.moorxConversion[1];
    Ball_and_Plate_MicroLabBox_student_B.xest[2] =
      Ball_and_Plate_MicroLabBox_student_B.moorxConversion[2];
    Ball_and_Plate_MicroLabBox_student_B.xest[3] =
      Ball_and_Plate_MicroLabBox_student_B.moorxConversion[3];

    /* '<S93>:1:137' */
    memcpy(&Ball_and_Plate_MicroLabBox_student_B.iAout[0],
           &Ball_and_Plate_MicroLabBox_student_B.Memory[0], 212U * sizeof
           (boolean_T));
    memset(&tmp[0], 0, 156U * sizeof(real_T));
    memset(&tmp_0[0], 0, 50U * sizeof(real_T));
    for (r1 = 0; r1 < 212; r1++) {
      a21 = b_Mx[r1] * Ball_and_Plate_MicroLabBox_student_B.xest[0];
      a21 += b_Mx[r1 + 212] * Ball_and_Plate_MicroLabBox_student_B.xest[1];
      a21 += b_Mx[r1 + 424] * Ball_and_Plate_MicroLabBox_student_B.xest[2];
      a21 += b_Mx[r1 + 636] * Ball_and_Plate_MicroLabBox_student_B.xest[3];
      b_Mlim_0[r1] = b_Mlim_1[r1] + a21;
    }

    for (r1 = 0; r1 < 212; r1++) {
      a21 = b_Mu1[r1] * old_u[0];
      a21 += b_Mu1[r1 + 212] * old_u[1];
      b_Mlim[r1] = -(b_Mlim_0[r1] + a21);
    }

    Ball_and_Plate_Micr_mpc_solveQP(Ball_and_Plate_MicroLabBox_student_B.xest,
      b_Kx, b_Kr, rseq, b_Ku1, old_u, tmp, vseq, b_Kut, tmp_0, b_Linv, b_Hinv,
      b_Ac, b_Mlim, Ball_and_Plate_MicroLabBox_student_B.iAout, zopt, f, &a21);
    Ball_and_Plate_MicroLabBox_student_B.u[0] = old_u[0] + zopt[0];
    Ball_and_Plate_MicroLabBox_student_B.u[1] = old_u[1] + zopt[1];

    /* '<S93>:1:137' */
    Ball_and_Plate_MicroLabBox_student_B.cost = 0.0;

    /* '<S93>:1:137' */
    memset(&Ball_and_Plate_MicroLabBox_student_B.useq[0], 0, 52U * sizeof(real_T));

    /* '<S93>:1:137' */
    Ball_and_Plate_MicroLabBox_student_B.status = a21;

    /* '<S93>:1:137' */
    /* '<S93>:1:153' */
    memset(&Ball_and_Plate_MicroLabBox_student_B.yseq[0], 0, 104U * sizeof
           (real_T));

    /* '<S93>:1:154' */
    memset(&Ball_and_Plate_MicroLabBox_student_B.xseq[0], 0, 104U * sizeof
           (real_T));

    /* '<S93>:1:163' */
    Ball_and_Plate_MicroLabBox_student_B.xk1[0] =
      Ball_and_Plate_MicroLabBox_student_B.xest[0];
    Ball_and_Plate_MicroLabBox_student_B.xk1[1] =
      Ball_and_Plate_MicroLabBox_student_B.xest[1];
    Ball_and_Plate_MicroLabBox_student_B.xk1[2] =
      Ball_and_Plate_MicroLabBox_student_B.xest[2];
    Ball_and_Plate_MicroLabBox_student_B.xk1[3] =
      Ball_and_Plate_MicroLabBox_student_B.xest[3];

    /* End of MATLAB Function: '<S92>/optimizer' */

    /* Gain: '<S72>/umin_scale1' */
    /* '<S93>:1:166' */
    Ball_and_Plate_MicroLabBox_student_B.umin_scale1[0] =
      Ball_and_Plate_MicroLabBox_student_P.umin_scale1_Gain[0] *
      Ball_and_Plate_MicroLabBox_student_B.u[0];
    Ball_and_Plate_MicroLabBox_student_B.umin_scale1[1] =
      Ball_and_Plate_MicroLabBox_student_P.umin_scale1_Gain[1] *
      Ball_and_Plate_MicroLabBox_student_B.u[1];
  }

  if (rtmIsMajorTimeStep(Ball_and_Plate_MicroLabBox_student_M) &&
      Ball_and_Plate_MicroLabBox_student_M->Timing.TaskCounters.TID[1] == 0) {
    /* SampleTimeMath: '<S2>/TSamp'
     *
     * About '<S2>/TSamp':
     *  y = u * K where K = 1 / ( w * Ts )
     */
    Ball_and_Plate_MicroLabBox_student_B.TSamp =
      Ball_and_Plate_MicroLabBox_student_B.r[1] *
      Ball_and_Plate_MicroLabBox_student_P.TSamp_WtEt;

    /* UnitDelay: '<S2>/UD' */
    Ball_and_Plate_MicroLabBox_student_B.Uk1_h =
      Ball_and_Plate_MicroLabBox_student_DW.UD_DSTATE_c;

    /* Sum: '<S2>/Diff' */
    Ball_and_Plate_MicroLabBox_student_B.Diff_e =
      Ball_and_Plate_MicroLabBox_student_B.TSamp -
      Ball_and_Plate_MicroLabBox_student_B.Uk1_h;

    /* Switch: '<Root>/Switch1' incorporates:
     *  Constant: '<Root>/Constant7'
     *  Constant: '<Root>/ff_enable'
     */
    if (Ball_and_Plate_MicroLabBox_student_P.ff_enable_Value >
        Ball_and_Plate_MicroLabBox_student_P.Switch1_Threshold) {
      /* Gain: '<Root>/Gain8' */
      a21 = 7.0 / (5.0 * Ball_and_Plate_MicroLabBox_student_P.g) / 10.0;
      Ball_and_Plate_MicroLabBox_student_B.Gain8 = a21 *
        Ball_and_Plate_MicroLabBox_student_B.Diff_e;
      Ball_and_Plate_MicroLabBox_student_B.Switch1 =
        Ball_and_Plate_MicroLabBox_student_B.Gain8;
    } else {
      Ball_and_Plate_MicroLabBox_student_B.Switch1 =
        Ball_and_Plate_MicroLabBox_student_P.Constant7_Value;
    }

    /* End of Switch: '<Root>/Switch1' */

    /* Sum: '<Root>/Add1' */
    Ball_and_Plate_MicroLabBox_student_B.Add1 =
      Ball_and_Plate_MicroLabBox_student_B.umin_scale1[0] -
      Ball_and_Plate_MicroLabBox_student_B.Switch1;

    /* SampleTimeMath: '<S3>/TSamp'
     *
     * About '<S3>/TSamp':
     *  y = u * K where K = 1 / ( w * Ts )
     */
    Ball_and_Plate_MicroLabBox_student_B.TSamp_p =
      Ball_and_Plate_MicroLabBox_student_B.r[3] *
      Ball_and_Plate_MicroLabBox_student_P.TSamp_WtEt_b;

    /* UnitDelay: '<S3>/UD' */
    Ball_and_Plate_MicroLabBox_student_B.Uk1_d =
      Ball_and_Plate_MicroLabBox_student_DW.UD_DSTATE_o;

    /* Sum: '<S3>/Diff' */
    Ball_and_Plate_MicroLabBox_student_B.Diff_e2 =
      Ball_and_Plate_MicroLabBox_student_B.TSamp_p -
      Ball_and_Plate_MicroLabBox_student_B.Uk1_d;

    /* Switch: '<Root>/Switch2' incorporates:
     *  Constant: '<Root>/Constant8'
     *  Constant: '<Root>/ff_enable'
     */
    if (Ball_and_Plate_MicroLabBox_student_P.ff_enable_Value >
        Ball_and_Plate_MicroLabBox_student_P.Switch2_Threshold) {
      /* Gain: '<Root>/Gain9' */
      a21 = 7.0 / (5.0 * Ball_and_Plate_MicroLabBox_student_P.g) / 10.0;
      Ball_and_Plate_MicroLabBox_student_B.Gain9 = a21 *
        Ball_and_Plate_MicroLabBox_student_B.Diff_e2;
      Ball_and_Plate_MicroLabBox_student_B.Switch2 =
        Ball_and_Plate_MicroLabBox_student_B.Gain9;
    } else {
      Ball_and_Plate_MicroLabBox_student_B.Switch2 =
        Ball_and_Plate_MicroLabBox_student_P.Constant8_Value;
    }

    /* End of Switch: '<Root>/Switch2' */

    /* Sum: '<Root>/Add2' */
    Ball_and_Plate_MicroLabBox_student_B.Add2 =
      Ball_and_Plate_MicroLabBox_student_B.umin_scale1[1] -
      Ball_and_Plate_MicroLabBox_student_B.Switch2;

    /* Gain: '<S12>/Gain1' */
    Ball_and_Plate_MicroLabBox_student_B.Gain1_c =
      Ball_and_Plate_MicroLabBox_student_P.Gain1_Gain_bk *
      Ball_and_Plate_MicroLabBox_student_B.Add2;

    /* S-Function (dlowpass1): '<S12>/Dct1lowpass2' */

    /* Level2 S-Function Block: '<S12>/Dct1lowpass2' (dlowpass1) */
    {
      SimStruct *rts = Ball_and_Plate_MicroLabBox_student_M->childSfunctions[4];
      sfcnOutputs(rts,1);
    }

    /* Gain: '<Root>/Gain4' */
    Ball_and_Plate_MicroLabBox_student_B.Gain4 =
      Ball_and_Plate_MicroLabBox_student_P.Gain4_Gain *
      Ball_and_Plate_MicroLabBox_student_B.Dct1lowpass2_b2;

    /* Saturate: '<Root>/Alpha_sat ' */
    a21 = -Ball_and_Plate_MicroLabBox_student_P.plate_angle_sat;
    P3_global_idx_2 = Ball_and_Plate_MicroLabBox_student_B.Gain4;
    uX_idx_0 = Ball_and_Plate_MicroLabBox_student_P.plate_angle_sat;
    if (P3_global_idx_2 > uX_idx_0) {
      Ball_and_Plate_MicroLabBox_student_B.Alpha_sat = uX_idx_0;
    } else if (P3_global_idx_2 < a21) {
      Ball_and_Plate_MicroLabBox_student_B.Alpha_sat = a21;
    } else {
      Ball_and_Plate_MicroLabBox_student_B.Alpha_sat = P3_global_idx_2;
    }

    /* End of Saturate: '<Root>/Alpha_sat ' */

    /* Gain: '<S13>/Gain1' */
    Ball_and_Plate_MicroLabBox_student_B.Gain1_d =
      Ball_and_Plate_MicroLabBox_student_P.Gain1_Gain_by *
      Ball_and_Plate_MicroLabBox_student_B.Add1;

    /* S-Function (dlowpass1): '<S13>/Dct1lowpass2' */

    /* Level2 S-Function Block: '<S13>/Dct1lowpass2' (dlowpass1) */
    {
      SimStruct *rts = Ball_and_Plate_MicroLabBox_student_M->childSfunctions[5];
      sfcnOutputs(rts,1);
    }

    /* Gain: '<Root>/Gain5' */
    Ball_and_Plate_MicroLabBox_student_B.Gain5 =
      Ball_and_Plate_MicroLabBox_student_P.Gain5_Gain *
      Ball_and_Plate_MicroLabBox_student_B.Dct1lowpass2_g;

    /* Saturate: '<Root>/Beta_sat ' */
    a21 = -Ball_and_Plate_MicroLabBox_student_P.plate_angle_sat;
    P3_global_idx_2 = Ball_and_Plate_MicroLabBox_student_B.Gain5;
    uX_idx_0 = Ball_and_Plate_MicroLabBox_student_P.plate_angle_sat;
    if (P3_global_idx_2 > uX_idx_0) {
      Ball_and_Plate_MicroLabBox_student_B.Beta_sat = uX_idx_0;
    } else if (P3_global_idx_2 < a21) {
      Ball_and_Plate_MicroLabBox_student_B.Beta_sat = a21;
    } else {
      Ball_and_Plate_MicroLabBox_student_B.Beta_sat = P3_global_idx_2;
    }

    /* End of Saturate: '<Root>/Beta_sat ' */

    /* S-Function (rti_commonblock): '<S16>/S-Function1' */
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

    /* S-Function (rti_commonblock): '<S18>/S-Function1' */
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

    /* Gain: '<Root>/Gain2' */
    Ball_and_Plate_MicroLabBox_student_B.Gain2 =
      Ball_and_Plate_MicroLabBox_student_P.Gain2_Gain *
      Ball_and_Plate_MicroLabBox_student_B.x;

    /* Gain: '<Root>/Gain3' */
    Ball_and_Plate_MicroLabBox_student_B.Gain3 =
      Ball_and_Plate_MicroLabBox_student_P.Gain3_Gain *
      Ball_and_Plate_MicroLabBox_student_B.y;

    /* MATLAB Function: '<S5>/AngleToPos ' incorporates:
     *  Constant: '<Root>/Psi_ref '
     */
    /* MATLAB Function 'Innerloop_Actuator/AngleToPos ': '<S20>:1' */
    /* '<S20>:1:12' */
    /* '<S20>:1:16' */
    /* '<S20>:1:17' */
    /* '<S20>:1:18' */
    /* '<S20>:1:23' */
    /* '<S20>:1:7' */
    /* '<S20>:1:27' */
    /* '<S20>:1:31' */
    /* '<S20>:1:19' */
    a21 = cos(Ball_and_Plate_MicroLabBox_student_P.Psi_ref_Value);
    P3_global_idx_2 = sin(Ball_and_Plate_MicroLabBox_student_P.Psi_ref_Value);
    uX_idx_0 = sin(Ball_and_Plate_MicroLabBox_student_P.Psi_ref_Value);
    uX_idx_1 = cos(Ball_and_Plate_MicroLabBox_student_P.Psi_ref_Value);
    uX_idx_2 = cos(Ball_and_Plate_MicroLabBox_student_B.Beta_sat);
    tmp_1 = sin(Ball_and_Plate_MicroLabBox_student_B.Beta_sat);
    tmp_2 = sin(Ball_and_Plate_MicroLabBox_student_B.Beta_sat);
    tmp_3 = cos(Ball_and_Plate_MicroLabBox_student_B.Beta_sat);
    tmp_4[0] = a21;
    tmp_4[3] = -P3_global_idx_2;
    tmp_4[6] = 0.0;
    tmp_4[1] = uX_idx_0;
    tmp_4[4] = uX_idx_1;
    tmp_4[7] = 0.0;
    tmp_5[0] = uX_idx_2;
    tmp_5[3] = 0.0;
    tmp_5[6] = tmp_1;
    tmp_4[2] = 0.0;
    tmp_5[1] = 0.0;
    tmp_4[5] = 0.0;
    tmp_5[4] = 1.0;
    tmp_4[8] = 1.0;
    tmp_5[7] = 0.0;
    tmp_5[2] = -tmp_2;
    tmp_5[5] = 0.0;
    tmp_5[8] = tmp_3;
    a21 = cos(Ball_and_Plate_MicroLabBox_student_B.Alpha_sat);
    P3_global_idx_2 = sin(Ball_and_Plate_MicroLabBox_student_B.Alpha_sat);
    uX_idx_0 = sin(Ball_and_Plate_MicroLabBox_student_B.Alpha_sat);
    uX_idx_1 = cos(Ball_and_Plate_MicroLabBox_student_B.Alpha_sat);
    for (r1 = 0; r1 < 3; r1++) {
      for (r2 = 0; r2 < 3; r2++) {
        tmp_6[r1 + 3 * r2] = 0.0;
        uX_idx_2 = tmp_6[3 * r2 + r1];
        uX_idx_2 += tmp_5[3 * r2] * tmp_4[r1];
        tmp_6[r1 + 3 * r2] = uX_idx_2;
        uX_idx_2 = tmp_6[3 * r2 + r1];
        uX_idx_2 += tmp_5[3 * r2 + 1] * tmp_4[r1 + 3];
        tmp_6[r1 + 3 * r2] = uX_idx_2;
        uX_idx_2 = tmp_6[3 * r2 + r1];
        uX_idx_2 += tmp_5[3 * r2 + 2] * tmp_4[r1 + 6];
        tmp_6[r1 + 3 * r2] = uX_idx_2;
      }

      e[3 * r1] = e_0[r1];
    }

    e[1] = 0.0;
    e[4] = a21;
    e[7] = -P3_global_idx_2;
    e[2] = 0.0;
    e[5] = uX_idx_0;
    e[8] = uX_idx_1;

    /* '<S20>:1:23' */
    /* '<S20>:1:24' */
    for (r1 = 0; r1 < 3; r1++) {
      P3_global_idx_2 = T_1[r1];
      a21 = 0.0;
      for (r2 = 0; r2 < 3; r2++) {
        R_BtoP[r1 + 3 * r2] = 0.0;
        uX_idx_0 = R_BtoP[3 * r2 + r1];
        uX_idx_0 += e[3 * r2] * tmp_6[r1];
        R_BtoP[r1 + 3 * r2] = uX_idx_0;
        uX_idx_0 = R_BtoP[3 * r2 + r1];
        uX_idx_0 += e[3 * r2 + 1] * tmp_6[r1 + 3];
        R_BtoP[r1 + 3 * r2] = uX_idx_0;
        uX_idx_0 = R_BtoP[3 * r2 + r1];
        uX_idx_0 += e[3 * r2 + 2] * tmp_6[r1 + 6];
        R_BtoP[r1 + 3 * r2] = uX_idx_0;
        a21 += R_BtoP[3 * r2 + r1] * b[r2];
      }

      P1_global[r1] = (P3_global_idx_2 + a21) - b[r1];
      a21 = R_BtoP[r1] * -0.085000000000000075;
      a21 += R_BtoP[r1 + 3] * -0.14722431864335456;
      a21 += R_BtoP[r1 + 6] * 0.0;
      T_0[r1] = (P3_global_idx_2 + a21) - c[r1];
      a21 = R_BtoP[r1] * -0.084999999999999964;
      a21 += R_BtoP[r1 + 3] * 0.14722431864335458;
      a21 += R_BtoP[r1 + 6] * 0.0;
      T[r1] = (P3_global_idx_2 + a21) - d[r1];
    }

    Ball_and_Plate_MicroLabBox_student_B.pos1 = Ball_and_Plate_MicroLabBox_norm
      (P1_global);

    /* '<S20>:1:27' */
    /* '<S20>:1:28' */
    Ball_and_Plate_MicroLabBox_student_B.pos2 = Ball_and_Plate_MicroLabBox_norm
      (T_0);

    /* '<S20>:1:31' */
    /* '<S20>:1:32' */
    Ball_and_Plate_MicroLabBox_student_B.pos3 = Ball_and_Plate_MicroLabBox_norm
      (T);

    /* End of MATLAB Function: '<S5>/AngleToPos ' */

    /* Switch: '<S21>/Switch' incorporates:
     *  Constant: '<S21>/Constant3'
     *  Constant: '<S5>/enable_quintic  '
     *  Constant: '<S5>/quintic_ref '
     */
    if (Ball_and_Plate_MicroLabBox_student_P.enable_quintic_Value >
        Ball_and_Plate_MicroLabBox_student_P.Switch_Threshold_j) {
      Ball_and_Plate_MicroLabBox_student_B.Switch_h =
        Ball_and_Plate_MicroLabBox_student_P.Constant3_Value_d;
    } else {
      Ball_and_Plate_MicroLabBox_student_B.Switch_h =
        Ball_and_Plate_MicroLabBox_student_P.quintic_ref_Value;
    }

    /* End of Switch: '<S21>/Switch' */

    /* MATLAB Function: '<S21>/MATLAB Function1' incorporates:
     *  Constant: '<S21>/Constant1'
     *  Constant: '<S21>/Constant2'
     */
    Ball_and_Plate__MATLABFunction1
      (Ball_and_Plate_MicroLabBox_student_B.Switch_h,
       Ball_and_Plate_MicroLabBox_student_P.Constant2_Value_j,
       Ball_and_Plate_MicroLabBox_student_P.Constant1_Value_na,
       &Ball_and_Plate_MicroLabBox_student_B.sf_MATLABFunction1,
       &Ball_and_Plate_MicroLabBox_student_DW.sf_MATLABFunction1);

    /* Switch: '<S21>/enable_ref ' incorporates:
     *  Constant: '<S5>/enable_ref '
     */
    if (Ball_and_Plate_MicroLabBox_student_P.enable_ref_Value >
        Ball_and_Plate_MicroLabBox_student_P.enable_ref_Threshold) {
      Ball_and_Plate_MicroLabBox_student_B.enable_ref =
        Ball_and_Plate_MicroLabBox_student_B.sf_MATLABFunction1.path;
    } else {
      /* Sum: '<S5>/Add1' incorporates:
       *  Constant: '<S5>/Constant'
       */
      Ball_and_Plate_MicroLabBox_student_B.Add1_m =
        Ball_and_Plate_MicroLabBox_student_B.pos1 -
        Ball_and_Plate_MicroLabBox_student_P.Constant_Value;
      Ball_and_Plate_MicroLabBox_student_B.enable_ref =
        Ball_and_Plate_MicroLabBox_student_B.Add1_m;
    }

    /* End of Switch: '<S21>/enable_ref ' */

    /* Sum: '<S21>/Sum1' */
    Ball_and_Plate_MicroLabBox_student_B.Sum1 =
      Ball_and_Plate_MicroLabBox_student_B.enable_ref -
      Ball_and_Plate_MicroLabBox_student_B.mm2m_po;

    /* Gain: '<S30>/Gain1' */
    Ball_and_Plate_MicroLabBox_student_B.Gain1_b4 =
      Ball_and_Plate_MicroLabBox_student_P.Gain1_Gain_j *
      Ball_and_Plate_MicroLabBox_student_B.Sum1;

    /* S-Function (dleadlag): '<S30>/Dctleadlag2' */

    /* Level2 S-Function Block: '<S30>/Dctleadlag2' (dleadlag) */
    {
      SimStruct *rts = Ball_and_Plate_MicroLabBox_student_M->childSfunctions[6];
      sfcnOutputs(rts,1);
    }

    /* S-Function (dlowpass1): '<S30>/Dct1lowpass3' */

    /* Level2 S-Function Block: '<S30>/Dct1lowpass3' (dlowpass1) */
    {
      SimStruct *rts = Ball_and_Plate_MicroLabBox_student_M->childSfunctions[7];
      sfcnOutputs(rts,1);
    }

    /* Switch: '<S21>/Switch2' incorporates:
     *  Constant: '<S21>/Constant'
     *  Constant: '<S5>/controller_disable'
     */
    if (Ball_and_Plate_MicroLabBox_student_P.controller_disable_Value >=
        Ball_and_Plate_MicroLabBox_student_P.Switch2_Threshold_g) {
      Ball_and_Plate_MicroLabBox_student_B.Switch2_k =
        Ball_and_Plate_MicroLabBox_student_P.Constant_Value_h;
    } else {
      Ball_and_Plate_MicroLabBox_student_B.Switch2_k =
        Ball_and_Plate_MicroLabBox_student_B.Dct1lowpass3;
    }

    /* End of Switch: '<S21>/Switch2' */

    /* MATLAB Function: '<S27>/MATLAB Function' incorporates:
     *  Constant: '<S21>/enable_ID_A '
     *  Constant: '<S27>/Constant'
     */
    Ball_and_Plate_MATLABFunction_e
      (Ball_and_Plate_MicroLabBox_student_P.enable_ID_A_Value,
       Ball_and_Plate_MicroLabBox_student_P.uA,
       &Ball_and_Plate_MicroLabBox_student_B.sf_MATLABFunction_e,
       &Ball_and_Plate_MicroLabBox_student_DW.sf_MATLABFunction_e);

    /* Sum: '<S21>/Sum' */
    Ball_and_Plate_MicroLabBox_student_B.Sum =
      Ball_and_Plate_MicroLabBox_student_B.Switch2_k +
      Ball_and_Plate_MicroLabBox_student_B.sf_MATLABFunction_e.y;
  }

  /* Switch: '<S21>/Switch3' incorporates:
   *  Constant: '<S5>/CloseLoop_disable'
   */
  if (Ball_and_Plate_MicroLabBox_student_P.CloseLoop_disable_Value >=
      Ball_and_Plate_MicroLabBox_student_P.Switch3_Threshold) {
    Ball_and_Plate_MicroLabBox_student_B.Switch3 =
      Ball_and_Plate_MicroLabBox_student_B.TransferFcn_f;
  } else {
    Ball_and_Plate_MicroLabBox_student_B.Switch3 =
      Ball_and_Plate_MicroLabBox_student_B.Sum;
  }

  /* End of Switch: '<S21>/Switch3' */

  /* Outputs for Atomic SubSystem: '<S21>/Outputs to Amplifier' */

  /* Saturate: '<S28>/Saturation' */
  P3_global_idx_2 = Ball_and_Plate_MicroLabBox_student_B.Switch3;
  a21 = Ball_and_Plate_MicroLabBox_student_P.Saturation_LowerSat;
  uX_idx_0 = Ball_and_Plate_MicroLabBox_student_P.Saturation_UpperSat;
  if (P3_global_idx_2 > uX_idx_0) {
    Ball_and_Plate_MicroLabBox_student_B.Saturation_a = uX_idx_0;
  } else if (P3_global_idx_2 < a21) {
    Ball_and_Plate_MicroLabBox_student_B.Saturation_a = a21;
  } else {
    Ball_and_Plate_MicroLabBox_student_B.Saturation_a = P3_global_idx_2;
  }

  /* End of Saturate: '<S28>/Saturation' */

  /* Gain: '<S28>/Current2V' */
  Ball_and_Plate_MicroLabBox_student_B.Current2V_h =
    Ball_and_Plate_MicroLabBox_student_P.Current2V_Gain *
    Ball_and_Plate_MicroLabBox_student_B.Saturation_a;

  /* Gain: '<S28>/DSPscale' */
  Ball_and_Plate_MicroLabBox_student_B.DSPscale_m =
    Ball_and_Plate_MicroLabBox_student_P.DSPscale_Gain *
    Ball_and_Plate_MicroLabBox_student_B.Current2V_h;
  if (rtmIsMajorTimeStep(Ball_and_Plate_MicroLabBox_student_M) &&
      Ball_and_Plate_MicroLabBox_student_M->Timing.TaskCounters.TID[1] == 0) {
    /* S-Function (rti_commonblock): '<S34>/S-Function1' */
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

  /* End of Outputs for SubSystem: '<S21>/Outputs to Amplifier' */
  if (rtmIsMajorTimeStep(Ball_and_Plate_MicroLabBox_student_M) &&
      Ball_and_Plate_MicroLabBox_student_M->Timing.TaskCounters.TID[1] == 0) {
    /* MATLAB Function: '<S25>/MATLAB Function' incorporates:
     *  Constant: '<S25>/Constant4'
     */
    Ball_and_Plate_M_MATLABFunction(0.0,
      Ball_and_Plate_MicroLabBox_student_P.path,
      &Ball_and_Plate_MicroLabBox_student_B.sf_MATLABFunction_i,
      &Ball_and_Plate_MicroLabBox_student_DW.sf_MATLABFunction_i);

    /* MATLAB Function: '<S25>/MATLAB Function2' incorporates:
     *  Constant: '<S21>/1_no_0_init_motion'
     *  Constant: '<S21>/Constant4'
     */
    Ball_and_Plate__MATLABFunction2(0.0,
      Ball_and_Plate_MicroLabBox_student_P.u_no_0_init_motion_Value,
      Ball_and_Plate_MicroLabBox_student_P.Constant4_Value,
      &Ball_and_Plate_MicroLabBox_student_B.sf_MATLABFunction2);

    /* Switch: '<S25>/Switch' incorporates:
     *  Constant: '<S21>/1_no_0_init_motion'
     *  Constant: '<S21>/Constant4'
     *  Constant: '<S25>/Constant3'
     */
    if (Ball_and_Plate_MicroLabBox_student_P.u_no_0_init_motion_Value >
        Ball_and_Plate_MicroLabBox_student_P.Switch_Threshold_c) {
      Ball_and_Plate_MicroLabBox_student_B.Switch_f =
        Ball_and_Plate_MicroLabBox_student_P.Constant4_Value;
    } else {
      Ball_and_Plate_MicroLabBox_student_B.Switch_f =
        Ball_and_Plate_MicroLabBox_student_P.Constant3_Value_b;
    }

    /* End of Switch: '<S25>/Switch' */

    /* Constant: '<S25>/Constant1' */
    Ball_and_Plate_MicroLabBox_student_B.Constant1 =
      Ball_and_Plate_MicroLabBox_student_P.Constant1_Value_i;

    /* Constant: '<S25>/Constant2' */
    Ball_and_Plate_MicroLabBox_student_B.Constant2 =
      Ball_and_Plate_MicroLabBox_student_P.Constant2_Value_e;

    /* Switch: '<S22>/Switch1' incorporates:
     *  Constant: '<S22>/Constant5'
     *  Constant: '<S5>/enable_quintic  '
     *  Constant: '<S5>/quintic_ref '
     */
    if (Ball_and_Plate_MicroLabBox_student_P.enable_quintic_Value >
        Ball_and_Plate_MicroLabBox_student_P.Switch1_Threshold_j) {
      Ball_and_Plate_MicroLabBox_student_B.Switch1_j =
        Ball_and_Plate_MicroLabBox_student_P.Constant5_Value;
    } else {
      Ball_and_Plate_MicroLabBox_student_B.Switch1_j =
        Ball_and_Plate_MicroLabBox_student_P.quintic_ref_Value;
    }

    /* End of Switch: '<S22>/Switch1' */

    /* MATLAB Function: '<S22>/MATLAB Function2' incorporates:
     *  Constant: '<S22>/Constant2'
     *  Constant: '<S22>/Constant3'
     */
    Ball_and_Plate__MATLABFunction1
      (Ball_and_Plate_MicroLabBox_student_B.Switch1_j,
       Ball_and_Plate_MicroLabBox_student_P.Constant3_Value_k,
       Ball_and_Plate_MicroLabBox_student_P.Constant2_Value_c,
       &Ball_and_Plate_MicroLabBox_student_B.sf_MATLABFunction2_d,
       &Ball_and_Plate_MicroLabBox_student_DW.sf_MATLABFunction2_d);

    /* Switch: '<S22>/Switch' incorporates:
     *  Constant: '<S5>/enable_ref '
     */
    if (Ball_and_Plate_MicroLabBox_student_P.enable_ref_Value >
        Ball_and_Plate_MicroLabBox_student_P.Switch_Threshold_a) {
      Ball_and_Plate_MicroLabBox_student_B.Switch_j =
        Ball_and_Plate_MicroLabBox_student_B.sf_MATLABFunction2_d.path;
    } else {
      /* Sum: '<S5>/Add2' incorporates:
       *  Constant: '<S5>/Constant1'
       */
      Ball_and_Plate_MicroLabBox_student_B.Add2_h =
        Ball_and_Plate_MicroLabBox_student_B.pos2 -
        Ball_and_Plate_MicroLabBox_student_P.Constant1_Value_n;
      Ball_and_Plate_MicroLabBox_student_B.Switch_j =
        Ball_and_Plate_MicroLabBox_student_B.Add2_h;
    }

    /* End of Switch: '<S22>/Switch' */

    /* Sum: '<S22>/Sum1' */
    Ball_and_Plate_MicroLabBox_student_B.Sum1_f =
      Ball_and_Plate_MicroLabBox_student_B.Switch_j -
      Ball_and_Plate_MicroLabBox_student_B.mm2m_p;

    /* Gain: '<S45>/Gain1' */
    Ball_and_Plate_MicroLabBox_student_B.Gain1_f =
      Ball_and_Plate_MicroLabBox_student_P.Gain1_Gain_l *
      Ball_and_Plate_MicroLabBox_student_B.Sum1_f;

    /* S-Function (dleadlag): '<S45>/Dctleadlag2' */

    /* Level2 S-Function Block: '<S45>/Dctleadlag2' (dleadlag) */
    {
      SimStruct *rts = Ball_and_Plate_MicroLabBox_student_M->childSfunctions[8];
      sfcnOutputs(rts,1);
    }

    /* S-Function (dlowpass1): '<S45>/Dct1lowpass3' */

    /* Level2 S-Function Block: '<S45>/Dct1lowpass3' (dlowpass1) */
    {
      SimStruct *rts = Ball_and_Plate_MicroLabBox_student_M->childSfunctions[9];
      sfcnOutputs(rts,1);
    }

    /* Switch: '<S22>/Switch2' incorporates:
     *  Constant: '<S22>/Constant1'
     *  Constant: '<S5>/controller_disable'
     */
    if (Ball_and_Plate_MicroLabBox_student_P.controller_disable_Value >=
        Ball_and_Plate_MicroLabBox_student_P.Switch2_Threshold_k) {
      Ball_and_Plate_MicroLabBox_student_B.Switch2_p =
        Ball_and_Plate_MicroLabBox_student_P.Constant1_Value;
    } else {
      Ball_and_Plate_MicroLabBox_student_B.Switch2_p =
        Ball_and_Plate_MicroLabBox_student_B.Dct1lowpass3_c;
    }

    /* End of Switch: '<S22>/Switch2' */

    /* MATLAB Function: '<S42>/MATLAB Function' incorporates:
     *  Constant: '<S22>/enable_ID_B '
     *  Constant: '<S42>/Constant'
     */
    Ball_and_Plate_MATLABFunction_e
      (Ball_and_Plate_MicroLabBox_student_P.enable_ID_B_Value,
       Ball_and_Plate_MicroLabBox_student_P.uB,
       &Ball_and_Plate_MicroLabBox_student_B.sf_MATLABFunction_ko,
       &Ball_and_Plate_MicroLabBox_student_DW.sf_MATLABFunction_ko);

    /* Sum: '<S22>/Sum' */
    Ball_and_Plate_MicroLabBox_student_B.Sum_g =
      Ball_and_Plate_MicroLabBox_student_B.Switch2_p +
      Ball_and_Plate_MicroLabBox_student_B.sf_MATLABFunction_ko.y;
  }

  /* Switch: '<S22>/Switch3' incorporates:
   *  Constant: '<S5>/CloseLoop_disable'
   */
  if (Ball_and_Plate_MicroLabBox_student_P.CloseLoop_disable_Value >=
      Ball_and_Plate_MicroLabBox_student_P.Switch3_Threshold_m) {
    Ball_and_Plate_MicroLabBox_student_B.Switch3_d =
      Ball_and_Plate_MicroLabBox_student_B.TransferFcn_o;
  } else {
    Ball_and_Plate_MicroLabBox_student_B.Switch3_d =
      Ball_and_Plate_MicroLabBox_student_B.Sum_g;
  }

  /* End of Switch: '<S22>/Switch3' */

  /* Outputs for Atomic SubSystem: '<S22>/Outputs to Amplifier' */

  /* Saturate: '<S43>/Saturation' */
  P3_global_idx_2 = Ball_and_Plate_MicroLabBox_student_B.Switch3_d;
  a21 = Ball_and_Plate_MicroLabBox_student_P.Saturation_LowerSat_f;
  uX_idx_0 = Ball_and_Plate_MicroLabBox_student_P.Saturation_UpperSat_e;
  if (P3_global_idx_2 > uX_idx_0) {
    Ball_and_Plate_MicroLabBox_student_B.Saturation_l = uX_idx_0;
  } else if (P3_global_idx_2 < a21) {
    Ball_and_Plate_MicroLabBox_student_B.Saturation_l = a21;
  } else {
    Ball_and_Plate_MicroLabBox_student_B.Saturation_l = P3_global_idx_2;
  }

  /* End of Saturate: '<S43>/Saturation' */

  /* Gain: '<S43>/Current2V' */
  Ball_and_Plate_MicroLabBox_student_B.Current2V_f =
    Ball_and_Plate_MicroLabBox_student_P.Current2V_Gain_f *
    Ball_and_Plate_MicroLabBox_student_B.Saturation_l;

  /* Gain: '<S43>/DSPscale' */
  Ball_and_Plate_MicroLabBox_student_B.DSPscale_o =
    Ball_and_Plate_MicroLabBox_student_P.DSPscale_Gain_d *
    Ball_and_Plate_MicroLabBox_student_B.Current2V_f;
  if (rtmIsMajorTimeStep(Ball_and_Plate_MicroLabBox_student_M) &&
      Ball_and_Plate_MicroLabBox_student_M->Timing.TaskCounters.TID[1] == 0) {
    /* S-Function (rti_commonblock): '<S49>/S-Function1' */
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

  /* End of Outputs for SubSystem: '<S22>/Outputs to Amplifier' */
  if (rtmIsMajorTimeStep(Ball_and_Plate_MicroLabBox_student_M) &&
      Ball_and_Plate_MicroLabBox_student_M->Timing.TaskCounters.TID[1] == 0) {
    /* MATLAB Function: '<S40>/MATLAB Function' incorporates:
     *  Constant: '<S40>/Constant4'
     */
    Ball_and_Plate_M_MATLABFunction(0.0,
      Ball_and_Plate_MicroLabBox_student_P.path,
      &Ball_and_Plate_MicroLabBox_student_B.sf_MATLABFunction_k,
      &Ball_and_Plate_MicroLabBox_student_DW.sf_MATLABFunction_k);

    /* MATLAB Function: '<S40>/MATLAB Function2' incorporates:
     *  Constant: '<S22>/1_no_0_init_motion'
     *  Constant: '<S22>/Constant4'
     */
    Ball_and_Plate__MATLABFunction2(0.0,
      Ball_and_Plate_MicroLabBox_student_P.u_no_0_init_motion_Value_e,
      Ball_and_Plate_MicroLabBox_student_P.Constant4_Value_b,
      &Ball_and_Plate_MicroLabBox_student_B.sf_MATLABFunction2_m);

    /* Switch: '<S40>/Switch' incorporates:
     *  Constant: '<S22>/1_no_0_init_motion'
     *  Constant: '<S22>/Constant4'
     *  Constant: '<S40>/Constant3'
     */
    if (Ball_and_Plate_MicroLabBox_student_P.u_no_0_init_motion_Value_e >
        Ball_and_Plate_MicroLabBox_student_P.Switch_Threshold_k) {
      Ball_and_Plate_MicroLabBox_student_B.Switch_n =
        Ball_and_Plate_MicroLabBox_student_P.Constant4_Value_b;
    } else {
      Ball_and_Plate_MicroLabBox_student_B.Switch_n =
        Ball_and_Plate_MicroLabBox_student_P.Constant3_Value_i;
    }

    /* End of Switch: '<S40>/Switch' */

    /* Constant: '<S40>/Constant1' */
    Ball_and_Plate_MicroLabBox_student_B.Constant1_c =
      Ball_and_Plate_MicroLabBox_student_P.Constant1_Value_p;

    /* Constant: '<S40>/Constant2' */
    Ball_and_Plate_MicroLabBox_student_B.Constant2_b =
      Ball_and_Plate_MicroLabBox_student_P.Constant2_Value_g;

    /* Switch: '<S23>/Switch1' incorporates:
     *  Constant: '<S23>/Constant3'
     *  Constant: '<S5>/enable_quintic  '
     *  Constant: '<S5>/quintic_ref '
     */
    if (Ball_and_Plate_MicroLabBox_student_P.enable_quintic_Value >
        Ball_and_Plate_MicroLabBox_student_P.Switch1_Threshold_jm) {
      Ball_and_Plate_MicroLabBox_student_B.Switch1_k =
        Ball_and_Plate_MicroLabBox_student_P.Constant3_Value_n;
    } else {
      Ball_and_Plate_MicroLabBox_student_B.Switch1_k =
        Ball_and_Plate_MicroLabBox_student_P.quintic_ref_Value;
    }

    /* End of Switch: '<S23>/Switch1' */

    /* MATLAB Function: '<S23>/MATLAB Function1' incorporates:
     *  Constant: '<S23>/Constant1'
     *  Constant: '<S23>/Constant2'
     */
    Ball_and_Plate__MATLABFunction1
      (Ball_and_Plate_MicroLabBox_student_B.Switch1_k,
       Ball_and_Plate_MicroLabBox_student_P.Constant2_Value_b,
       Ball_and_Plate_MicroLabBox_student_P.Constant1_Value_c,
       &Ball_and_Plate_MicroLabBox_student_B.sf_MATLABFunction1_h,
       &Ball_and_Plate_MicroLabBox_student_DW.sf_MATLABFunction1_h);

    /* Switch: '<S23>/Switch' incorporates:
     *  Constant: '<S5>/enable_ref '
     */
    if (Ball_and_Plate_MicroLabBox_student_P.enable_ref_Value >
        Ball_and_Plate_MicroLabBox_student_P.Switch_Threshold_e) {
      Ball_and_Plate_MicroLabBox_student_B.Switch_jo =
        Ball_and_Plate_MicroLabBox_student_B.sf_MATLABFunction1_h.path;
    } else {
      /* Sum: '<S5>/Add3' incorporates:
       *  Constant: '<S5>/Constant2'
       */
      Ball_and_Plate_MicroLabBox_student_B.Add3 =
        Ball_and_Plate_MicroLabBox_student_B.pos3 -
        Ball_and_Plate_MicroLabBox_student_P.Constant2_Value;
      Ball_and_Plate_MicroLabBox_student_B.Switch_jo =
        Ball_and_Plate_MicroLabBox_student_B.Add3;
    }

    /* End of Switch: '<S23>/Switch' */

    /* Sum: '<S23>/Sum1' */
    Ball_and_Plate_MicroLabBox_student_B.Sum1_h =
      Ball_and_Plate_MicroLabBox_student_B.Switch_jo -
      Ball_and_Plate_MicroLabBox_student_B.mm2m;

    /* Gain: '<S61>/Gain1' */
    Ball_and_Plate_MicroLabBox_student_B.Gain1_mn =
      Ball_and_Plate_MicroLabBox_student_P.Gain1_Gain_p *
      Ball_and_Plate_MicroLabBox_student_B.Sum1_h;

    /* S-Function (dleadlag): '<S61>/Dctleadlag2' */

    /* Level2 S-Function Block: '<S61>/Dctleadlag2' (dleadlag) */
    {
      SimStruct *rts = Ball_and_Plate_MicroLabBox_student_M->childSfunctions[10];
      sfcnOutputs(rts,1);
    }

    /* S-Function (dlowpass1): '<S61>/Dct1lowpass3' */

    /* Level2 S-Function Block: '<S61>/Dct1lowpass3' (dlowpass1) */
    {
      SimStruct *rts = Ball_and_Plate_MicroLabBox_student_M->childSfunctions[11];
      sfcnOutputs(rts,1);
    }

    /* Switch: '<S23>/Switch2' incorporates:
     *  Constant: '<S23>/Constant'
     *  Constant: '<S5>/controller_disable'
     */
    if (Ball_and_Plate_MicroLabBox_student_P.controller_disable_Value >=
        Ball_and_Plate_MicroLabBox_student_P.Switch2_Threshold_b) {
      Ball_and_Plate_MicroLabBox_student_B.Switch2_h =
        Ball_and_Plate_MicroLabBox_student_P.Constant_Value_e;
    } else {
      Ball_and_Plate_MicroLabBox_student_B.Switch2_h =
        Ball_and_Plate_MicroLabBox_student_B.Dct1lowpass3_e;
    }

    /* End of Switch: '<S23>/Switch2' */

    /* MATLAB Function: '<S58>/MATLAB Function' incorporates:
     *  Constant: '<S23>/Enable_ID_C '
     *  Constant: '<S58>/Constant'
     */
    Ball_and_Plate_MATLABFunction_e
      (Ball_and_Plate_MicroLabBox_student_P.Enable_ID_C_Value,
       Ball_and_Plate_MicroLabBox_student_P.uC,
       &Ball_and_Plate_MicroLabBox_student_B.sf_MATLABFunction_hg,
       &Ball_and_Plate_MicroLabBox_student_DW.sf_MATLABFunction_hg);

    /* Sum: '<S23>/Sum' */
    Ball_and_Plate_MicroLabBox_student_B.Sum_h =
      Ball_and_Plate_MicroLabBox_student_B.Switch2_h +
      Ball_and_Plate_MicroLabBox_student_B.sf_MATLABFunction_hg.y;
  }

  /* Switch: '<S23>/Switch3' incorporates:
   *  Constant: '<S5>/CloseLoop_disable'
   */
  if (Ball_and_Plate_MicroLabBox_student_P.CloseLoop_disable_Value >=
      Ball_and_Plate_MicroLabBox_student_P.Switch3_Threshold_g) {
    Ball_and_Plate_MicroLabBox_student_B.Switch3_l =
      Ball_and_Plate_MicroLabBox_student_B.TransferFcn;
  } else {
    Ball_and_Plate_MicroLabBox_student_B.Switch3_l =
      Ball_and_Plate_MicroLabBox_student_B.Sum_h;
  }

  /* End of Switch: '<S23>/Switch3' */

  /* Outputs for Atomic SubSystem: '<S23>/Outputs to Amplifier' */

  /* Saturate: '<S59>/Saturation' */
  P3_global_idx_2 = Ball_and_Plate_MicroLabBox_student_B.Switch3_l;
  a21 = Ball_and_Plate_MicroLabBox_student_P.Saturation_LowerSat_p;
  uX_idx_0 = Ball_and_Plate_MicroLabBox_student_P.Saturation_UpperSat_h;
  if (P3_global_idx_2 > uX_idx_0) {
    Ball_and_Plate_MicroLabBox_student_B.Saturation_i = uX_idx_0;
  } else if (P3_global_idx_2 < a21) {
    Ball_and_Plate_MicroLabBox_student_B.Saturation_i = a21;
  } else {
    Ball_and_Plate_MicroLabBox_student_B.Saturation_i = P3_global_idx_2;
  }

  /* End of Saturate: '<S59>/Saturation' */

  /* Gain: '<S59>/Current2V' */
  Ball_and_Plate_MicroLabBox_student_B.Current2V =
    Ball_and_Plate_MicroLabBox_student_P.Current2V_Gain_p *
    Ball_and_Plate_MicroLabBox_student_B.Saturation_i;

  /* Gain: '<S59>/DSPscale' */
  Ball_and_Plate_MicroLabBox_student_B.DSPscale =
    Ball_and_Plate_MicroLabBox_student_P.DSPscale_Gain_h *
    Ball_and_Plate_MicroLabBox_student_B.Current2V;
  if (rtmIsMajorTimeStep(Ball_and_Plate_MicroLabBox_student_M) &&
      Ball_and_Plate_MicroLabBox_student_M->Timing.TaskCounters.TID[1] == 0) {
    /* S-Function (rti_commonblock): '<S66>/S-Function1' */
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

  /* End of Outputs for SubSystem: '<S23>/Outputs to Amplifier' */
  if (rtmIsMajorTimeStep(Ball_and_Plate_MicroLabBox_student_M) &&
      Ball_and_Plate_MicroLabBox_student_M->Timing.TaskCounters.TID[1] == 0) {
    /* MATLAB Function: '<S55>/MATLAB Function' incorporates:
     *  Constant: '<S55>/Constant4'
     */
    Ball_and_Plate_M_MATLABFunction(0.0,
      Ball_and_Plate_MicroLabBox_student_P.path,
      &Ball_and_Plate_MicroLabBox_student_B.sf_MATLABFunction_h,
      &Ball_and_Plate_MicroLabBox_student_DW.sf_MATLABFunction_h);

    /* MATLAB Function: '<S55>/MATLAB Function2' incorporates:
     *  Constant: '<S23>/1_no_0_init_motion'
     *  Constant: '<S23>/Constant4'
     */
    Ball_and_Plate__MATLABFunction2(0.0,
      Ball_and_Plate_MicroLabBox_student_P.u_no_0_init_motion_Value_n,
      Ball_and_Plate_MicroLabBox_student_P.Constant4_Value_c,
      &Ball_and_Plate_MicroLabBox_student_B.sf_MATLABFunction2_e);

    /* Switch: '<S55>/Switch' incorporates:
     *  Constant: '<S23>/1_no_0_init_motion'
     *  Constant: '<S23>/Constant4'
     *  Constant: '<S55>/Constant3'
     */
    if (Ball_and_Plate_MicroLabBox_student_P.u_no_0_init_motion_Value_n >
        Ball_and_Plate_MicroLabBox_student_P.Switch_Threshold_l) {
      Ball_and_Plate_MicroLabBox_student_B.Switch_e =
        Ball_and_Plate_MicroLabBox_student_P.Constant4_Value_c;
    } else {
      Ball_and_Plate_MicroLabBox_student_B.Switch_e =
        Ball_and_Plate_MicroLabBox_student_P.Constant3_Value_p;
    }

    /* End of Switch: '<S55>/Switch' */

    /* Constant: '<S55>/Constant1' */
    Ball_and_Plate_MicroLabBox_student_B.Constant1_h =
      Ball_and_Plate_MicroLabBox_student_P.Constant1_Value_f;

    /* Constant: '<S55>/Constant2' */
    Ball_and_Plate_MicroLabBox_student_B.Constant2_h =
      Ball_and_Plate_MicroLabBox_student_P.Constant2_Value_i;

    /* MATLAB Function: '<S57>/MATLAB Function' incorporates:
     *  Constant: '<S23>/1_to_enable_id'
     *  Constant: '<S57>/Constant'
     */
    Ball_and_Plate_MATLABFunction_e
      (Ball_and_Plate_MicroLabBox_student_P.u_to_enable_id_Value,
       Ball_and_Plate_MicroLabBox_student_P.uA,
       &Ball_and_Plate_MicroLabBox_student_B.sf_MATLABFunction_g,
       &Ball_and_Plate_MicroLabBox_student_DW.sf_MATLABFunction_g);

    /* Constant: '<S5>/reser_integrator' */
    Ball_and_Plate_MicroLabBox_student_B.reser_integrator =
      Ball_and_Plate_MicroLabBox_student_P.reser_integrator_Value;

    /* DataTypeConversion: '<S72>/Data Type Conversion14' incorporates:
     *  Constant: '<S8>/p_zero'
     */
    Ball_and_Plate_MicroLabBox_student_B.DataTypeConversion14 =
      Ball_and_Plate_MicroLabBox_student_P.p_zero_Value;

    /* DataTypeConversion: '<S72>/Data Type Conversion15' incorporates:
     *  Constant: '<S8>/m_zero'
     */
    Ball_and_Plate_MicroLabBox_student_B.DataTypeConversion15 =
      Ball_and_Plate_MicroLabBox_student_P.m_zero_Value;
  }

  if (rtmIsMajorTimeStep(Ball_and_Plate_MicroLabBox_student_M) &&
      Ball_and_Plate_MicroLabBox_student_M->Timing.TaskCounters.TID[2] == 0) {
  }

  if (rtmIsMajorTimeStep(Ball_and_Plate_MicroLabBox_student_M) &&
      Ball_and_Plate_MicroLabBox_student_M->Timing.TaskCounters.TID[1] == 0) {
  }

  if (rtmIsMajorTimeStep(Ball_and_Plate_MicroLabBox_student_M) &&
      Ball_and_Plate_MicroLabBox_student_M->Timing.TaskCounters.TID[2] == 0) {
  }

  if (rtmIsMajorTimeStep(Ball_and_Plate_MicroLabBox_student_M) &&
      Ball_and_Plate_MicroLabBox_student_M->Timing.TaskCounters.TID[1] == 0) {
  }

  if (rtmIsMajorTimeStep(Ball_and_Plate_MicroLabBox_student_M) &&
      Ball_and_Plate_MicroLabBox_student_M->Timing.TaskCounters.TID[2] == 0) {
  }

  if (rtmIsMajorTimeStep(Ball_and_Plate_MicroLabBox_student_M) &&
      Ball_and_Plate_MicroLabBox_student_M->Timing.TaskCounters.TID[1] == 0) {
  }

  if (rtmIsMajorTimeStep(Ball_and_Plate_MicroLabBox_student_M) &&
      Ball_and_Plate_MicroLabBox_student_M->Timing.TaskCounters.TID[2] == 0) {
    /* Memory: '<S72>/last_x' */
    Ball_and_Plate_MicroLabBox_student_B.last_x[0] =
      Ball_and_Plate_MicroLabBox_student_DW.last_x_PreviousInput[0];
    Ball_and_Plate_MicroLabBox_student_B.last_x[1] =
      Ball_and_Plate_MicroLabBox_student_DW.last_x_PreviousInput[1];
    Ball_and_Plate_MicroLabBox_student_B.last_x[2] =
      Ball_and_Plate_MicroLabBox_student_DW.last_x_PreviousInput[2];
    Ball_and_Plate_MicroLabBox_student_B.last_x[3] =
      Ball_and_Plate_MicroLabBox_student_DW.last_x_PreviousInput[3];

    /* Gain: '<S72>/umin_scale3' */
    for (r1 = 0; r1 < 52; r1++) {
      Ball_and_Plate_MicroLabBox_student_B.umin_scale3[r1] =
        Ball_and_Plate_MicroLabBox_student_P.umin_scale3_Gain[r1] *
        Ball_and_Plate_MicroLabBox_student_B.useq[r1];
    }

    /* End of Gain: '<S72>/umin_scale3' */

    /* Gain: '<S72>/umin_scale5' */
    for (r1 = 0; r1 < 104; r1++) {
      Ball_and_Plate_MicroLabBox_student_B.umin_scale5[r1] =
        Ball_and_Plate_MicroLabBox_student_P.umin_scale5_Gain[r1] *
        Ball_and_Plate_MicroLabBox_student_B.yseq[r1];
    }

    /* End of Gain: '<S72>/umin_scale5' */

    /* Gain: '<S72>/umin_scale2' incorporates:
     *  Constant: '<S72>/constant'
     */
    Ball_and_Plate_MicroLabBox_student_B.umin_scale2[0] =
      Ball_and_Plate_MicroLabBox_student_P.umin_scale2_Gain[0] *
      Ball_and_Plate_MicroLabBox_student_P.constant_Value[0];
    Ball_and_Plate_MicroLabBox_student_B.umin_scale2[1] =
      Ball_and_Plate_MicroLabBox_student_P.umin_scale2_Gain[1] *
      Ball_and_Plate_MicroLabBox_student_P.constant_Value[1];
  }

  /* RateTransition: '<Root>/Rate Transition4' incorporates:
   *  RateTransition: '<Root>/Rate Transition5'
   */
  if (rtmIsMajorTimeStep(Ball_and_Plate_MicroLabBox_student_M) &&
      Ball_and_Plate_MicroLabBox_student_M->Timing.TaskCounters.TID[1] == 0) {
    if (Ball_and_Plate_MicroLabBox_student_DW.RateTransition4_semaphoreTaken ==
        0) {
      Ball_and_Plate_MicroLabBox_student_DW.RateTransition4_Buffer0 =
        Ball_and_Plate_MicroLabBox_student_B.Gain2;
    }

    if (Ball_and_Plate_MicroLabBox_student_DW.RateTransition5_semaphoreTaken ==
        0) {
      Ball_and_Plate_MicroLabBox_student_DW.RateTransition5_Buffer0 =
        Ball_and_Plate_MicroLabBox_student_B.Gain3;
    }
  }

  if (rtmIsMajorTimeStep(Ball_and_Plate_MicroLabBox_student_M) &&
      Ball_and_Plate_MicroLabBox_student_M->Timing.TaskCounters.TID[2] == 0) {
    Ball_and_Plate_MicroLabBox_student_DW.RateTransition4_semaphoreTaken = 1;
    Ball_and_Plate_MicroLabBox_student_B.RateTransition4 =
      Ball_and_Plate_MicroLabBox_student_DW.RateTransition4_Buffer0;
    Ball_and_Plate_MicroLabBox_student_DW.RateTransition4_semaphoreTaken = 0;
    Ball_and_Plate_MicroLabBox_student_DW.RateTransition5_semaphoreTaken = 1;
    Ball_and_Plate_MicroLabBox_student_B.RateTransition5 =
      Ball_and_Plate_MicroLabBox_student_DW.RateTransition5_Buffer0;
    Ball_and_Plate_MicroLabBox_student_DW.RateTransition5_semaphoreTaken = 0;
  }

  /* End of RateTransition: '<Root>/Rate Transition4' */

  /* Sin: '<Root>/Alpha_sine ' */
  Ball_and_Plate_MicroLabBox_student_B.Alpha_sine = sin
    (Ball_and_Plate_MicroLabBox_student_P.Alpha_sine_Freq *
     Ball_and_Plate_MicroLabBox_student_M->Timing.t[0] +
     Ball_and_Plate_MicroLabBox_student_P.Alpha_sine_Phase) *
    Ball_and_Plate_MicroLabBox_student_P.Alpha_sine_Amp +
    Ball_and_Plate_MicroLabBox_student_P.Alpha_sine_Bias;

  /* Sin: '<Root>/Beta_sine' */
  Ball_and_Plate_MicroLabBox_student_B.Beta_sine = sin
    (Ball_and_Plate_MicroLabBox_student_P.Beta_sine_Freq *
     Ball_and_Plate_MicroLabBox_student_M->Timing.t[0] +
     Ball_and_Plate_MicroLabBox_student_P.Beta_sine_Phase) *
    Ball_and_Plate_MicroLabBox_student_P.Beta_sine_Amp +
    Ball_and_Plate_MicroLabBox_student_P.Beta_sine_Bias;
  if (rtmIsMajorTimeStep(Ball_and_Plate_MicroLabBox_student_M) &&
      Ball_and_Plate_MicroLabBox_student_M->Timing.TaskCounters.TID[1] == 0) {
    /* Constant: '<Root>/Constant5' */
    Ball_and_Plate_MicroLabBox_student_B.Constant5 =
      Ball_and_Plate_MicroLabBox_student_P.Ts_Outer - 0.004;

    /* Constant: '<Root>/Constant6' */
    Ball_and_Plate_MicroLabBox_student_B.Constant6 =
      Ball_and_Plate_MicroLabBox_student_P.Ts_fast;
  }
}

/* Model update function */
void Ball_and_Plate_MicroLabBox_student_update(void)
{
  boolean_T resetCoeff;
  real_T tol;
  real_T err;
  boolean_T guard1 = false;
  if (rtmIsMajorTimeStep(Ball_and_Plate_MicroLabBox_student_M) &&
      Ball_and_Plate_MicroLabBox_student_M->Timing.TaskCounters.TID[2] == 0) {
    /* Update for UnitDelay: '<S1>/UD' */
    Ball_and_Plate_MicroLabBox_student_DW.UD_DSTATE =
      Ball_and_Plate_MicroLabBox_student_B.RateTransition3;

    /* Update for FirstOrderHold: '<Root>/First Order Hold' */
    resetCoeff = (Ball_and_Plate_MicroLabBox_student_DW.Tk == (rtInf));
    guard1 = false;
    if (!resetCoeff) {
      if ((Ball_and_Plate_MicroLabBox_student_B.RateTransition4 >= -1.0) &&
          (Ball_and_Plate_MicroLabBox_student_B.RateTransition4 <= 1.0)) {
        tol = Ball_and_Plate_MicroLabBox_student_P.FirstOrderHold_ErrTol;
      } else if (Ball_and_Plate_MicroLabBox_student_B.RateTransition4 > 1.0) {
        tol = Ball_and_Plate_MicroLabBox_student_B.RateTransition4 *
          Ball_and_Plate_MicroLabBox_student_P.FirstOrderHold_ErrTol;
      } else {
        tol = -(Ball_and_Plate_MicroLabBox_student_B.RateTransition4 *
                Ball_and_Plate_MicroLabBox_student_P.FirstOrderHold_ErrTol);
      }

      err = Ball_and_Plate_MicroLabBox_student_B.FirstOrderHold -
        Ball_and_Plate_MicroLabBox_student_B.RateTransition4;
      if ((err > tol) || (err < -tol)) {
        guard1 = true;
      } else {
        tol = Ball_and_Plate_MicroLabBox_student_M->Timing.t[0] -
          Ball_and_Plate_MicroLabBox_student_DW.Tk;
        Ball_and_Plate_MicroLabBox_student_DW.Mk =
          (Ball_and_Plate_MicroLabBox_student_B.RateTransition4 -
           Ball_and_Plate_MicroLabBox_student_DW.Uk) / tol;
        Ball_and_Plate_MicroLabBox_student_DW.Ck =
          Ball_and_Plate_MicroLabBox_student_B.FirstOrderHold;
      }
    } else {
      guard1 = true;
    }

    if (guard1) {
      if (Ball_and_Plate_MicroLabBox_student_B.RateTransition4 !=
          Ball_and_Plate_MicroLabBox_student_B.FirstOrderHold) {
        rtsiSetBlockStateForSolverChangedAtMajorStep
          (&Ball_and_Plate_MicroLabBox_student_M->solverInfo, true);
        rtsiSetContTimeOutputInconsistentWithStateAtMajorStep
          (&Ball_and_Plate_MicroLabBox_student_M->solverInfo, true);
      }

      Ball_and_Plate_MicroLabBox_student_DW.Ck =
        Ball_and_Plate_MicroLabBox_student_B.RateTransition4;
      Ball_and_Plate_MicroLabBox_student_DW.Mk = 0.0;
    }

    Ball_and_Plate_MicroLabBox_student_DW.Uk =
      Ball_and_Plate_MicroLabBox_student_B.RateTransition4;
    Ball_and_Plate_MicroLabBox_student_DW.Tk =
      Ball_and_Plate_MicroLabBox_student_M->Timing.t[0];

    /* End of Update for FirstOrderHold: '<Root>/First Order Hold' */

    /* Update for FirstOrderHold: '<Root>/First Order Hold1' */
    resetCoeff = (Ball_and_Plate_MicroLabBox_student_DW.Tk_c == (rtInf));
    guard1 = false;
    if (!resetCoeff) {
      if ((Ball_and_Plate_MicroLabBox_student_B.RateTransition5 >= -1.0) &&
          (Ball_and_Plate_MicroLabBox_student_B.RateTransition5 <= 1.0)) {
        tol = Ball_and_Plate_MicroLabBox_student_P.FirstOrderHold1_ErrTol;
      } else if (Ball_and_Plate_MicroLabBox_student_B.RateTransition5 > 1.0) {
        tol = Ball_and_Plate_MicroLabBox_student_B.RateTransition5 *
          Ball_and_Plate_MicroLabBox_student_P.FirstOrderHold1_ErrTol;
      } else {
        tol = -(Ball_and_Plate_MicroLabBox_student_B.RateTransition5 *
                Ball_and_Plate_MicroLabBox_student_P.FirstOrderHold1_ErrTol);
      }

      err = Ball_and_Plate_MicroLabBox_student_B.FirstOrderHold1 -
        Ball_and_Plate_MicroLabBox_student_B.RateTransition5;
      if ((err > tol) || (err < -tol)) {
        guard1 = true;
      } else {
        tol = Ball_and_Plate_MicroLabBox_student_M->Timing.t[0] -
          Ball_and_Plate_MicroLabBox_student_DW.Tk_c;
        Ball_and_Plate_MicroLabBox_student_DW.Mk_g =
          (Ball_and_Plate_MicroLabBox_student_B.RateTransition5 -
           Ball_and_Plate_MicroLabBox_student_DW.Uk_i) / tol;
        Ball_and_Plate_MicroLabBox_student_DW.Ck_h =
          Ball_and_Plate_MicroLabBox_student_B.FirstOrderHold1;
      }
    } else {
      guard1 = true;
    }

    if (guard1) {
      if (Ball_and_Plate_MicroLabBox_student_B.RateTransition5 !=
          Ball_and_Plate_MicroLabBox_student_B.FirstOrderHold1) {
        rtsiSetBlockStateForSolverChangedAtMajorStep
          (&Ball_and_Plate_MicroLabBox_student_M->solverInfo, true);
        rtsiSetContTimeOutputInconsistentWithStateAtMajorStep
          (&Ball_and_Plate_MicroLabBox_student_M->solverInfo, true);
      }

      Ball_and_Plate_MicroLabBox_student_DW.Ck_h =
        Ball_and_Plate_MicroLabBox_student_B.RateTransition5;
      Ball_and_Plate_MicroLabBox_student_DW.Mk_g = 0.0;
    }

    Ball_and_Plate_MicroLabBox_student_DW.Uk_i =
      Ball_and_Plate_MicroLabBox_student_B.RateTransition5;
    Ball_and_Plate_MicroLabBox_student_DW.Tk_c =
      Ball_and_Plate_MicroLabBox_student_M->Timing.t[0];

    /* End of Update for FirstOrderHold: '<Root>/First Order Hold1' */
  }

  if (rtmIsMajorTimeStep(Ball_and_Plate_MicroLabBox_student_M) &&
      Ball_and_Plate_MicroLabBox_student_M->Timing.TaskCounters.TID[2] == 0) {
    /* Update for UnitDelay: '<S72>/last_mv' */
    Ball_and_Plate_MicroLabBox_student_DW.last_mv_DSTATE[0] =
      Ball_and_Plate_MicroLabBox_student_B.u[0];
    Ball_and_Plate_MicroLabBox_student_DW.last_mv_DSTATE[1] =
      Ball_and_Plate_MicroLabBox_student_B.u[1];

    /* Update for Memory: '<S72>/Memory' */
    memcpy(&Ball_and_Plate_MicroLabBox_student_DW.Memory_PreviousInput[0],
           &Ball_and_Plate_MicroLabBox_student_B.iAout[0], 212U * sizeof
           (boolean_T));

    /* Update for Memory: '<S72>/last_x' */
    Ball_and_Plate_MicroLabBox_student_DW.last_x_PreviousInput[0] =
      Ball_and_Plate_MicroLabBox_student_B.xk1[0];
    Ball_and_Plate_MicroLabBox_student_DW.last_x_PreviousInput[1] =
      Ball_and_Plate_MicroLabBox_student_B.xk1[1];
    Ball_and_Plate_MicroLabBox_student_DW.last_x_PreviousInput[2] =
      Ball_and_Plate_MicroLabBox_student_B.xk1[2];
    Ball_and_Plate_MicroLabBox_student_DW.last_x_PreviousInput[3] =
      Ball_and_Plate_MicroLabBox_student_B.xk1[3];
  }

  if (rtmIsMajorTimeStep(Ball_and_Plate_MicroLabBox_student_M) &&
      Ball_and_Plate_MicroLabBox_student_M->Timing.TaskCounters.TID[1] == 0) {
    /* Update for UnitDelay: '<S2>/UD' */
    Ball_and_Plate_MicroLabBox_student_DW.UD_DSTATE_c =
      Ball_and_Plate_MicroLabBox_student_B.TSamp;

    /* Update for UnitDelay: '<S3>/UD' */
    Ball_and_Plate_MicroLabBox_student_DW.UD_DSTATE_o =
      Ball_and_Plate_MicroLabBox_student_B.TSamp_p;
  }

  /* ContTimeOutputInconsistentWithStateAtMajorOutputFlag is set, need to run a minor output */
  if (rtmIsMajorTimeStep(Ball_and_Plate_MicroLabBox_student_M)) {
    if (rtsiGetContTimeOutputInconsistentWithStateAtMajorStep
        (&Ball_and_Plate_MicroLabBox_student_M->solverInfo)) {
      rtsiSetSimTimeStep(&Ball_and_Plate_MicroLabBox_student_M->solverInfo,
                         MINOR_TIME_STEP);
      rtsiSetContTimeOutputInconsistentWithStateAtMajorStep
        (&Ball_and_Plate_MicroLabBox_student_M->solverInfo, false);
      Ball_and_Plate_MicroLabBox_student_output();
      rtsiSetSimTimeStep(&Ball_and_Plate_MicroLabBox_student_M->solverInfo,
                         MAJOR_TIME_STEP);
    }
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

  /* Derivatives for Atomic SubSystem: '<S21>/Position Measurement' */
  /* Derivatives for TransferFcn: '<S37>/Transfer Fcn' */
  _rtXdot->TransferFcn_CSTATE_l = 0.0;
  _rtXdot->TransferFcn_CSTATE_l +=
    Ball_and_Plate_MicroLabBox_student_P.TransferFcn_A *
    Ball_and_Plate_MicroLabBox_student_X.TransferFcn_CSTATE_l;
  _rtXdot->TransferFcn_CSTATE_l +=
    Ball_and_Plate_MicroLabBox_student_B.EnabledSubsystem.OutportBufferForOut1;

  /* End of Derivatives for SubSystem: '<S21>/Position Measurement' */

  /* Derivatives for Atomic SubSystem: '<S22>/Position Measurement' */
  /* Derivatives for TransferFcn: '<S52>/Transfer Fcn' */
  _rtXdot->TransferFcn_CSTATE_h = 0.0;
  _rtXdot->TransferFcn_CSTATE_h +=
    Ball_and_Plate_MicroLabBox_student_P.TransferFcn_A_g *
    Ball_and_Plate_MicroLabBox_student_X.TransferFcn_CSTATE_h;
  _rtXdot->TransferFcn_CSTATE_h +=
    Ball_and_Plate_MicroLabBox_student_B.EnabledSubsystem_a.OutportBufferForOut1;

  /* End of Derivatives for SubSystem: '<S22>/Position Measurement' */

  /* Derivatives for Atomic SubSystem: '<S23>/Position Measurement' */
  /* Derivatives for TransferFcn: '<S69>/Transfer Fcn' */
  _rtXdot->TransferFcn_CSTATE = 0.0;
  _rtXdot->TransferFcn_CSTATE +=
    Ball_and_Plate_MicroLabBox_student_P.TransferFcn_A_m *
    Ball_and_Plate_MicroLabBox_student_X.TransferFcn_CSTATE;
  _rtXdot->TransferFcn_CSTATE +=
    Ball_and_Plate_MicroLabBox_student_B.EnabledSubsystem_n.OutportBufferForOut1;

  /* End of Derivatives for SubSystem: '<S23>/Position Measurement' */
}

/* Model initialize function */
void Ball_and_Plate_MicroLabBox_student_initialize(void)
{
  /* Registration code */

  /* initialize non-finites */
  rt_InitInfAndNaN(sizeof(real_T));

  /* non-finite (run-time) assignments */
  Ball_and_Plate_MicroLabBox_student_P.FirstOrderHold_ErrTol = rtInf;
  Ball_and_Plate_MicroLabBox_student_P.FirstOrderHold1_ErrTol = rtInf;

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
    Ball_and_Plate_MicroLabBox_student_M->Timing.sampleTimes[2] = (0.02);

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

  Ball_and_Plate_MicroLabBox_student_M->Sizes.numSFcns = (12);

  /* register each child */
  {
    (void) memset((void *)
                  &Ball_and_Plate_MicroLabBox_student_M->NonInlinedSFcns.childSFunctions
                  [0], 0,
                  12*sizeof(SimStruct));
    Ball_and_Plate_MicroLabBox_student_M->childSfunctions =
      (&Ball_and_Plate_MicroLabBox_student_M->
       NonInlinedSFcns.childSFunctionPtrs[0]);

    {
      int_T i;
      for (i = 0; i < 12; i++) {
        Ball_and_Plate_MicroLabBox_student_M->childSfunctions[i] =
          (&Ball_and_Plate_MicroLabBox_student_M->NonInlinedSFcns.childSFunctions
           [i]);
      }
    }

    /* Level2 S-Function Block: Ball_and_Plate_MicroLabBox_student/<S10>/Dct1lowpass2 (dlowpass1) */
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
            &Ball_and_Plate_MicroLabBox_student_B.Dct1lowpass2));
        }
      }

      /* path info */
      ssSetModelName(rts, "Dct1lowpass2");
      ssSetPath(rts, "Ball_and_Plate_MicroLabBox_student/lowpass /Dct1lowpass2");
      ssSetRTModel(rts,Ball_and_Plate_MicroLabBox_student_M);
      ssSetParentSS(rts, (NULL));
      ssSetRootSS(rts, rts);
      ssSetVersion(rts, SIMSTRUCT_VERSION_LEVEL2);

      /* parameters */
      {
        mxArray **sfcnParams = (mxArray **)
          &Ball_and_Plate_MicroLabBox_student_M->NonInlinedSFcns.Sfcn0.params;
        ssSetSFcnParamsCount(rts, 2);
        ssSetSFcnParamsPtr(rts, &sfcnParams[0]);
        ssSetSFcnParam(rts, 0, (mxArray*)
                       Ball_and_Plate_MicroLabBox_student_P.Dct1lowpass2_P1_Size);
        ssSetSFcnParam(rts, 1, (mxArray*)
                       Ball_and_Plate_MicroLabBox_student_P.Dct1lowpass2_P2_Size);
      }

      /* work vectors */
      ssSetRWork(rts, (real_T *)
                 &Ball_and_Plate_MicroLabBox_student_DW.Dct1lowpass2_RWORK[0]);

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
                   &Ball_and_Plate_MicroLabBox_student_DW.Dct1lowpass2_RWORK[0]);
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

    /* Level2 S-Function Block: Ball_and_Plate_MicroLabBox_student/<S11>/Dct1lowpass2 (dlowpass1) */
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
          sfcnUPtrs[0] = &Ball_and_Plate_MicroLabBox_student_B.Gain1_m;
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
            &Ball_and_Plate_MicroLabBox_student_B.Dct1lowpass2_b));
        }
      }

      /* path info */
      ssSetModelName(rts, "Dct1lowpass2");
      ssSetPath(rts, "Ball_and_Plate_MicroLabBox_student/lowpass 1/Dct1lowpass2");
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
                       Ball_and_Plate_MicroLabBox_student_P.Dct1lowpass2_P1_Size_b);
        ssSetSFcnParam(rts, 1, (mxArray*)
                       Ball_and_Plate_MicroLabBox_student_P.Dct1lowpass2_P2_Size_n);
      }

      /* work vectors */
      ssSetRWork(rts, (real_T *)
                 &Ball_and_Plate_MicroLabBox_student_DW.Dct1lowpass2_RWORK_p[0]);

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
                   &Ball_and_Plate_MicroLabBox_student_DW.Dct1lowpass2_RWORK_p[0]);
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

    /* Level2 S-Function Block: Ball_and_Plate_MicroLabBox_student/<S14>/Dct1lowpass2 (dlowpass1) */
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
          sfcnUPtrs[0] = &Ball_and_Plate_MicroLabBox_student_B.Gain1_b;
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
            &Ball_and_Plate_MicroLabBox_student_B.Dct1lowpass2_l));
        }
      }

      /* path info */
      ssSetModelName(rts, "Dct1lowpass2");
      ssSetPath(rts, "Ball_and_Plate_MicroLabBox_student/lowpass 4/Dct1lowpass2");
      ssSetRTModel(rts,Ball_and_Plate_MicroLabBox_student_M);
      ssSetParentSS(rts, (NULL));
      ssSetRootSS(rts, rts);
      ssSetVersion(rts, SIMSTRUCT_VERSION_LEVEL2);

      /* parameters */
      {
        mxArray **sfcnParams = (mxArray **)
          &Ball_and_Plate_MicroLabBox_student_M->NonInlinedSFcns.Sfcn2.params;
        ssSetSFcnParamsCount(rts, 2);
        ssSetSFcnParamsPtr(rts, &sfcnParams[0]);
        ssSetSFcnParam(rts, 0, (mxArray*)
                       Ball_and_Plate_MicroLabBox_student_P.Dct1lowpass2_P1_Size_l);
        ssSetSFcnParam(rts, 1, (mxArray*)
                       Ball_and_Plate_MicroLabBox_student_P.Dct1lowpass2_P2_Size_na);
      }

      /* work vectors */
      ssSetRWork(rts, (real_T *)
                 &Ball_and_Plate_MicroLabBox_student_DW.Dct1lowpass2_RWORK_n[0]);

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
                   &Ball_and_Plate_MicroLabBox_student_DW.Dct1lowpass2_RWORK_n[0]);
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

    /* Level2 S-Function Block: Ball_and_Plate_MicroLabBox_student/<S15>/Dct1lowpass2 (dlowpass1) */
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
          sfcnUPtrs[0] = &Ball_and_Plate_MicroLabBox_student_B.Gain1_bh;
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
            &Ball_and_Plate_MicroLabBox_student_B.Dct1lowpass2_o));
        }
      }

      /* path info */
      ssSetModelName(rts, "Dct1lowpass2");
      ssSetPath(rts, "Ball_and_Plate_MicroLabBox_student/lowpass 5/Dct1lowpass2");
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
                       Ball_and_Plate_MicroLabBox_student_P.Dct1lowpass2_P1_Size_o);
        ssSetSFcnParam(rts, 1, (mxArray*)
                       Ball_and_Plate_MicroLabBox_student_P.Dct1lowpass2_P2_Size_k);
      }

      /* work vectors */
      ssSetRWork(rts, (real_T *)
                 &Ball_and_Plate_MicroLabBox_student_DW.Dct1lowpass2_RWORK_g[0]);

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
                   &Ball_and_Plate_MicroLabBox_student_DW.Dct1lowpass2_RWORK_g[0]);
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

    /* Level2 S-Function Block: Ball_and_Plate_MicroLabBox_student/<S12>/Dct1lowpass2 (dlowpass1) */
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
          sfcnUPtrs[0] = &Ball_and_Plate_MicroLabBox_student_B.Gain1_c;
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
            &Ball_and_Plate_MicroLabBox_student_B.Dct1lowpass2_b2));
        }
      }

      /* path info */
      ssSetModelName(rts, "Dct1lowpass2");
      ssSetPath(rts, "Ball_and_Plate_MicroLabBox_student/lowpass 2/Dct1lowpass2");
      ssSetRTModel(rts,Ball_and_Plate_MicroLabBox_student_M);
      ssSetParentSS(rts, (NULL));
      ssSetRootSS(rts, rts);
      ssSetVersion(rts, SIMSTRUCT_VERSION_LEVEL2);

      /* parameters */
      {
        mxArray **sfcnParams = (mxArray **)
          &Ball_and_Plate_MicroLabBox_student_M->NonInlinedSFcns.Sfcn4.params;
        ssSetSFcnParamsCount(rts, 2);
        ssSetSFcnParamsPtr(rts, &sfcnParams[0]);
        ssSetSFcnParam(rts, 0, (mxArray*)
                       Ball_and_Plate_MicroLabBox_student_P.Dct1lowpass2_P1_Size_j);
        ssSetSFcnParam(rts, 1, (mxArray*)
                       Ball_and_Plate_MicroLabBox_student_P.Dct1lowpass2_P2_Size_g);
      }

      /* work vectors */
      ssSetRWork(rts, (real_T *)
                 &Ball_and_Plate_MicroLabBox_student_DW.Dct1lowpass2_RWORK_gm[0]);

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
                   &Ball_and_Plate_MicroLabBox_student_DW.Dct1lowpass2_RWORK_gm
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

    /* Level2 S-Function Block: Ball_and_Plate_MicroLabBox_student/<S13>/Dct1lowpass2 (dlowpass1) */
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
          sfcnUPtrs[0] = &Ball_and_Plate_MicroLabBox_student_B.Gain1_d;
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
            &Ball_and_Plate_MicroLabBox_student_B.Dct1lowpass2_g));
        }
      }

      /* path info */
      ssSetModelName(rts, "Dct1lowpass2");
      ssSetPath(rts, "Ball_and_Plate_MicroLabBox_student/lowpass 3/Dct1lowpass2");
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
                       Ball_and_Plate_MicroLabBox_student_P.Dct1lowpass2_P1_Size_p);
        ssSetSFcnParam(rts, 1, (mxArray*)
                       Ball_and_Plate_MicroLabBox_student_P.Dct1lowpass2_P2_Size_c);
      }

      /* work vectors */
      ssSetRWork(rts, (real_T *)
                 &Ball_and_Plate_MicroLabBox_student_DW.Dct1lowpass2_RWORK_gt[0]);

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
                   &Ball_and_Plate_MicroLabBox_student_DW.Dct1lowpass2_RWORK_gt
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

    /* Level2 S-Function Block: Ball_and_Plate_MicroLabBox_student/<S30>/Dctleadlag2 (dleadlag) */
    {
      SimStruct *rts = Ball_and_Plate_MicroLabBox_student_M->childSfunctions[6];

      /* timing info */
      time_T *sfcnPeriod =
        Ball_and_Plate_MicroLabBox_student_M->NonInlinedSFcns.Sfcn6.sfcnPeriod;
      time_T *sfcnOffset =
        Ball_and_Plate_MicroLabBox_student_M->NonInlinedSFcns.Sfcn6.sfcnOffset;
      int_T *sfcnTsMap =
        Ball_and_Plate_MicroLabBox_student_M->NonInlinedSFcns.Sfcn6.sfcnTsMap;
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
                         [6]);
      }

      _ssSetBlkInfo2PortInfo2Ptr(rts,
        &Ball_and_Plate_MicroLabBox_student_M->NonInlinedSFcns.inputOutputPortInfo2
        [6]);

      /* Set up the mdlInfo pointer */
      ssSetRTWSfcnInfo(rts, Ball_and_Plate_MicroLabBox_student_M->sfcnInfo);

      /* Allocate memory of model methods 2 */
      {
        ssSetModelMethods2(rts,
                           &Ball_and_Plate_MicroLabBox_student_M->NonInlinedSFcns.methods2
                           [6]);
      }

      /* Allocate memory of model methods 3 */
      {
        ssSetModelMethods3(rts,
                           &Ball_and_Plate_MicroLabBox_student_M->NonInlinedSFcns.methods3
                           [6]);
      }

      /* Allocate memory of model methods 4 */
      {
        ssSetModelMethods4(rts,
                           &Ball_and_Plate_MicroLabBox_student_M->NonInlinedSFcns.methods4
                           [6]);
      }

      /* Allocate memory for states auxilliary information */
      {
        ssSetStatesInfo2(rts,
                         &Ball_and_Plate_MicroLabBox_student_M->NonInlinedSFcns.statesInfo2
                         [6]);
        ssSetPeriodicStatesInfo(rts,
          &Ball_and_Plate_MicroLabBox_student_M->NonInlinedSFcns.periodicStatesInfo
          [6]);
      }

      /* inputs */
      {
        _ssSetNumInputPorts(rts, 1);
        ssSetPortInfoForInputs(rts,
          &Ball_and_Plate_MicroLabBox_student_M->NonInlinedSFcns.Sfcn6.inputPortInfo
          [0]);
        _ssSetPortInfo2ForInputUnits(rts,
          &Ball_and_Plate_MicroLabBox_student_M->NonInlinedSFcns.Sfcn6.inputPortUnits
          [0]);
        ssSetInputPortUnit(rts, 0, 0);
        _ssSetPortInfo2ForInputCoSimAttribute(rts,
          &Ball_and_Plate_MicroLabBox_student_M->NonInlinedSFcns.Sfcn6.inputPortCoSimAttribute
          [0]);
        ssSetInputPortIsContinuousQuantity(rts, 0, 0);

        /* port 0 */
        {
          real_T const **sfcnUPtrs = (real_T const **)
            &Ball_and_Plate_MicroLabBox_student_M->NonInlinedSFcns.Sfcn6.UPtrs0;
          sfcnUPtrs[0] = &Ball_and_Plate_MicroLabBox_student_B.Gain1_b4;
          ssSetInputPortSignalPtrs(rts, 0, (InputPtrsType)&sfcnUPtrs[0]);
          _ssSetInputPortNumDimensions(rts, 0, 1);
          ssSetInputPortWidth(rts, 0, 1);
        }
      }

      /* outputs */
      {
        ssSetPortInfoForOutputs(rts,
          &Ball_and_Plate_MicroLabBox_student_M->NonInlinedSFcns.Sfcn6.outputPortInfo
          [0]);
        _ssSetNumOutputPorts(rts, 1);
        _ssSetPortInfo2ForOutputUnits(rts,
          &Ball_and_Plate_MicroLabBox_student_M->NonInlinedSFcns.Sfcn6.outputPortUnits
          [0]);
        ssSetOutputPortUnit(rts, 0, 0);
        _ssSetPortInfo2ForOutputCoSimAttribute(rts,
          &Ball_and_Plate_MicroLabBox_student_M->NonInlinedSFcns.Sfcn6.outputPortCoSimAttribute
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
                "Ball_and_Plate_MicroLabBox_student/Innerloop_Actuator/Motor_A/motorA /Dctleadlag2");
      ssSetRTModel(rts,Ball_and_Plate_MicroLabBox_student_M);
      ssSetParentSS(rts, (NULL));
      ssSetRootSS(rts, rts);
      ssSetVersion(rts, SIMSTRUCT_VERSION_LEVEL2);

      /* parameters */
      {
        mxArray **sfcnParams = (mxArray **)
          &Ball_and_Plate_MicroLabBox_student_M->NonInlinedSFcns.Sfcn6.params;
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
          &Ball_and_Plate_MicroLabBox_student_M->NonInlinedSFcns.Sfcn6.dWork;
        struct _ssDWorkAuxRecord *dWorkAuxRecord = (struct _ssDWorkAuxRecord *)
          &Ball_and_Plate_MicroLabBox_student_M->NonInlinedSFcns.Sfcn6.dWorkAux;
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

    /* Level2 S-Function Block: Ball_and_Plate_MicroLabBox_student/<S30>/Dct1lowpass3 (dlowpass1) */
    {
      SimStruct *rts = Ball_and_Plate_MicroLabBox_student_M->childSfunctions[7];

      /* timing info */
      time_T *sfcnPeriod =
        Ball_and_Plate_MicroLabBox_student_M->NonInlinedSFcns.Sfcn7.sfcnPeriod;
      time_T *sfcnOffset =
        Ball_and_Plate_MicroLabBox_student_M->NonInlinedSFcns.Sfcn7.sfcnOffset;
      int_T *sfcnTsMap =
        Ball_and_Plate_MicroLabBox_student_M->NonInlinedSFcns.Sfcn7.sfcnTsMap;
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
                         [7]);
      }

      _ssSetBlkInfo2PortInfo2Ptr(rts,
        &Ball_and_Plate_MicroLabBox_student_M->NonInlinedSFcns.inputOutputPortInfo2
        [7]);

      /* Set up the mdlInfo pointer */
      ssSetRTWSfcnInfo(rts, Ball_and_Plate_MicroLabBox_student_M->sfcnInfo);

      /* Allocate memory of model methods 2 */
      {
        ssSetModelMethods2(rts,
                           &Ball_and_Plate_MicroLabBox_student_M->NonInlinedSFcns.methods2
                           [7]);
      }

      /* Allocate memory of model methods 3 */
      {
        ssSetModelMethods3(rts,
                           &Ball_and_Plate_MicroLabBox_student_M->NonInlinedSFcns.methods3
                           [7]);
      }

      /* Allocate memory of model methods 4 */
      {
        ssSetModelMethods4(rts,
                           &Ball_and_Plate_MicroLabBox_student_M->NonInlinedSFcns.methods4
                           [7]);
      }

      /* Allocate memory for states auxilliary information */
      {
        ssSetStatesInfo2(rts,
                         &Ball_and_Plate_MicroLabBox_student_M->NonInlinedSFcns.statesInfo2
                         [7]);
        ssSetPeriodicStatesInfo(rts,
          &Ball_and_Plate_MicroLabBox_student_M->NonInlinedSFcns.periodicStatesInfo
          [7]);
      }

      /* inputs */
      {
        _ssSetNumInputPorts(rts, 1);
        ssSetPortInfoForInputs(rts,
          &Ball_and_Plate_MicroLabBox_student_M->NonInlinedSFcns.Sfcn7.inputPortInfo
          [0]);
        _ssSetPortInfo2ForInputUnits(rts,
          &Ball_and_Plate_MicroLabBox_student_M->NonInlinedSFcns.Sfcn7.inputPortUnits
          [0]);
        ssSetInputPortUnit(rts, 0, 0);
        _ssSetPortInfo2ForInputCoSimAttribute(rts,
          &Ball_and_Plate_MicroLabBox_student_M->NonInlinedSFcns.Sfcn7.inputPortCoSimAttribute
          [0]);
        ssSetInputPortIsContinuousQuantity(rts, 0, 0);

        /* port 0 */
        {
          real_T const **sfcnUPtrs = (real_T const **)
            &Ball_and_Plate_MicroLabBox_student_M->NonInlinedSFcns.Sfcn7.UPtrs0;
          sfcnUPtrs[0] = &Ball_and_Plate_MicroLabBox_student_B.Dctleadlag2;
          ssSetInputPortSignalPtrs(rts, 0, (InputPtrsType)&sfcnUPtrs[0]);
          _ssSetInputPortNumDimensions(rts, 0, 1);
          ssSetInputPortWidth(rts, 0, 1);
        }
      }

      /* outputs */
      {
        ssSetPortInfoForOutputs(rts,
          &Ball_and_Plate_MicroLabBox_student_M->NonInlinedSFcns.Sfcn7.outputPortInfo
          [0]);
        _ssSetNumOutputPorts(rts, 1);
        _ssSetPortInfo2ForOutputUnits(rts,
          &Ball_and_Plate_MicroLabBox_student_M->NonInlinedSFcns.Sfcn7.outputPortUnits
          [0]);
        ssSetOutputPortUnit(rts, 0, 0);
        _ssSetPortInfo2ForOutputCoSimAttribute(rts,
          &Ball_and_Plate_MicroLabBox_student_M->NonInlinedSFcns.Sfcn7.outputPortCoSimAttribute
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
                "Ball_and_Plate_MicroLabBox_student/Innerloop_Actuator/Motor_A/motorA /Dct1lowpass3");
      ssSetRTModel(rts,Ball_and_Plate_MicroLabBox_student_M);
      ssSetParentSS(rts, (NULL));
      ssSetRootSS(rts, rts);
      ssSetVersion(rts, SIMSTRUCT_VERSION_LEVEL2);

      /* parameters */
      {
        mxArray **sfcnParams = (mxArray **)
          &Ball_and_Plate_MicroLabBox_student_M->NonInlinedSFcns.Sfcn7.params;
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
          &Ball_and_Plate_MicroLabBox_student_M->NonInlinedSFcns.Sfcn7.dWork;
        struct _ssDWorkAuxRecord *dWorkAuxRecord = (struct _ssDWorkAuxRecord *)
          &Ball_and_Plate_MicroLabBox_student_M->NonInlinedSFcns.Sfcn7.dWorkAux;
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

    /* Level2 S-Function Block: Ball_and_Plate_MicroLabBox_student/<S45>/Dctleadlag2 (dleadlag) */
    {
      SimStruct *rts = Ball_and_Plate_MicroLabBox_student_M->childSfunctions[8];

      /* timing info */
      time_T *sfcnPeriod =
        Ball_and_Plate_MicroLabBox_student_M->NonInlinedSFcns.Sfcn8.sfcnPeriod;
      time_T *sfcnOffset =
        Ball_and_Plate_MicroLabBox_student_M->NonInlinedSFcns.Sfcn8.sfcnOffset;
      int_T *sfcnTsMap =
        Ball_and_Plate_MicroLabBox_student_M->NonInlinedSFcns.Sfcn8.sfcnTsMap;
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
                         [8]);
      }

      _ssSetBlkInfo2PortInfo2Ptr(rts,
        &Ball_and_Plate_MicroLabBox_student_M->NonInlinedSFcns.inputOutputPortInfo2
        [8]);

      /* Set up the mdlInfo pointer */
      ssSetRTWSfcnInfo(rts, Ball_and_Plate_MicroLabBox_student_M->sfcnInfo);

      /* Allocate memory of model methods 2 */
      {
        ssSetModelMethods2(rts,
                           &Ball_and_Plate_MicroLabBox_student_M->NonInlinedSFcns.methods2
                           [8]);
      }

      /* Allocate memory of model methods 3 */
      {
        ssSetModelMethods3(rts,
                           &Ball_and_Plate_MicroLabBox_student_M->NonInlinedSFcns.methods3
                           [8]);
      }

      /* Allocate memory of model methods 4 */
      {
        ssSetModelMethods4(rts,
                           &Ball_and_Plate_MicroLabBox_student_M->NonInlinedSFcns.methods4
                           [8]);
      }

      /* Allocate memory for states auxilliary information */
      {
        ssSetStatesInfo2(rts,
                         &Ball_and_Plate_MicroLabBox_student_M->NonInlinedSFcns.statesInfo2
                         [8]);
        ssSetPeriodicStatesInfo(rts,
          &Ball_and_Plate_MicroLabBox_student_M->NonInlinedSFcns.periodicStatesInfo
          [8]);
      }

      /* inputs */
      {
        _ssSetNumInputPorts(rts, 1);
        ssSetPortInfoForInputs(rts,
          &Ball_and_Plate_MicroLabBox_student_M->NonInlinedSFcns.Sfcn8.inputPortInfo
          [0]);
        _ssSetPortInfo2ForInputUnits(rts,
          &Ball_and_Plate_MicroLabBox_student_M->NonInlinedSFcns.Sfcn8.inputPortUnits
          [0]);
        ssSetInputPortUnit(rts, 0, 0);
        _ssSetPortInfo2ForInputCoSimAttribute(rts,
          &Ball_and_Plate_MicroLabBox_student_M->NonInlinedSFcns.Sfcn8.inputPortCoSimAttribute
          [0]);
        ssSetInputPortIsContinuousQuantity(rts, 0, 0);

        /* port 0 */
        {
          real_T const **sfcnUPtrs = (real_T const **)
            &Ball_and_Plate_MicroLabBox_student_M->NonInlinedSFcns.Sfcn8.UPtrs0;
          sfcnUPtrs[0] = &Ball_and_Plate_MicroLabBox_student_B.Gain1_f;
          ssSetInputPortSignalPtrs(rts, 0, (InputPtrsType)&sfcnUPtrs[0]);
          _ssSetInputPortNumDimensions(rts, 0, 1);
          ssSetInputPortWidth(rts, 0, 1);
        }
      }

      /* outputs */
      {
        ssSetPortInfoForOutputs(rts,
          &Ball_and_Plate_MicroLabBox_student_M->NonInlinedSFcns.Sfcn8.outputPortInfo
          [0]);
        _ssSetNumOutputPorts(rts, 1);
        _ssSetPortInfo2ForOutputUnits(rts,
          &Ball_and_Plate_MicroLabBox_student_M->NonInlinedSFcns.Sfcn8.outputPortUnits
          [0]);
        ssSetOutputPortUnit(rts, 0, 0);
        _ssSetPortInfo2ForOutputCoSimAttribute(rts,
          &Ball_and_Plate_MicroLabBox_student_M->NonInlinedSFcns.Sfcn8.outputPortCoSimAttribute
          [0]);
        ssSetOutputPortIsContinuousQuantity(rts, 0, 0);

        /* port 0 */
        {
          _ssSetOutputPortNumDimensions(rts, 0, 1);
          ssSetOutputPortWidth(rts, 0, 1);
          ssSetOutputPortSignal(rts, 0, ((real_T *)
            &Ball_and_Plate_MicroLabBox_student_B.Dctleadlag2_k));
        }
      }

      /* path info */
      ssSetModelName(rts, "Dctleadlag2");
      ssSetPath(rts,
                "Ball_and_Plate_MicroLabBox_student/Innerloop_Actuator/Motor_B/motorB/Dctleadlag2");
      ssSetRTModel(rts,Ball_and_Plate_MicroLabBox_student_M);
      ssSetParentSS(rts, (NULL));
      ssSetRootSS(rts, rts);
      ssSetVersion(rts, SIMSTRUCT_VERSION_LEVEL2);

      /* parameters */
      {
        mxArray **sfcnParams = (mxArray **)
          &Ball_and_Plate_MicroLabBox_student_M->NonInlinedSFcns.Sfcn8.params;
        ssSetSFcnParamsCount(rts, 3);
        ssSetSFcnParamsPtr(rts, &sfcnParams[0]);
        ssSetSFcnParam(rts, 0, (mxArray*)
                       Ball_and_Plate_MicroLabBox_student_P.Dctleadlag2_P1_Size_o);
        ssSetSFcnParam(rts, 1, (mxArray*)
                       Ball_and_Plate_MicroLabBox_student_P.Dctleadlag2_P2_Size_b);
        ssSetSFcnParam(rts, 2, (mxArray*)
                       Ball_and_Plate_MicroLabBox_student_P.Dctleadlag2_P3_Size_f);
      }

      /* work vectors */
      ssSetRWork(rts, (real_T *)
                 &Ball_and_Plate_MicroLabBox_student_DW.Dctleadlag2_RWORK_o[0]);

      {
        struct _ssDWorkRecord *dWorkRecord = (struct _ssDWorkRecord *)
          &Ball_and_Plate_MicroLabBox_student_M->NonInlinedSFcns.Sfcn8.dWork;
        struct _ssDWorkAuxRecord *dWorkAuxRecord = (struct _ssDWorkAuxRecord *)
          &Ball_and_Plate_MicroLabBox_student_M->NonInlinedSFcns.Sfcn8.dWorkAux;
        ssSetSFcnDWork(rts, dWorkRecord);
        ssSetSFcnDWorkAux(rts, dWorkAuxRecord);
        _ssSetNumDWork(rts, 1);

        /* RWORK */
        ssSetDWorkWidth(rts, 0, 2);
        ssSetDWorkDataType(rts, 0,SS_DOUBLE);
        ssSetDWorkComplexSignal(rts, 0, 0);
        ssSetDWork(rts, 0,
                   &Ball_and_Plate_MicroLabBox_student_DW.Dctleadlag2_RWORK_o[0]);
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

    /* Level2 S-Function Block: Ball_and_Plate_MicroLabBox_student/<S45>/Dct1lowpass3 (dlowpass1) */
    {
      SimStruct *rts = Ball_and_Plate_MicroLabBox_student_M->childSfunctions[9];

      /* timing info */
      time_T *sfcnPeriod =
        Ball_and_Plate_MicroLabBox_student_M->NonInlinedSFcns.Sfcn9.sfcnPeriod;
      time_T *sfcnOffset =
        Ball_and_Plate_MicroLabBox_student_M->NonInlinedSFcns.Sfcn9.sfcnOffset;
      int_T *sfcnTsMap =
        Ball_and_Plate_MicroLabBox_student_M->NonInlinedSFcns.Sfcn9.sfcnTsMap;
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
                         [9]);
      }

      _ssSetBlkInfo2PortInfo2Ptr(rts,
        &Ball_and_Plate_MicroLabBox_student_M->NonInlinedSFcns.inputOutputPortInfo2
        [9]);

      /* Set up the mdlInfo pointer */
      ssSetRTWSfcnInfo(rts, Ball_and_Plate_MicroLabBox_student_M->sfcnInfo);

      /* Allocate memory of model methods 2 */
      {
        ssSetModelMethods2(rts,
                           &Ball_and_Plate_MicroLabBox_student_M->NonInlinedSFcns.methods2
                           [9]);
      }

      /* Allocate memory of model methods 3 */
      {
        ssSetModelMethods3(rts,
                           &Ball_and_Plate_MicroLabBox_student_M->NonInlinedSFcns.methods3
                           [9]);
      }

      /* Allocate memory of model methods 4 */
      {
        ssSetModelMethods4(rts,
                           &Ball_and_Plate_MicroLabBox_student_M->NonInlinedSFcns.methods4
                           [9]);
      }

      /* Allocate memory for states auxilliary information */
      {
        ssSetStatesInfo2(rts,
                         &Ball_and_Plate_MicroLabBox_student_M->NonInlinedSFcns.statesInfo2
                         [9]);
        ssSetPeriodicStatesInfo(rts,
          &Ball_and_Plate_MicroLabBox_student_M->NonInlinedSFcns.periodicStatesInfo
          [9]);
      }

      /* inputs */
      {
        _ssSetNumInputPorts(rts, 1);
        ssSetPortInfoForInputs(rts,
          &Ball_and_Plate_MicroLabBox_student_M->NonInlinedSFcns.Sfcn9.inputPortInfo
          [0]);
        _ssSetPortInfo2ForInputUnits(rts,
          &Ball_and_Plate_MicroLabBox_student_M->NonInlinedSFcns.Sfcn9.inputPortUnits
          [0]);
        ssSetInputPortUnit(rts, 0, 0);
        _ssSetPortInfo2ForInputCoSimAttribute(rts,
          &Ball_and_Plate_MicroLabBox_student_M->NonInlinedSFcns.Sfcn9.inputPortCoSimAttribute
          [0]);
        ssSetInputPortIsContinuousQuantity(rts, 0, 0);

        /* port 0 */
        {
          real_T const **sfcnUPtrs = (real_T const **)
            &Ball_and_Plate_MicroLabBox_student_M->NonInlinedSFcns.Sfcn9.UPtrs0;
          sfcnUPtrs[0] = &Ball_and_Plate_MicroLabBox_student_B.Dctleadlag2_k;
          ssSetInputPortSignalPtrs(rts, 0, (InputPtrsType)&sfcnUPtrs[0]);
          _ssSetInputPortNumDimensions(rts, 0, 1);
          ssSetInputPortWidth(rts, 0, 1);
        }
      }

      /* outputs */
      {
        ssSetPortInfoForOutputs(rts,
          &Ball_and_Plate_MicroLabBox_student_M->NonInlinedSFcns.Sfcn9.outputPortInfo
          [0]);
        _ssSetNumOutputPorts(rts, 1);
        _ssSetPortInfo2ForOutputUnits(rts,
          &Ball_and_Plate_MicroLabBox_student_M->NonInlinedSFcns.Sfcn9.outputPortUnits
          [0]);
        ssSetOutputPortUnit(rts, 0, 0);
        _ssSetPortInfo2ForOutputCoSimAttribute(rts,
          &Ball_and_Plate_MicroLabBox_student_M->NonInlinedSFcns.Sfcn9.outputPortCoSimAttribute
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
                "Ball_and_Plate_MicroLabBox_student/Innerloop_Actuator/Motor_B/motorB/Dct1lowpass3");
      ssSetRTModel(rts,Ball_and_Plate_MicroLabBox_student_M);
      ssSetParentSS(rts, (NULL));
      ssSetRootSS(rts, rts);
      ssSetVersion(rts, SIMSTRUCT_VERSION_LEVEL2);

      /* parameters */
      {
        mxArray **sfcnParams = (mxArray **)
          &Ball_and_Plate_MicroLabBox_student_M->NonInlinedSFcns.Sfcn9.params;
        ssSetSFcnParamsCount(rts, 2);
        ssSetSFcnParamsPtr(rts, &sfcnParams[0]);
        ssSetSFcnParam(rts, 0, (mxArray*)
                       Ball_and_Plate_MicroLabBox_student_P.Dct1lowpass3_P1_Size_m);
        ssSetSFcnParam(rts, 1, (mxArray*)
                       Ball_and_Plate_MicroLabBox_student_P.Dct1lowpass3_P2_Size_i);
      }

      /* work vectors */
      ssSetRWork(rts, (real_T *)
                 &Ball_and_Plate_MicroLabBox_student_DW.Dct1lowpass3_RWORK_b[0]);

      {
        struct _ssDWorkRecord *dWorkRecord = (struct _ssDWorkRecord *)
          &Ball_and_Plate_MicroLabBox_student_M->NonInlinedSFcns.Sfcn9.dWork;
        struct _ssDWorkAuxRecord *dWorkAuxRecord = (struct _ssDWorkAuxRecord *)
          &Ball_and_Plate_MicroLabBox_student_M->NonInlinedSFcns.Sfcn9.dWorkAux;
        ssSetSFcnDWork(rts, dWorkRecord);
        ssSetSFcnDWorkAux(rts, dWorkAuxRecord);
        _ssSetNumDWork(rts, 1);

        /* RWORK */
        ssSetDWorkWidth(rts, 0, 2);
        ssSetDWorkDataType(rts, 0,SS_DOUBLE);
        ssSetDWorkComplexSignal(rts, 0, 0);
        ssSetDWork(rts, 0,
                   &Ball_and_Plate_MicroLabBox_student_DW.Dct1lowpass3_RWORK_b[0]);
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

    /* Level2 S-Function Block: Ball_and_Plate_MicroLabBox_student/<S61>/Dctleadlag2 (dleadlag) */
    {
      SimStruct *rts = Ball_and_Plate_MicroLabBox_student_M->childSfunctions[10];

      /* timing info */
      time_T *sfcnPeriod =
        Ball_and_Plate_MicroLabBox_student_M->NonInlinedSFcns.Sfcn10.sfcnPeriod;
      time_T *sfcnOffset =
        Ball_and_Plate_MicroLabBox_student_M->NonInlinedSFcns.Sfcn10.sfcnOffset;
      int_T *sfcnTsMap =
        Ball_and_Plate_MicroLabBox_student_M->NonInlinedSFcns.Sfcn10.sfcnTsMap;
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
                         [10]);
      }

      _ssSetBlkInfo2PortInfo2Ptr(rts,
        &Ball_and_Plate_MicroLabBox_student_M->NonInlinedSFcns.inputOutputPortInfo2
        [10]);

      /* Set up the mdlInfo pointer */
      ssSetRTWSfcnInfo(rts, Ball_and_Plate_MicroLabBox_student_M->sfcnInfo);

      /* Allocate memory of model methods 2 */
      {
        ssSetModelMethods2(rts,
                           &Ball_and_Plate_MicroLabBox_student_M->NonInlinedSFcns.methods2
                           [10]);
      }

      /* Allocate memory of model methods 3 */
      {
        ssSetModelMethods3(rts,
                           &Ball_and_Plate_MicroLabBox_student_M->NonInlinedSFcns.methods3
                           [10]);
      }

      /* Allocate memory of model methods 4 */
      {
        ssSetModelMethods4(rts,
                           &Ball_and_Plate_MicroLabBox_student_M->NonInlinedSFcns.methods4
                           [10]);
      }

      /* Allocate memory for states auxilliary information */
      {
        ssSetStatesInfo2(rts,
                         &Ball_and_Plate_MicroLabBox_student_M->NonInlinedSFcns.statesInfo2
                         [10]);
        ssSetPeriodicStatesInfo(rts,
          &Ball_and_Plate_MicroLabBox_student_M->NonInlinedSFcns.periodicStatesInfo
          [10]);
      }

      /* inputs */
      {
        _ssSetNumInputPorts(rts, 1);
        ssSetPortInfoForInputs(rts,
          &Ball_and_Plate_MicroLabBox_student_M->NonInlinedSFcns.Sfcn10.inputPortInfo
          [0]);
        _ssSetPortInfo2ForInputUnits(rts,
          &Ball_and_Plate_MicroLabBox_student_M->NonInlinedSFcns.Sfcn10.inputPortUnits
          [0]);
        ssSetInputPortUnit(rts, 0, 0);
        _ssSetPortInfo2ForInputCoSimAttribute(rts,
          &Ball_and_Plate_MicroLabBox_student_M->NonInlinedSFcns.Sfcn10.inputPortCoSimAttribute
          [0]);
        ssSetInputPortIsContinuousQuantity(rts, 0, 0);

        /* port 0 */
        {
          real_T const **sfcnUPtrs = (real_T const **)
            &Ball_and_Plate_MicroLabBox_student_M->NonInlinedSFcns.Sfcn10.UPtrs0;
          sfcnUPtrs[0] = &Ball_and_Plate_MicroLabBox_student_B.Gain1_mn;
          ssSetInputPortSignalPtrs(rts, 0, (InputPtrsType)&sfcnUPtrs[0]);
          _ssSetInputPortNumDimensions(rts, 0, 1);
          ssSetInputPortWidth(rts, 0, 1);
        }
      }

      /* outputs */
      {
        ssSetPortInfoForOutputs(rts,
          &Ball_and_Plate_MicroLabBox_student_M->NonInlinedSFcns.Sfcn10.outputPortInfo
          [0]);
        _ssSetNumOutputPorts(rts, 1);
        _ssSetPortInfo2ForOutputUnits(rts,
          &Ball_and_Plate_MicroLabBox_student_M->NonInlinedSFcns.Sfcn10.outputPortUnits
          [0]);
        ssSetOutputPortUnit(rts, 0, 0);
        _ssSetPortInfo2ForOutputCoSimAttribute(rts,
          &Ball_and_Plate_MicroLabBox_student_M->NonInlinedSFcns.Sfcn10.outputPortCoSimAttribute
          [0]);
        ssSetOutputPortIsContinuousQuantity(rts, 0, 0);

        /* port 0 */
        {
          _ssSetOutputPortNumDimensions(rts, 0, 1);
          ssSetOutputPortWidth(rts, 0, 1);
          ssSetOutputPortSignal(rts, 0, ((real_T *)
            &Ball_and_Plate_MicroLabBox_student_B.Dctleadlag2_a));
        }
      }

      /* path info */
      ssSetModelName(rts, "Dctleadlag2");
      ssSetPath(rts,
                "Ball_and_Plate_MicroLabBox_student/Innerloop_Actuator/Motor_C/motorC/Dctleadlag2");
      ssSetRTModel(rts,Ball_and_Plate_MicroLabBox_student_M);
      ssSetParentSS(rts, (NULL));
      ssSetRootSS(rts, rts);
      ssSetVersion(rts, SIMSTRUCT_VERSION_LEVEL2);

      /* parameters */
      {
        mxArray **sfcnParams = (mxArray **)
          &Ball_and_Plate_MicroLabBox_student_M->NonInlinedSFcns.Sfcn10.params;
        ssSetSFcnParamsCount(rts, 3);
        ssSetSFcnParamsPtr(rts, &sfcnParams[0]);
        ssSetSFcnParam(rts, 0, (mxArray*)
                       Ball_and_Plate_MicroLabBox_student_P.Dctleadlag2_P1_Size_f);
        ssSetSFcnParam(rts, 1, (mxArray*)
                       Ball_and_Plate_MicroLabBox_student_P.Dctleadlag2_P2_Size_d);
        ssSetSFcnParam(rts, 2, (mxArray*)
                       Ball_and_Plate_MicroLabBox_student_P.Dctleadlag2_P3_Size_p);
      }

      /* work vectors */
      ssSetRWork(rts, (real_T *)
                 &Ball_and_Plate_MicroLabBox_student_DW.Dctleadlag2_RWORK_p[0]);

      {
        struct _ssDWorkRecord *dWorkRecord = (struct _ssDWorkRecord *)
          &Ball_and_Plate_MicroLabBox_student_M->NonInlinedSFcns.Sfcn10.dWork;
        struct _ssDWorkAuxRecord *dWorkAuxRecord = (struct _ssDWorkAuxRecord *)
          &Ball_and_Plate_MicroLabBox_student_M->NonInlinedSFcns.Sfcn10.dWorkAux;
        ssSetSFcnDWork(rts, dWorkRecord);
        ssSetSFcnDWorkAux(rts, dWorkAuxRecord);
        _ssSetNumDWork(rts, 1);

        /* RWORK */
        ssSetDWorkWidth(rts, 0, 2);
        ssSetDWorkDataType(rts, 0,SS_DOUBLE);
        ssSetDWorkComplexSignal(rts, 0, 0);
        ssSetDWork(rts, 0,
                   &Ball_and_Plate_MicroLabBox_student_DW.Dctleadlag2_RWORK_p[0]);
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

    /* Level2 S-Function Block: Ball_and_Plate_MicroLabBox_student/<S61>/Dct1lowpass3 (dlowpass1) */
    {
      SimStruct *rts = Ball_and_Plate_MicroLabBox_student_M->childSfunctions[11];

      /* timing info */
      time_T *sfcnPeriod =
        Ball_and_Plate_MicroLabBox_student_M->NonInlinedSFcns.Sfcn11.sfcnPeriod;
      time_T *sfcnOffset =
        Ball_and_Plate_MicroLabBox_student_M->NonInlinedSFcns.Sfcn11.sfcnOffset;
      int_T *sfcnTsMap =
        Ball_and_Plate_MicroLabBox_student_M->NonInlinedSFcns.Sfcn11.sfcnTsMap;
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
                         [11]);
      }

      _ssSetBlkInfo2PortInfo2Ptr(rts,
        &Ball_and_Plate_MicroLabBox_student_M->NonInlinedSFcns.inputOutputPortInfo2
        [11]);

      /* Set up the mdlInfo pointer */
      ssSetRTWSfcnInfo(rts, Ball_and_Plate_MicroLabBox_student_M->sfcnInfo);

      /* Allocate memory of model methods 2 */
      {
        ssSetModelMethods2(rts,
                           &Ball_and_Plate_MicroLabBox_student_M->NonInlinedSFcns.methods2
                           [11]);
      }

      /* Allocate memory of model methods 3 */
      {
        ssSetModelMethods3(rts,
                           &Ball_and_Plate_MicroLabBox_student_M->NonInlinedSFcns.methods3
                           [11]);
      }

      /* Allocate memory of model methods 4 */
      {
        ssSetModelMethods4(rts,
                           &Ball_and_Plate_MicroLabBox_student_M->NonInlinedSFcns.methods4
                           [11]);
      }

      /* Allocate memory for states auxilliary information */
      {
        ssSetStatesInfo2(rts,
                         &Ball_and_Plate_MicroLabBox_student_M->NonInlinedSFcns.statesInfo2
                         [11]);
        ssSetPeriodicStatesInfo(rts,
          &Ball_and_Plate_MicroLabBox_student_M->NonInlinedSFcns.periodicStatesInfo
          [11]);
      }

      /* inputs */
      {
        _ssSetNumInputPorts(rts, 1);
        ssSetPortInfoForInputs(rts,
          &Ball_and_Plate_MicroLabBox_student_M->NonInlinedSFcns.Sfcn11.inputPortInfo
          [0]);
        _ssSetPortInfo2ForInputUnits(rts,
          &Ball_and_Plate_MicroLabBox_student_M->NonInlinedSFcns.Sfcn11.inputPortUnits
          [0]);
        ssSetInputPortUnit(rts, 0, 0);
        _ssSetPortInfo2ForInputCoSimAttribute(rts,
          &Ball_and_Plate_MicroLabBox_student_M->NonInlinedSFcns.Sfcn11.inputPortCoSimAttribute
          [0]);
        ssSetInputPortIsContinuousQuantity(rts, 0, 0);

        /* port 0 */
        {
          real_T const **sfcnUPtrs = (real_T const **)
            &Ball_and_Plate_MicroLabBox_student_M->NonInlinedSFcns.Sfcn11.UPtrs0;
          sfcnUPtrs[0] = &Ball_and_Plate_MicroLabBox_student_B.Dctleadlag2_a;
          ssSetInputPortSignalPtrs(rts, 0, (InputPtrsType)&sfcnUPtrs[0]);
          _ssSetInputPortNumDimensions(rts, 0, 1);
          ssSetInputPortWidth(rts, 0, 1);
        }
      }

      /* outputs */
      {
        ssSetPortInfoForOutputs(rts,
          &Ball_and_Plate_MicroLabBox_student_M->NonInlinedSFcns.Sfcn11.outputPortInfo
          [0]);
        _ssSetNumOutputPorts(rts, 1);
        _ssSetPortInfo2ForOutputUnits(rts,
          &Ball_and_Plate_MicroLabBox_student_M->NonInlinedSFcns.Sfcn11.outputPortUnits
          [0]);
        ssSetOutputPortUnit(rts, 0, 0);
        _ssSetPortInfo2ForOutputCoSimAttribute(rts,
          &Ball_and_Plate_MicroLabBox_student_M->NonInlinedSFcns.Sfcn11.outputPortCoSimAttribute
          [0]);
        ssSetOutputPortIsContinuousQuantity(rts, 0, 0);

        /* port 0 */
        {
          _ssSetOutputPortNumDimensions(rts, 0, 1);
          ssSetOutputPortWidth(rts, 0, 1);
          ssSetOutputPortSignal(rts, 0, ((real_T *)
            &Ball_and_Plate_MicroLabBox_student_B.Dct1lowpass3_e));
        }
      }

      /* path info */
      ssSetModelName(rts, "Dct1lowpass3");
      ssSetPath(rts,
                "Ball_and_Plate_MicroLabBox_student/Innerloop_Actuator/Motor_C/motorC/Dct1lowpass3");
      ssSetRTModel(rts,Ball_and_Plate_MicroLabBox_student_M);
      ssSetParentSS(rts, (NULL));
      ssSetRootSS(rts, rts);
      ssSetVersion(rts, SIMSTRUCT_VERSION_LEVEL2);

      /* parameters */
      {
        mxArray **sfcnParams = (mxArray **)
          &Ball_and_Plate_MicroLabBox_student_M->NonInlinedSFcns.Sfcn11.params;
        ssSetSFcnParamsCount(rts, 2);
        ssSetSFcnParamsPtr(rts, &sfcnParams[0]);
        ssSetSFcnParam(rts, 0, (mxArray*)
                       Ball_and_Plate_MicroLabBox_student_P.Dct1lowpass3_P1_Size_d);
        ssSetSFcnParam(rts, 1, (mxArray*)
                       Ball_and_Plate_MicroLabBox_student_P.Dct1lowpass3_P2_Size_k);
      }

      /* work vectors */
      ssSetRWork(rts, (real_T *)
                 &Ball_and_Plate_MicroLabBox_student_DW.Dct1lowpass3_RWORK_d[0]);

      {
        struct _ssDWorkRecord *dWorkRecord = (struct _ssDWorkRecord *)
          &Ball_and_Plate_MicroLabBox_student_M->NonInlinedSFcns.Sfcn11.dWork;
        struct _ssDWorkAuxRecord *dWorkAuxRecord = (struct _ssDWorkAuxRecord *)
          &Ball_and_Plate_MicroLabBox_student_M->NonInlinedSFcns.Sfcn11.dWorkAux;
        ssSetSFcnDWork(rts, dWorkRecord);
        ssSetSFcnDWorkAux(rts, dWorkAuxRecord);
        _ssSetNumDWork(rts, 1);

        /* RWORK */
        ssSetDWorkWidth(rts, 0, 2);
        ssSetDWorkDataType(rts, 0,SS_DOUBLE);
        ssSetDWorkComplexSignal(rts, 0, 0);
        ssSetDWork(rts, 0,
                   &Ball_and_Plate_MicroLabBox_student_DW.Dct1lowpass3_RWORK_d[0]);
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

  /* Start for S-Function (dlowpass1): '<S10>/Dct1lowpass2' */
  /* Level2 S-Function Block: '<S10>/Dct1lowpass2' (dlowpass1) */
  {
    SimStruct *rts = Ball_and_Plate_MicroLabBox_student_M->childSfunctions[0];
    sfcnStart(rts);
    if (ssGetErrorStatus(rts) != (NULL))
      return;
  }

  /* Start for S-Function (dlowpass1): '<S11>/Dct1lowpass2' */
  /* Level2 S-Function Block: '<S11>/Dct1lowpass2' (dlowpass1) */
  {
    SimStruct *rts = Ball_and_Plate_MicroLabBox_student_M->childSfunctions[1];
    sfcnStart(rts);
    if (ssGetErrorStatus(rts) != (NULL))
      return;
  }

  /* Start for Enabled SubSystem: '<S37>/Enabled Subsystem' */
  Ball_and_EnabledSubsystem_Start
    (&Ball_and_Plate_MicroLabBox_student_DW.EnabledSubsystem);

  /* End of Start for SubSystem: '<S37>/Enabled Subsystem' */

  /* Start for Enabled SubSystem: '<S37>/Enabled Subsystem1' */
  Ball_an_EnabledSubsystem1_Start
    (&Ball_and_Plate_MicroLabBox_student_DW.EnabledSubsystem1);

  /* End of Start for SubSystem: '<S37>/Enabled Subsystem1' */
  /* End of Start for SubSystem: '<S21>/Position Measurement' */

  /* Start for Enabled SubSystem: '<S52>/Enabled Subsystem' */
  Ball_and_EnabledSubsystem_Start
    (&Ball_and_Plate_MicroLabBox_student_DW.EnabledSubsystem_a);

  /* End of Start for SubSystem: '<S52>/Enabled Subsystem' */

  /* Start for Enabled SubSystem: '<S52>/Enabled Subsystem1' */
  Ball_an_EnabledSubsystem1_Start
    (&Ball_and_Plate_MicroLabBox_student_DW.EnabledSubsystem1_f);

  /* End of Start for SubSystem: '<S52>/Enabled Subsystem1' */
  /* End of Start for SubSystem: '<S22>/Position Measurement' */

  /* Start for Enabled SubSystem: '<S69>/Enabled Subsystem' */
  Ball_and_EnabledSubsystem_Start
    (&Ball_and_Plate_MicroLabBox_student_DW.EnabledSubsystem_n);

  /* End of Start for SubSystem: '<S69>/Enabled Subsystem' */

  /* Start for Enabled SubSystem: '<S69>/Enabled Subsystem1' */
  Ball_an_EnabledSubsystem1_Start
    (&Ball_and_Plate_MicroLabBox_student_DW.EnabledSubsystem1_b);

  /* End of Start for SubSystem: '<S69>/Enabled Subsystem1' */
  /* End of Start for SubSystem: '<S23>/Position Measurement' */

  /* Start for S-Function (dlowpass1): '<S14>/Dct1lowpass2' */
  /* Level2 S-Function Block: '<S14>/Dct1lowpass2' (dlowpass1) */
  {
    SimStruct *rts = Ball_and_Plate_MicroLabBox_student_M->childSfunctions[2];
    sfcnStart(rts);
    if (ssGetErrorStatus(rts) != (NULL))
      return;
  }

  /* Start for S-Function (dlowpass1): '<S15>/Dct1lowpass2' */
  /* Level2 S-Function Block: '<S15>/Dct1lowpass2' (dlowpass1) */
  {
    SimStruct *rts = Ball_and_Plate_MicroLabBox_student_M->childSfunctions[3];
    sfcnStart(rts);
    if (ssGetErrorStatus(rts) != (NULL))
      return;
  }

  /* Start for S-Function (dlowpass1): '<S12>/Dct1lowpass2' */
  /* Level2 S-Function Block: '<S12>/Dct1lowpass2' (dlowpass1) */
  {
    SimStruct *rts = Ball_and_Plate_MicroLabBox_student_M->childSfunctions[4];
    sfcnStart(rts);
    if (ssGetErrorStatus(rts) != (NULL))
      return;
  }

  /* Start for S-Function (dlowpass1): '<S13>/Dct1lowpass2' */
  /* Level2 S-Function Block: '<S13>/Dct1lowpass2' (dlowpass1) */
  {
    SimStruct *rts = Ball_and_Plate_MicroLabBox_student_M->childSfunctions[5];
    sfcnStart(rts);
    if (ssGetErrorStatus(rts) != (NULL))
      return;
  }

  /* Start for S-Function (dleadlag): '<S30>/Dctleadlag2' */
  /* Level2 S-Function Block: '<S30>/Dctleadlag2' (dleadlag) */
  {
    SimStruct *rts = Ball_and_Plate_MicroLabBox_student_M->childSfunctions[6];
    sfcnStart(rts);
    if (ssGetErrorStatus(rts) != (NULL))
      return;
  }

  /* Start for S-Function (dlowpass1): '<S30>/Dct1lowpass3' */
  /* Level2 S-Function Block: '<S30>/Dct1lowpass3' (dlowpass1) */
  {
    SimStruct *rts = Ball_and_Plate_MicroLabBox_student_M->childSfunctions[7];
    sfcnStart(rts);
    if (ssGetErrorStatus(rts) != (NULL))
      return;
  }

  /* Start for S-Function (dleadlag): '<S45>/Dctleadlag2' */
  /* Level2 S-Function Block: '<S45>/Dctleadlag2' (dleadlag) */
  {
    SimStruct *rts = Ball_and_Plate_MicroLabBox_student_M->childSfunctions[8];
    sfcnStart(rts);
    if (ssGetErrorStatus(rts) != (NULL))
      return;
  }

  /* Start for S-Function (dlowpass1): '<S45>/Dct1lowpass3' */
  /* Level2 S-Function Block: '<S45>/Dct1lowpass3' (dlowpass1) */
  {
    SimStruct *rts = Ball_and_Plate_MicroLabBox_student_M->childSfunctions[9];
    sfcnStart(rts);
    if (ssGetErrorStatus(rts) != (NULL))
      return;
  }

  /* Start for S-Function (dleadlag): '<S61>/Dctleadlag2' */
  /* Level2 S-Function Block: '<S61>/Dctleadlag2' (dleadlag) */
  {
    SimStruct *rts = Ball_and_Plate_MicroLabBox_student_M->childSfunctions[10];
    sfcnStart(rts);
    if (ssGetErrorStatus(rts) != (NULL))
      return;
  }

  /* Start for S-Function (dlowpass1): '<S61>/Dct1lowpass3' */
  /* Level2 S-Function Block: '<S61>/Dct1lowpass3' (dlowpass1) */
  {
    SimStruct *rts = Ball_and_Plate_MicroLabBox_student_M->childSfunctions[11];
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
    static const real_T tmp[16] = { 0.1, 0.0, 0.0, 0.0, 0.0, 0.1, 0.0, 0.0, 0.0,
      0.0, 0.1, 0.0, 0.0, 0.0, 0.0, 0.1 };

    /* InitializeConditions for UnitDelay: '<S1>/UD' */
    Ball_and_Plate_MicroLabBox_student_DW.UD_DSTATE =
      Ball_and_Plate_MicroLabBox_student_P.Difference_ICPrevInput;

    /* InitializeConditions for FirstOrderHold: '<Root>/First Order Hold' */
    Ball_and_Plate_MicroLabBox_student_DW.Tk = (rtInf);
    Ball_and_Plate_MicroLabBox_student_DW.Ck =
      Ball_and_Plate_MicroLabBox_student_P.FirstOrderHold_IniOut;
    Ball_and_Plate_MicroLabBox_student_DW.Uk = (rtInf);
    Ball_and_Plate_MicroLabBox_student_DW.Mk = 0.0;

    /* InitializeConditions for FirstOrderHold: '<Root>/First Order Hold1' */
    Ball_and_Plate_MicroLabBox_student_DW.Tk_c = (rtInf);
    Ball_and_Plate_MicroLabBox_student_DW.Ck_h =
      Ball_and_Plate_MicroLabBox_student_P.FirstOrderHold1_IniOut;
    Ball_and_Plate_MicroLabBox_student_DW.Uk_i = (rtInf);
    Ball_and_Plate_MicroLabBox_student_DW.Mk_g = 0.0;

    /* InitializeConditions for UnitDelay: '<S72>/last_mv' */
    Ball_and_Plate_MicroLabBox_student_DW.last_mv_DSTATE[0] =
      Ball_and_Plate_MicroLabBox_student_P.last_mv_InitialCondition[0];
    Ball_and_Plate_MicroLabBox_student_DW.last_mv_DSTATE[1] =
      Ball_and_Plate_MicroLabBox_student_P.last_mv_InitialCondition[1];

    /* InitializeConditions for Memory: '<S72>/Memory' */
    memcpy(&Ball_and_Plate_MicroLabBox_student_DW.Memory_PreviousInput[0],
           &Ball_and_Plate_MicroLabBox_student_P.Memory_InitialCondition[0],
           212U * sizeof(boolean_T));

    /* InitializeConditions for UnitDelay: '<S2>/UD' */
    Ball_and_Plate_MicroLabBox_student_DW.UD_DSTATE_c =
      Ball_and_Plate_MicroLabBox_student_P.DiscreteDerivative_ICPrevScaled;

    /* InitializeConditions for UnitDelay: '<S3>/UD' */
    Ball_and_Plate_MicroLabBox_student_DW.UD_DSTATE_o =
      Ball_and_Plate_MicroLabBox_student_P.DiscreteDerivative1_ICPrevScale;

    /* InitializeConditions for Memory: '<S72>/last_x' */
    Ball_and_Plate_MicroLabBox_student_DW.last_x_PreviousInput[0] =
      Ball_and_Plate_MicroLabBox_student_P.last_x_InitialCondition[0];
    Ball_and_Plate_MicroLabBox_student_DW.last_x_PreviousInput[1] =
      Ball_and_Plate_MicroLabBox_student_P.last_x_InitialCondition[1];
    Ball_and_Plate_MicroLabBox_student_DW.last_x_PreviousInput[2] =
      Ball_and_Plate_MicroLabBox_student_P.last_x_InitialCondition[2];
    Ball_and_Plate_MicroLabBox_student_DW.last_x_PreviousInput[3] =
      Ball_and_Plate_MicroLabBox_student_P.last_x_InitialCondition[3];

    /* SystemInitialize for Atomic SubSystem: '<S21>/Position Measurement' */
    /* InitializeConditions for TransferFcn: '<S37>/Transfer Fcn' */
    Ball_and_Plate_MicroLabBox_student_X.TransferFcn_CSTATE_l = 0.0;

    /* SystemInitialize for Enabled SubSystem: '<S37>/Enabled Subsystem' */
    Ball_and__EnabledSubsystem_Init
      (&Ball_and_Plate_MicroLabBox_student_B.EnabledSubsystem,
       &Ball_and_Plate_MicroLabBox_student_P.EnabledSubsystem);

    /* End of SystemInitialize for SubSystem: '<S37>/Enabled Subsystem' */

    /* SystemInitialize for Enabled SubSystem: '<S37>/Enabled Subsystem1' */
    Ball_and_EnabledSubsystem1_Init
      (&Ball_and_Plate_MicroLabBox_student_B.EnabledSubsystem1,
       &Ball_and_Plate_MicroLabBox_student_P.EnabledSubsystem1);

    /* End of SystemInitialize for SubSystem: '<S37>/Enabled Subsystem1' */
    /* End of SystemInitialize for SubSystem: '<S21>/Position Measurement' */

    /* SystemInitialize for Atomic SubSystem: '<S22>/Position Measurement' */
    /* InitializeConditions for TransferFcn: '<S52>/Transfer Fcn' */
    Ball_and_Plate_MicroLabBox_student_X.TransferFcn_CSTATE_h = 0.0;

    /* SystemInitialize for Enabled SubSystem: '<S52>/Enabled Subsystem' */
    Ball_and__EnabledSubsystem_Init
      (&Ball_and_Plate_MicroLabBox_student_B.EnabledSubsystem_a,
       &Ball_and_Plate_MicroLabBox_student_P.EnabledSubsystem_a);

    /* End of SystemInitialize for SubSystem: '<S52>/Enabled Subsystem' */

    /* SystemInitialize for Enabled SubSystem: '<S52>/Enabled Subsystem1' */
    Ball_and_EnabledSubsystem1_Init
      (&Ball_and_Plate_MicroLabBox_student_B.EnabledSubsystem1_f,
       &Ball_and_Plate_MicroLabBox_student_P.EnabledSubsystem1_f);

    /* End of SystemInitialize for SubSystem: '<S52>/Enabled Subsystem1' */
    /* End of SystemInitialize for SubSystem: '<S22>/Position Measurement' */

    /* SystemInitialize for Atomic SubSystem: '<S23>/Position Measurement' */
    /* InitializeConditions for TransferFcn: '<S69>/Transfer Fcn' */
    Ball_and_Plate_MicroLabBox_student_X.TransferFcn_CSTATE = 0.0;

    /* SystemInitialize for Enabled SubSystem: '<S69>/Enabled Subsystem' */
    Ball_and__EnabledSubsystem_Init
      (&Ball_and_Plate_MicroLabBox_student_B.EnabledSubsystem_n,
       &Ball_and_Plate_MicroLabBox_student_P.EnabledSubsystem_n);

    /* End of SystemInitialize for SubSystem: '<S69>/Enabled Subsystem' */

    /* SystemInitialize for Enabled SubSystem: '<S69>/Enabled Subsystem1' */
    Ball_and_EnabledSubsystem1_Init
      (&Ball_and_Plate_MicroLabBox_student_B.EnabledSubsystem1_b,
       &Ball_and_Plate_MicroLabBox_student_P.EnabledSubsystem1_b);

    /* End of SystemInitialize for SubSystem: '<S69>/Enabled Subsystem1' */
    /* End of SystemInitialize for SubSystem: '<S23>/Position Measurement' */

    /* SystemInitialize for MATLAB Function: '<Root>/MATLAB Function' */
    memcpy(&Ball_and_Plate_MicroLabBox_student_DW.P[0], &tmp[0], sizeof(real_T) <<
           4U);
    Ball_and_Plate_MicroLabBox_student_DW.x_hat[0] = 0.0;
    Ball_and_Plate_MicroLabBox_student_DW.x_hat[1] = 0.0;
    Ball_and_Plate_MicroLabBox_student_DW.x_hat[2] = 0.0;
    Ball_and_Plate_MicroLabBox_student_DW.x_hat[3] = 0.0;

    /* SystemInitialize for MATLAB Function: '<Root>/MATLAB Function3' */
    Ball_and_Plate_MicroLabBox_student_DW.theta = 0.0;
    Ball_and_Plate_MicroLabBox_student_DW.current_R = 0.0;

    /* SystemInitialize for MATLAB Function: '<S21>/MATLAB Function1' */
    Ball_and_P_MATLABFunction1_Init
      (&Ball_and_Plate_MicroLabBox_student_DW.sf_MATLABFunction1);

    /* SystemInitialize for MATLAB Function: '<S27>/MATLAB Function' */
    Ball_and__MATLABFunction_o_Init
      (&Ball_and_Plate_MicroLabBox_student_DW.sf_MATLABFunction_e);

    /* SystemInitialize for MATLAB Function: '<S25>/MATLAB Function' */
    Ball_and_Pl_MATLABFunction_Init
      (&Ball_and_Plate_MicroLabBox_student_DW.sf_MATLABFunction_i);

    /* SystemInitialize for MATLAB Function: '<S22>/MATLAB Function2' */
    Ball_and_P_MATLABFunction1_Init
      (&Ball_and_Plate_MicroLabBox_student_DW.sf_MATLABFunction2_d);

    /* SystemInitialize for MATLAB Function: '<S42>/MATLAB Function' */
    Ball_and__MATLABFunction_o_Init
      (&Ball_and_Plate_MicroLabBox_student_DW.sf_MATLABFunction_ko);

    /* SystemInitialize for MATLAB Function: '<S40>/MATLAB Function' */
    Ball_and_Pl_MATLABFunction_Init
      (&Ball_and_Plate_MicroLabBox_student_DW.sf_MATLABFunction_k);

    /* SystemInitialize for MATLAB Function: '<S23>/MATLAB Function1' */
    Ball_and_P_MATLABFunction1_Init
      (&Ball_and_Plate_MicroLabBox_student_DW.sf_MATLABFunction1_h);

    /* SystemInitialize for MATLAB Function: '<S58>/MATLAB Function' */
    Ball_and__MATLABFunction_o_Init
      (&Ball_and_Plate_MicroLabBox_student_DW.sf_MATLABFunction_hg);

    /* SystemInitialize for MATLAB Function: '<S55>/MATLAB Function' */
    Ball_and_Pl_MATLABFunction_Init
      (&Ball_and_Plate_MicroLabBox_student_DW.sf_MATLABFunction_h);

    /* SystemInitialize for MATLAB Function: '<S57>/MATLAB Function' */
    Ball_and__MATLABFunction_o_Init
      (&Ball_and_Plate_MicroLabBox_student_DW.sf_MATLABFunction_g);
  }
}

/* Model terminate function */
void Ball_and_Plate_MicroLabBox_student_terminate(void)
{
  /* Terminate for S-Function (dlowpass1): '<S10>/Dct1lowpass2' */
  /* Level2 S-Function Block: '<S10>/Dct1lowpass2' (dlowpass1) */
  {
    SimStruct *rts = Ball_and_Plate_MicroLabBox_student_M->childSfunctions[0];
    sfcnTerminate(rts);
  }

  /* Terminate for S-Function (dlowpass1): '<S11>/Dct1lowpass2' */
  /* Level2 S-Function Block: '<S11>/Dct1lowpass2' (dlowpass1) */
  {
    SimStruct *rts = Ball_and_Plate_MicroLabBox_student_M->childSfunctions[1];
    sfcnTerminate(rts);
  }

  /* Terminate for Atomic SubSystem: '<S21>/Position Measurement' */
  /* Terminate for Triggered SubSystem: '<S29>/EMC_ENCODER_POS_SET_BL1' */
  /* Terminate for S-Function (rti_commonblock): '<S36>/S-Function1' incorporates:
   *  Constant: '<S29>/Constant'
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

  /* End of Terminate for SubSystem: '<S29>/EMC_ENCODER_POS_SET_BL1' */

  /* Terminate for S-Function (rti_commonblock): '<S35>/S-Function1' */

  /* --- Ball_and_Plate_MicroLabBox_student/Innerloop_Actuator/Motor_A/Position Measurement/EMC_ENCODER_BL1 --- */
  /* --- [RTIEMC, Encoder] - DIO class: 2 - Unit: 1 - Port: 1 - Channel: 1 --- */
  {
    /* Deactivates encoder interface functionality */
    DioCl2EncoderIn_stop(pRTIEmcEncoder_Unit_1_DioCl_2_Port_1_Ch1);
  }

  /* End of Terminate for SubSystem: '<S21>/Position Measurement' */

  /* Terminate for Atomic SubSystem: '<S22>/Position Measurement' */
  /* Terminate for Triggered SubSystem: '<S44>/EMC_ENCODER_POS_SET_BL1' */
  /* Terminate for S-Function (rti_commonblock): '<S51>/S-Function1' incorporates:
   *  Constant: '<S44>/Constant'
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

  /* End of Terminate for SubSystem: '<S44>/EMC_ENCODER_POS_SET_BL1' */

  /* Terminate for S-Function (rti_commonblock): '<S50>/S-Function1' */

  /* --- Ball_and_Plate_MicroLabBox_student/Innerloop_Actuator/Motor_B/Position Measurement/EMC_ENCODER_BL1 --- */
  /* --- [RTIEMC, Encoder] - DIO class: 2 - Unit: 2 - Port: 1 - Channel: 3 --- */
  {
    /* Deactivates encoder interface functionality */
    DioCl2EncoderIn_stop(pRTIEmcEncoder_Unit_2_DioCl_2_Port_1_Ch3);
  }

  /* End of Terminate for SubSystem: '<S22>/Position Measurement' */

  /* Terminate for Atomic SubSystem: '<S23>/Position Measurement' */
  /* Terminate for Triggered SubSystem: '<S60>/EMC_ENCODER_POS_SET_BL1' */
  /* Terminate for S-Function (rti_commonblock): '<S68>/S-Function1' incorporates:
   *  Constant: '<S60>/Constant'
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

  /* End of Terminate for SubSystem: '<S60>/EMC_ENCODER_POS_SET_BL1' */

  /* Terminate for S-Function (rti_commonblock): '<S67>/S-Function1' */

  /* --- Ball_and_Plate_MicroLabBox_student/Innerloop_Actuator/Motor_C/Position Measurement/EMC_ENCODER_BL1 --- */
  /* --- [RTIEMC, Encoder] - DIO class: 2 - Unit: 3 - Port: 1 - Channel: 5 --- */
  {
    /* Deactivates encoder interface functionality */
    DioCl2EncoderIn_stop(pRTIEmcEncoder_Unit_3_DioCl_2_Port_1_Ch5);
  }

  /* End of Terminate for SubSystem: '<S23>/Position Measurement' */

  /* Terminate for S-Function (dlowpass1): '<S14>/Dct1lowpass2' */
  /* Level2 S-Function Block: '<S14>/Dct1lowpass2' (dlowpass1) */
  {
    SimStruct *rts = Ball_and_Plate_MicroLabBox_student_M->childSfunctions[2];
    sfcnTerminate(rts);
  }

  /* Terminate for S-Function (dlowpass1): '<S15>/Dct1lowpass2' */
  /* Level2 S-Function Block: '<S15>/Dct1lowpass2' (dlowpass1) */
  {
    SimStruct *rts = Ball_and_Plate_MicroLabBox_student_M->childSfunctions[3];
    sfcnTerminate(rts);
  }

  /* Terminate for S-Function (dlowpass1): '<S12>/Dct1lowpass2' */
  /* Level2 S-Function Block: '<S12>/Dct1lowpass2' (dlowpass1) */
  {
    SimStruct *rts = Ball_and_Plate_MicroLabBox_student_M->childSfunctions[4];
    sfcnTerminate(rts);
  }

  /* Terminate for S-Function (dlowpass1): '<S13>/Dct1lowpass2' */
  /* Level2 S-Function Block: '<S13>/Dct1lowpass2' (dlowpass1) */
  {
    SimStruct *rts = Ball_and_Plate_MicroLabBox_student_M->childSfunctions[5];
    sfcnTerminate(rts);
  }

  /* Terminate for S-Function (rti_commonblock): '<S18>/S-Function1' */
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

  /* Terminate for S-Function (dleadlag): '<S30>/Dctleadlag2' */
  /* Level2 S-Function Block: '<S30>/Dctleadlag2' (dleadlag) */
  {
    SimStruct *rts = Ball_and_Plate_MicroLabBox_student_M->childSfunctions[6];
    sfcnTerminate(rts);
  }

  /* Terminate for S-Function (dlowpass1): '<S30>/Dct1lowpass3' */
  /* Level2 S-Function Block: '<S30>/Dct1lowpass3' (dlowpass1) */
  {
    SimStruct *rts = Ball_and_Plate_MicroLabBox_student_M->childSfunctions[7];
    sfcnTerminate(rts);
  }

  /* Terminate for Atomic SubSystem: '<S21>/Outputs to Amplifier' */
  /* Terminate for S-Function (rti_commonblock): '<S34>/S-Function1' */

  /* --- Ball_and_Plate_MicroLabBox_student/Innerloop_Actuator/Motor_A/Outputs to Amplifier/DAC_CLASS1_BL1 --- */
  /* --- [RTI120X, DAC C1] - Channel: 1 --- */

  /* All channel outputs are set to high impedance state */
  DacCl1AnalogOut_setOutputHighZ(pRTIDacC1AnalogOut_Ch_1, DAC_CLASS1_HIGH_Z_ON);

  /* Deactivates AnalogOut functionality */
  DacCl1AnalogOut_stop(pRTIDacC1AnalogOut_Ch_1);

  /* End of Terminate for SubSystem: '<S21>/Outputs to Amplifier' */

  /* Terminate for S-Function (dleadlag): '<S45>/Dctleadlag2' */
  /* Level2 S-Function Block: '<S45>/Dctleadlag2' (dleadlag) */
  {
    SimStruct *rts = Ball_and_Plate_MicroLabBox_student_M->childSfunctions[8];
    sfcnTerminate(rts);
  }

  /* Terminate for S-Function (dlowpass1): '<S45>/Dct1lowpass3' */
  /* Level2 S-Function Block: '<S45>/Dct1lowpass3' (dlowpass1) */
  {
    SimStruct *rts = Ball_and_Plate_MicroLabBox_student_M->childSfunctions[9];
    sfcnTerminate(rts);
  }

  /* Terminate for Atomic SubSystem: '<S22>/Outputs to Amplifier' */
  /* Terminate for S-Function (rti_commonblock): '<S49>/S-Function1' */

  /* --- Ball_and_Plate_MicroLabBox_student/Innerloop_Actuator/Motor_B/Outputs to Amplifier/DAC_CLASS1_BL1 --- */
  /* --- [RTI120X, DAC C1] - Channel: 2 --- */

  /* All channel outputs are set to high impedance state */
  DacCl1AnalogOut_setOutputHighZ(pRTIDacC1AnalogOut_Ch_2, DAC_CLASS1_HIGH_Z_ON);

  /* Deactivates AnalogOut functionality */
  DacCl1AnalogOut_stop(pRTIDacC1AnalogOut_Ch_2);

  /* End of Terminate for SubSystem: '<S22>/Outputs to Amplifier' */

  /* Terminate for S-Function (dleadlag): '<S61>/Dctleadlag2' */
  /* Level2 S-Function Block: '<S61>/Dctleadlag2' (dleadlag) */
  {
    SimStruct *rts = Ball_and_Plate_MicroLabBox_student_M->childSfunctions[10];
    sfcnTerminate(rts);
  }

  /* Terminate for S-Function (dlowpass1): '<S61>/Dct1lowpass3' */
  /* Level2 S-Function Block: '<S61>/Dct1lowpass3' (dlowpass1) */
  {
    SimStruct *rts = Ball_and_Plate_MicroLabBox_student_M->childSfunctions[11];
    sfcnTerminate(rts);
  }

  /* Terminate for Atomic SubSystem: '<S23>/Outputs to Amplifier' */
  /* Terminate for S-Function (rti_commonblock): '<S66>/S-Function1' */

  /* --- Ball_and_Plate_MicroLabBox_student/Innerloop_Actuator/Motor_C/Outputs to Amplifier/DAC_CLASS1_BL1 --- */
  /* --- [RTI120X, DAC C1] - Channel: 3 --- */

  /* All channel outputs are set to high impedance state */
  DacCl1AnalogOut_setOutputHighZ(pRTIDacC1AnalogOut_Ch_3, DAC_CLASS1_HIGH_Z_ON);

  /* Deactivates AnalogOut functionality */
  DacCl1AnalogOut_stop(pRTIDacC1AnalogOut_Ch_3);

  /* End of Terminate for SubSystem: '<S23>/Outputs to Amplifier' */
}
