/*
 * Ball_and_Plate_MicroLabBox_student.c
 *
 * Academic License - for use in teaching, academic research, and meeting
 * course requirements at degree granting institutions only.  Not for
 * government, commercial, or other organizational use.
 *
 * Code generation for model "Ball_and_Plate_MicroLabBox_student".
 *
 * Model version              : 1.147
 * Simulink Coder version : 9.2 (R2019b) 18-Jul-2019
 * C source code generated on : Thu Jun 25 16:40:41 2026
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

/* Named constants for MATLAB Function: '<S84>/optimizer' */
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
static boolean_T Ball_and_Plate_MicroL_vectorAny(const boolean_T x_data[], const
  int32_T *x_size);
static real_T Ball_and_Plate_MicroLabB_norm_m(const real_T x[16]);
static void Ball_and_Plate_Mi_Unconstrained(const real_T b_Hinv[49], const
  real_T f[7], real_T x[7], int16_T n);
static real_T Ball_and_Plate_MicroLabB_norm_g(const real_T x[7]);
static void Ball_and_Plate_MicroLabBox__abs(const real_T x[7], real_T y[7]);
static void Ball_and_Plate_MicroLabBo_abs_o(const real_T x[212], real_T y[212]);
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

/* Function for MATLAB Function: '<S19>/MATLAB Function1' */
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
 *    '<S19>/MATLAB Function1'
 *    '<S20>/MATLAB Function2'
 *    '<S21>/MATLAB Function1'
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
 *    '<S19>/MATLAB Function1'
 *    '<S20>/MATLAB Function2'
 *    '<S21>/MATLAB Function1'
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

  /* MATLAB Function 'Innerloop_Actuator/Motor_A/MATLAB Function1': '<S23>:1' */
  /* '<S23>:1:36' */
  /* '<S23>:1:37' */
  /* '<S23>:1:38' */
  if (!localDW->prev_ref_end_not_empty) {
    /* '<S23>:1:13' */
    /* '<S23>:1:14' */
    localDW->prev_ref_end = rtu_reference_end;
    localDW->prev_ref_end_not_empty = true;
  }

  if (rtu_reference_end != localDW->prev_ref_end) {
    /* '<S23>:1:21' */
    /* '<S23>:1:23' */
    localDW->t_elapsed = 0.0;

    /* '<S23>:1:24' */
    localDW->tf_internal = rtu_end_time;

    /* '<S23>:1:36' */
    /* '<S23>:1:43' */
    /* '<S23>:1:46' */
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
    /* '<S23>:1:52' */
    /* '<S23>:1:54' */
    path = ((((localDW->coeffs[1] * localDW->t_elapsed + localDW->coeffs[0]) +
              localDW->t_elapsed * localDW->t_elapsed * localDW->coeffs[2]) +
             localDW->coeffs[3] * rt_powd_snf(localDW->t_elapsed, 3.0)) +
            localDW->coeffs[4] * rt_powd_snf(localDW->t_elapsed, 4.0)) +
      localDW->coeffs[5] * rt_powd_snf(localDW->t_elapsed, 5.0);

    /* '<S23>:1:57' */
    localDW->t_elapsed += rtu_Ts;
  } else {
    /* '<S23>:1:60' */
    path = rtu_reference_end;
  }

  /* '<S23>:1:64' */
  localDW->prev_ref_end = rtu_reference_end;
  localB->path = path;
}

/*
 * System initialize for atomic system:
 *    '<S24>/MATLAB Function'
 *    '<S37>/MATLAB Function'
 *    '<S51>/MATLAB Function'
 *    '<S52>/MATLAB Function'
 */
void Ball_and_Pl_MATLABFunction_Init(DW_MATLABFunction_Ball_and_Pl_T *localDW)
{
  localDW->index = 1.0;
  localDW->previous_enable = 0.0;
}

/*
 * Output and update for atomic system:
 *    '<S24>/MATLAB Function'
 *    '<S37>/MATLAB Function'
 *    '<S51>/MATLAB Function'
 *    '<S52>/MATLAB Function'
 */
void Ball_and_Plate_M_MATLABFunction(real_T rtu_enable, const real_T rtu_u
  [200000], B_MATLABFunction_Ball_and_Pla_T *localB,
  DW_MATLABFunction_Ball_and_Pl_T *localDW)
{
  /* MATLAB Function 'Innerloop_Actuator/Motor_A/Multisine/MATLAB Function': '<S28>:1' */
  /* '<S28>:1:14' */
  if ((rtu_enable != 0.0) && (!(localDW->previous_enable != 0.0))) {
    /* '<S28>:1:17' */
    /* '<S28>:1:18' */
    localDW->index = 1.0;
  }

  if (rtu_enable != 0.0) {
    if (localDW->index <= 200000.0) {
      /* '<S28>:1:25' */
      /* '<S28>:1:26' */
      localB->y = rtu_u[(int32_T)localDW->index - 1];

      /* '<S28>:1:27' */
      localDW->index++;
    } else {
      /* '<S28>:1:31' */
      localB->y = 0.0;
    }
  } else {
    /* '<S28>:1:36' */
    localB->y = 0.0;
  }

  /* '<S28>:1:39' */
  localDW->previous_enable = rtu_enable;
}

/*
 * System initialize for enable system:
 *    '<S32>/Enabled Subsystem'
 *    '<S47>/Enabled Subsystem'
 *    '<S61>/Enabled Subsystem'
 */
void Ball_and__EnabledSubsystem_Init(B_EnabledSubsystem_Ball_and_P_T *localB,
  P_EnabledSubsystem_Ball_and_P_T *localP)
{
  /* SystemInitialize for Outport: '<S33>/Out1' */
  localB->OutportBufferForOut1 = localP->Out1_Y0;
}

/*
 * Disable for enable system:
 *    '<S32>/Enabled Subsystem'
 *    '<S47>/Enabled Subsystem'
 *    '<S61>/Enabled Subsystem'
 */
void Ball_a_EnabledSubsystem_Disable(B_EnabledSubsystem_Ball_and_P_T *localB,
  DW_EnabledSubsystem_Ball_and__T *localDW, P_EnabledSubsystem_Ball_and_P_T
  *localP)
{
  /* Disable for Outport: '<S33>/Out1' */
  localB->OutportBufferForOut1 = localP->Out1_Y0;
  localDW->EnabledSubsystem_MODE = false;
}

/*
 * Start for enable system:
 *    '<S32>/Enabled Subsystem'
 *    '<S47>/Enabled Subsystem'
 *    '<S61>/Enabled Subsystem'
 */
void Ball_and_EnabledSubsystem_Start(DW_EnabledSubsystem_Ball_and__T *localDW)
{
  localDW->EnabledSubsystem_MODE = false;
}

/*
 * Output and update for enable system:
 *    '<S32>/Enabled Subsystem'
 *    '<S47>/Enabled Subsystem'
 *    '<S61>/Enabled Subsystem'
 */
void Ball_and_Plate_EnabledSubsystem
  (RT_MODEL_Ball_and_Plate_MicroLabBox_student_T * const
   Ball_and_Plate_MicroLabBox_student_M, real_T rtu_Enable,
   B_EnabledSubsystem_Ball_and_P_T *localB, DW_EnabledSubsystem_Ball_and__T
   *localDW, P_EnabledSubsystem_Ball_and_P_T *localP)
{
  /* Outputs for Enabled SubSystem: '<S32>/Enabled Subsystem' incorporates:
   *  EnablePort: '<S33>/Enable'
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
    /* SignalConversion generated from: '<S33>/Out1' incorporates:
     *  Constant: '<S33>/Constant'
     */
    localB->OutportBufferForOut1 = localP->Constant_Value;
  }

  /* End of Outputs for SubSystem: '<S32>/Enabled Subsystem' */
}

/*
 * System initialize for enable system:
 *    '<S32>/Enabled Subsystem1'
 *    '<S47>/Enabled Subsystem1'
 *    '<S61>/Enabled Subsystem1'
 */
void Ball_and_EnabledSubsystem1_Init(B_EnabledSubsystem1_Ball_and__T *localB,
  P_EnabledSubsystem1_Ball_and__T *localP)
{
  /* SystemInitialize for Outport: '<S34>/Out1' */
  localB->In1 = localP->Out1_Y0;
}

/*
 * Disable for enable system:
 *    '<S32>/Enabled Subsystem1'
 *    '<S47>/Enabled Subsystem1'
 *    '<S61>/Enabled Subsystem1'
 */
void Ball__EnabledSubsystem1_Disable(DW_EnabledSubsystem1_Ball_and_T *localDW)
{
  localDW->EnabledSubsystem1_MODE = false;
}

/*
 * Start for enable system:
 *    '<S32>/Enabled Subsystem1'
 *    '<S47>/Enabled Subsystem1'
 *    '<S61>/Enabled Subsystem1'
 */
void Ball_an_EnabledSubsystem1_Start(DW_EnabledSubsystem1_Ball_and_T *localDW)
{
  localDW->EnabledSubsystem1_MODE = false;
}

/*
 * Output and update for enable system:
 *    '<S32>/Enabled Subsystem1'
 *    '<S47>/Enabled Subsystem1'
 *    '<S61>/Enabled Subsystem1'
 */
void Ball_and_Plat_EnabledSubsystem1
  (RT_MODEL_Ball_and_Plate_MicroLabBox_student_T * const
   Ball_and_Plate_MicroLabBox_student_M, boolean_T rtu_Enable, real_T rtu_In1,
   B_EnabledSubsystem1_Ball_and__T *localB, DW_EnabledSubsystem1_Ball_and_T
   *localDW)
{
  /* Outputs for Enabled SubSystem: '<S32>/Enabled Subsystem1' incorporates:
   *  EnablePort: '<S34>/Enable'
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
    /* Inport: '<S34>/In1' */
    localB->In1 = rtu_In1;
  }

  /* End of Outputs for SubSystem: '<S32>/Enabled Subsystem1' */
}

real_T rt_urand_Upu32_Yd_f_pw_snf(uint32_T *u)
{
  uint32_T lo;
  uint32_T hi;

  /* Uniform random number generator (random number between 0 and 1)

     #define IA      16807                      magic multiplier = 7^5
     #define IM      2147483647                 modulus = 2^31-1
     #define IQ      127773                     IM div IA
     #define IR      2836                       IM modulo IA
     #define S       4.656612875245797e-10      reciprocal of 2^31-1
     test = IA * (seed % IQ) - IR * (seed/IQ)
     seed = test < 0 ? (test + IM) : test
     return (seed*S)
   */
  lo = *u % 127773U * 16807U;
  hi = *u / 127773U * 2836U;
  if (lo < hi) {
    *u = 2147483647U - (hi - lo);
  } else {
    *u = lo - hi;
  }

  return (real_T)*u * 4.6566128752457969E-10;
}

real_T rt_nrand_Upu32_Yd_f_pw_snf(uint32_T *u)
{
  real_T y;
  real_T sr;
  real_T si;

  /* Normal (Gaussian) random number generator */
  do {
    sr = 2.0 * rt_urand_Upu32_Yd_f_pw_snf(u) - 1.0;
    si = 2.0 * rt_urand_Upu32_Yd_f_pw_snf(u) - 1.0;
    si = sr * sr + si * si;
  } while (si > 1.0);

  y = sqrt(-2.0 * log(si) / si) * sr;
  return y;
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

/* Function for MATLAB Function: '<S9>/MATLAB Function1' */
static boolean_T Ball_and_Plate_MicroL_vectorAny(const boolean_T x_data[], const
  int32_T *x_size)
{
  boolean_T y;
  int32_T k;
  boolean_T exitg1;
  y = false;
  k = 0;
  exitg1 = false;
  while ((!exitg1) && (k <= *x_size - 1)) {
    if (!x_data[k]) {
      k++;
    } else {
      y = true;
      exitg1 = true;
    }
  }

  return y;
}

/* Function for MATLAB Function: '<S9>/MATLAB Function1' */
static real_T Ball_and_Plate_MicroLabB_norm_m(const real_T x[16])
{
  real_T y;
  real_T scale;
  real_T absxk;
  real_T t;
  int32_T k;
  y = 0.0;
  scale = 3.3121686421112381E-170;
  for (k = 0; k < 16; k++) {
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

/* Function for MATLAB Function: '<S84>/optimizer' */
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

/* Function for MATLAB Function: '<S84>/optimizer' */
static real_T Ball_and_Plate_MicroLabB_norm_g(const real_T x[7])
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

/* Function for MATLAB Function: '<S84>/optimizer' */
static void Ball_and_Plate_MicroLabBox__abs(const real_T x[7], real_T y[7])
{
  int32_T k;
  for (k = 0; k < 7; k++) {
    y[k] = fabs(x[k]);
  }
}

/* Function for MATLAB Function: '<S84>/optimizer' */
static void Ball_and_Plate_MicroLabBo_abs_o(const real_T x[212], real_T y[212])
{
  int32_T k;
  for (k = 0; k < 212; k++) {
    y[k] = fabs(x[k]);
  }
}

/* Function for MATLAB Function: '<S84>/optimizer' */
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

/* Function for MATLAB Function: '<S84>/optimizer' */
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

/* Function for MATLAB Function: '<S84>/optimizer' */
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

/* Function for MATLAB Function: '<S84>/optimizer' */
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

/* Function for MATLAB Function: '<S84>/optimizer' */
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

/* Function for MATLAB Function: '<S84>/optimizer' */
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

/* Function for MATLAB Function: '<S84>/optimizer' */
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
    Xnorm0 = Ball_and_Plate_MicroLabB_norm_g(x);
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
              cMin = Ball_and_Plate_MicroLabB_norm_g(x);
              if (fabs(cMin - Xnorm0) > 0.001) {
                Xnorm0 = cMin;
                Ball_and_Plate_MicroLabBo_abs_o(b, varargin_1);
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

/* Function for MATLAB Function: '<S84>/optimizer' */
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
  real_T Ad[16];
  real_T x_pred[4];
  real_T P_pred[16];
  real_T S[4];
  real_T K[8];
  real_T y[8];
  int32_T r1;
  int32_T r2;
  real_T a21;
  real_T u_min[2];
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
  boolean_T tmp_data[16];
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
  real_T tmp_8[2];
  real_T center_idx_2;
  real_T uX_idx_2;
  real_T uX_idx_1;
  real_T uX_idx_0;
  static const int8_T e_0[3] = { 1, 0, 0 };

  static const real_T T_1[3] = { 0.0, 0.0, 0.32 };

  static const real_T b[3] = { 0.17, 0.0, 0.0 };

  static const real_T c[3] = { -0.085000000000000075, -0.14722431864335456, 0.0
  };

  static const real_T d[3] = { -0.084999999999999964, 0.14722431864335458, 0.0 };

  static const int8_T b_b[8] = { 1, 0, 0, 0, 0, 0, 1, 0 };

  static const int8_T a[8] = { 1, 0, 0, 0, 0, 1, 0, 0 };

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

  static const real_T b_Linv[49] = { 0.0018087922683098225, 0.0,
    -0.0078398020639469969, 0.0, -0.0011794930218640954, 0.0, 0.0, 0.0,
    0.0018087922683098225, 0.0, -0.0078398020639469969, 0.0,
    -0.0011794930218640954, 0.0, 0.0, 0.0, 0.0083066719359362745, 0.0,
    -0.0067348958730952006, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0083066719359362745, 0.0,
    -0.0067348958730952006, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0084059855524197084, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0084059855524197084, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.003162277660168379 };

  static const real_T b_Hinv[49] = { 6.6125429660391082E-5, 0.0,
    -5.7178901102586727E-5, 0.0, -9.9148013009694491E-6, 0.0, 0.0, 0.0,
    6.6125429660391082E-5, 0.0, -5.7178901102586727E-5, 0.0,
    -9.9148013009694491E-6, 0.0, -5.7178901102586727E-5, 0.0,
    0.00011435962107270606, 0.0, -5.6613437406289372E-5, 0.0, 0.0, 0.0,
    -5.7178901102586727E-5, 0.0, 0.00011435962107270606, 0.0,
    -5.6613437406289372E-5, 0.0, -9.9148013009694491E-6, 0.0,
    -5.6613437406289372E-5, 0.0, 7.0660593107488867E-5, 0.0, 0.0, 0.0,
    -9.9148013009694491E-6, 0.0, -5.6613437406289372E-5, 0.0,
    7.0660593107488867E-5, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
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
    /* DataTypeConversion: '<S2>/Data Type Conversion' incorporates:
     *  Constant: '<S2>/Constant'
     */
    Ball_and_Plate_MicroLabBox_student_B.DataTypeConversion =
      (Ball_and_Plate_MicroLabBox_student_P.Constant_Value_i != 0.0);

    /* S-Function (rti_commonblock): '<S16>/S-Function1' incorporates:
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

  /* Outputs for Atomic SubSystem: '<S19>/Position Measurement' */
  if (rtmIsMajorTimeStep(Ball_and_Plate_MicroLabBox_student_M) &&
      Ball_and_Plate_MicroLabBox_student_M->Timing.TaskCounters.TID[1] == 0) {
    /* Outputs for Triggered SubSystem: '<S26>/EMC_ENCODER_POS_SET_BL1' incorporates:
     *  TriggerPort: '<S31>/Trigger'
     */
    if (rtmIsMajorTimeStep(Ball_and_Plate_MicroLabBox_student_M)) {
      /* Constant: '<S3>/Init  ' */
      zcEvent = rt_ZCFcn(RISING_ZERO_CROSSING,
                         &Ball_and_Plate_MicroLabBox_student_PrevZCX.EMC_ENCODER_POS_SET_BL1_Trig_fh,
                         (Ball_and_Plate_MicroLabBox_student_P.Init_Value));
      if (zcEvent != NO_ZCEVENT) {
        /* S-Function (rti_commonblock): '<S31>/S-Function1' incorporates:
         *  Constant: '<S26>/Constant'
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

    /* End of Outputs for SubSystem: '<S26>/EMC_ENCODER_POS_SET_BL1' */

    /* S-Function (rti_commonblock): '<S30>/S-Function1' */
    /* This comment workarounds a code generation problem */

    /* Gain: '<S26>/Inc2Pos' */
    Ball_and_Plate_MicroLabBox_student_B.Inc2Pos_p =
      Ball_and_Plate_MicroLabBox_student_P.Inc2Pos_Gain *
      Ball_and_Plate_MicroLabBox_student_B.SFunction1_o1_p;

    /* Sum: '<S26>/AbsPosition' incorporates:
     *  Constant: '<S26>/Pos_offset'
     *  Constant: '<S26>/ZP'
     */
    Ball_and_Plate_MicroLabBox_student_B.AbsPosition_m =
      (Ball_and_Plate_MicroLabBox_student_B.Inc2Pos_p -
       Ball_and_Plate_MicroLabBox_student_P.ZP_Value) -
      Ball_and_Plate_MicroLabBox_student_P.Pos_offset_Value;

    /* Abs: '<S32>/Abs' */
    Ball_and_Plate_MicroLabBox_student_B.Abs_h = fabs
      (Ball_and_Plate_MicroLabBox_student_B.SFunction1_o2_d);

    /* Outputs for Enabled SubSystem: '<S32>/Enabled Subsystem' */
    /* Constant: '<S3>/Init  ' */
    Ball_and_Plate_EnabledSubsystem(Ball_and_Plate_MicroLabBox_student_M,
      Ball_and_Plate_MicroLabBox_student_P.Init_Value,
      &Ball_and_Plate_MicroLabBox_student_B.EnabledSubsystem,
      &Ball_and_Plate_MicroLabBox_student_DW.EnabledSubsystem,
      &Ball_and_Plate_MicroLabBox_student_P.EnabledSubsystem);

    /* End of Outputs for SubSystem: '<S32>/Enabled Subsystem' */

    /* RelationalOperator: '<S32>/Relational Operator1' incorporates:
     *  Constant: '<S32>/Const'
     */
    Ball_and_Plate_MicroLabBox_student_B.RelationalOperator1_f =
      (Ball_and_Plate_MicroLabBox_student_B.Abs_h <=
       Ball_and_Plate_MicroLabBox_student_P.Const_Value);
  }

  /* TransferFcn: '<S32>/Transfer Fcn' */
  Ball_and_Plate_MicroLabBox_student_B.TransferFcn_f = 0.0;
  Ball_and_Plate_MicroLabBox_student_B.TransferFcn_f +=
    Ball_and_Plate_MicroLabBox_student_P.TransferFcn_C *
    Ball_and_Plate_MicroLabBox_student_X.TransferFcn_CSTATE_l;

  /* RelationalOperator: '<S32>/Relational Operator' incorporates:
   *  Constant: '<S32>/Const1'
   */
  Ball_and_Plate_MicroLabBox_student_B.RelationalOperator_a =
    (Ball_and_Plate_MicroLabBox_student_P.Const1_Value >=
     Ball_and_Plate_MicroLabBox_student_B.TransferFcn_f);

  /* Logic: '<S32>/Logical Operator2' incorporates:
   *  Constant: '<S3>/Init  '
   */
  Ball_and_Plate_MicroLabBox_student_B.LogicalOperator2_a =
    (Ball_and_Plate_MicroLabBox_student_B.RelationalOperator1_f &&
     Ball_and_Plate_MicroLabBox_student_B.RelationalOperator_a &&
     (Ball_and_Plate_MicroLabBox_student_P.Init_Value != 0.0));

  /* Outputs for Enabled SubSystem: '<S32>/Enabled Subsystem1' */
  Ball_and_Plat_EnabledSubsystem1(Ball_and_Plate_MicroLabBox_student_M,
    Ball_and_Plate_MicroLabBox_student_B.LogicalOperator2_a,
    Ball_and_Plate_MicroLabBox_student_B.Inc2Pos_p,
    &Ball_and_Plate_MicroLabBox_student_B.EnabledSubsystem1,
    &Ball_and_Plate_MicroLabBox_student_DW.EnabledSubsystem1);

  /* End of Outputs for SubSystem: '<S32>/Enabled Subsystem1' */
  if (rtmIsMajorTimeStep(Ball_and_Plate_MicroLabBox_student_M) &&
      Ball_and_Plate_MicroLabBox_student_M->Timing.TaskCounters.TID[1] == 0) {
    /* Gain: '<S26>/mm2m' */
    Ball_and_Plate_MicroLabBox_student_B.mm2m_po =
      Ball_and_Plate_MicroLabBox_student_P.mm2m_Gain *
      Ball_and_Plate_MicroLabBox_student_B.AbsPosition_m;
  }

  /* End of Outputs for SubSystem: '<S19>/Position Measurement' */

  /* Outputs for Atomic SubSystem: '<S20>/Position Measurement' */
  if (rtmIsMajorTimeStep(Ball_and_Plate_MicroLabBox_student_M) &&
      Ball_and_Plate_MicroLabBox_student_M->Timing.TaskCounters.TID[1] == 0) {
    /* Outputs for Triggered SubSystem: '<S39>/EMC_ENCODER_POS_SET_BL1' incorporates:
     *  TriggerPort: '<S46>/Trigger'
     */
    if (rtmIsMajorTimeStep(Ball_and_Plate_MicroLabBox_student_M)) {
      /* Constant: '<S3>/Init  ' */
      zcEvent = rt_ZCFcn(RISING_ZERO_CROSSING,
                         &Ball_and_Plate_MicroLabBox_student_PrevZCX.EMC_ENCODER_POS_SET_BL1_Trig__f,
                         (Ball_and_Plate_MicroLabBox_student_P.Init_Value));
      if (zcEvent != NO_ZCEVENT) {
        /* S-Function (rti_commonblock): '<S46>/S-Function1' incorporates:
         *  Constant: '<S39>/Constant'
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

    /* End of Outputs for SubSystem: '<S39>/EMC_ENCODER_POS_SET_BL1' */

    /* S-Function (rti_commonblock): '<S45>/S-Function1' */
    /* This comment workarounds a code generation problem */

    /* Gain: '<S39>/Inc2Pos' */
    Ball_and_Plate_MicroLabBox_student_B.Inc2Pos_e =
      Ball_and_Plate_MicroLabBox_student_P.Inc2Pos_Gain_k *
      Ball_and_Plate_MicroLabBox_student_B.SFunction1_o1_i;

    /* Sum: '<S39>/AbsPosition' incorporates:
     *  Constant: '<S39>/Pos_offset'
     *  Constant: '<S39>/ZP'
     */
    Ball_and_Plate_MicroLabBox_student_B.AbsPosition_i =
      (Ball_and_Plate_MicroLabBox_student_B.Inc2Pos_e -
       Ball_and_Plate_MicroLabBox_student_P.ZP_Value_m) -
      Ball_and_Plate_MicroLabBox_student_P.Pos_offset_Value_k;

    /* Abs: '<S47>/Abs' */
    Ball_and_Plate_MicroLabBox_student_B.Abs_j = fabs
      (Ball_and_Plate_MicroLabBox_student_B.SFunction1_o2_p);

    /* Outputs for Enabled SubSystem: '<S47>/Enabled Subsystem' */
    /* Constant: '<S3>/Init  ' */
    Ball_and_Plate_EnabledSubsystem(Ball_and_Plate_MicroLabBox_student_M,
      Ball_and_Plate_MicroLabBox_student_P.Init_Value,
      &Ball_and_Plate_MicroLabBox_student_B.EnabledSubsystem_a,
      &Ball_and_Plate_MicroLabBox_student_DW.EnabledSubsystem_a,
      &Ball_and_Plate_MicroLabBox_student_P.EnabledSubsystem_a);

    /* End of Outputs for SubSystem: '<S47>/Enabled Subsystem' */

    /* RelationalOperator: '<S47>/Relational Operator1' incorporates:
     *  Constant: '<S47>/Const'
     */
    Ball_and_Plate_MicroLabBox_student_B.RelationalOperator1_c =
      (Ball_and_Plate_MicroLabBox_student_B.Abs_j <=
       Ball_and_Plate_MicroLabBox_student_P.Const_Value_g);
  }

  /* TransferFcn: '<S47>/Transfer Fcn' */
  Ball_and_Plate_MicroLabBox_student_B.TransferFcn_o = 0.0;
  Ball_and_Plate_MicroLabBox_student_B.TransferFcn_o +=
    Ball_and_Plate_MicroLabBox_student_P.TransferFcn_C_e *
    Ball_and_Plate_MicroLabBox_student_X.TransferFcn_CSTATE_h;

  /* RelationalOperator: '<S47>/Relational Operator' incorporates:
   *  Constant: '<S47>/Const1'
   */
  Ball_and_Plate_MicroLabBox_student_B.RelationalOperator_n =
    (Ball_and_Plate_MicroLabBox_student_P.Const1_Value_a >=
     Ball_and_Plate_MicroLabBox_student_B.TransferFcn_o);

  /* Logic: '<S47>/Logical Operator2' incorporates:
   *  Constant: '<S3>/Init  '
   */
  Ball_and_Plate_MicroLabBox_student_B.LogicalOperator2_l =
    (Ball_and_Plate_MicroLabBox_student_B.RelationalOperator1_c &&
     Ball_and_Plate_MicroLabBox_student_B.RelationalOperator_n &&
     (Ball_and_Plate_MicroLabBox_student_P.Init_Value != 0.0));

  /* Outputs for Enabled SubSystem: '<S47>/Enabled Subsystem1' */
  Ball_and_Plat_EnabledSubsystem1(Ball_and_Plate_MicroLabBox_student_M,
    Ball_and_Plate_MicroLabBox_student_B.LogicalOperator2_l,
    Ball_and_Plate_MicroLabBox_student_B.Inc2Pos_e,
    &Ball_and_Plate_MicroLabBox_student_B.EnabledSubsystem1_f,
    &Ball_and_Plate_MicroLabBox_student_DW.EnabledSubsystem1_f);

  /* End of Outputs for SubSystem: '<S47>/Enabled Subsystem1' */
  if (rtmIsMajorTimeStep(Ball_and_Plate_MicroLabBox_student_M) &&
      Ball_and_Plate_MicroLabBox_student_M->Timing.TaskCounters.TID[1] == 0) {
    /* Gain: '<S39>/mm2m' */
    Ball_and_Plate_MicroLabBox_student_B.mm2m_p =
      Ball_and_Plate_MicroLabBox_student_P.mm2m_Gain_f *
      Ball_and_Plate_MicroLabBox_student_B.AbsPosition_i;
  }

  /* End of Outputs for SubSystem: '<S20>/Position Measurement' */

  /* Outputs for Atomic SubSystem: '<S21>/Position Measurement' */
  if (rtmIsMajorTimeStep(Ball_and_Plate_MicroLabBox_student_M) &&
      Ball_and_Plate_MicroLabBox_student_M->Timing.TaskCounters.TID[1] == 0) {
    /* Outputs for Triggered SubSystem: '<S54>/EMC_ENCODER_POS_SET_BL1' incorporates:
     *  TriggerPort: '<S60>/Trigger'
     */
    if (rtmIsMajorTimeStep(Ball_and_Plate_MicroLabBox_student_M)) {
      /* Constant: '<S3>/Init  ' */
      zcEvent = rt_ZCFcn(RISING_ZERO_CROSSING,
                         &Ball_and_Plate_MicroLabBox_student_PrevZCX.EMC_ENCODER_POS_SET_BL1_Trig_ZC,
                         (Ball_and_Plate_MicroLabBox_student_P.Init_Value));
      if (zcEvent != NO_ZCEVENT) {
        /* S-Function (rti_commonblock): '<S60>/S-Function1' incorporates:
         *  Constant: '<S54>/Constant'
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

    /* End of Outputs for SubSystem: '<S54>/EMC_ENCODER_POS_SET_BL1' */

    /* S-Function (rti_commonblock): '<S59>/S-Function1' */
    /* This comment workarounds a code generation problem */

    /* Gain: '<S54>/Inc2Pos' */
    Ball_and_Plate_MicroLabBox_student_B.Inc2Pos =
      Ball_and_Plate_MicroLabBox_student_P.Inc2Pos_Gain_h *
      Ball_and_Plate_MicroLabBox_student_B.SFunction1_o1;

    /* Sum: '<S54>/AbsPosition' incorporates:
     *  Constant: '<S54>/Pos_offset'
     *  Constant: '<S54>/ZP'
     */
    Ball_and_Plate_MicroLabBox_student_B.AbsPosition =
      (Ball_and_Plate_MicroLabBox_student_B.Inc2Pos -
       Ball_and_Plate_MicroLabBox_student_P.ZP_Value_p) -
      Ball_and_Plate_MicroLabBox_student_P.Pos_offset_Value_f;

    /* Abs: '<S61>/Abs' */
    Ball_and_Plate_MicroLabBox_student_B.Abs = fabs
      (Ball_and_Plate_MicroLabBox_student_B.SFunction1_o2);

    /* Outputs for Enabled SubSystem: '<S61>/Enabled Subsystem' */
    /* Constant: '<S3>/Init  ' */
    Ball_and_Plate_EnabledSubsystem(Ball_and_Plate_MicroLabBox_student_M,
      Ball_and_Plate_MicroLabBox_student_P.Init_Value,
      &Ball_and_Plate_MicroLabBox_student_B.EnabledSubsystem_n,
      &Ball_and_Plate_MicroLabBox_student_DW.EnabledSubsystem_n,
      &Ball_and_Plate_MicroLabBox_student_P.EnabledSubsystem_n);

    /* End of Outputs for SubSystem: '<S61>/Enabled Subsystem' */

    /* RelationalOperator: '<S61>/Relational Operator1' incorporates:
     *  Constant: '<S61>/Const'
     */
    Ball_and_Plate_MicroLabBox_student_B.RelationalOperator1 =
      (Ball_and_Plate_MicroLabBox_student_B.Abs <=
       Ball_and_Plate_MicroLabBox_student_P.Const_Value_m);
  }

  /* TransferFcn: '<S61>/Transfer Fcn' */
  Ball_and_Plate_MicroLabBox_student_B.TransferFcn = 0.0;
  Ball_and_Plate_MicroLabBox_student_B.TransferFcn +=
    Ball_and_Plate_MicroLabBox_student_P.TransferFcn_C_c *
    Ball_and_Plate_MicroLabBox_student_X.TransferFcn_CSTATE;

  /* RelationalOperator: '<S61>/Relational Operator' incorporates:
   *  Constant: '<S61>/Const1'
   */
  Ball_and_Plate_MicroLabBox_student_B.RelationalOperator =
    (Ball_and_Plate_MicroLabBox_student_P.Const1_Value_p >=
     Ball_and_Plate_MicroLabBox_student_B.TransferFcn);

  /* Logic: '<S61>/Logical Operator2' incorporates:
   *  Constant: '<S3>/Init  '
   */
  Ball_and_Plate_MicroLabBox_student_B.LogicalOperator2 =
    (Ball_and_Plate_MicroLabBox_student_B.RelationalOperator1 &&
     Ball_and_Plate_MicroLabBox_student_B.RelationalOperator &&
     (Ball_and_Plate_MicroLabBox_student_P.Init_Value != 0.0));

  /* Outputs for Enabled SubSystem: '<S61>/Enabled Subsystem1' */
  Ball_and_Plat_EnabledSubsystem1(Ball_and_Plate_MicroLabBox_student_M,
    Ball_and_Plate_MicroLabBox_student_B.LogicalOperator2,
    Ball_and_Plate_MicroLabBox_student_B.Inc2Pos,
    &Ball_and_Plate_MicroLabBox_student_B.EnabledSubsystem1_b,
    &Ball_and_Plate_MicroLabBox_student_DW.EnabledSubsystem1_b);

  /* End of Outputs for SubSystem: '<S61>/Enabled Subsystem1' */
  if (rtmIsMajorTimeStep(Ball_and_Plate_MicroLabBox_student_M) &&
      Ball_and_Plate_MicroLabBox_student_M->Timing.TaskCounters.TID[1] == 0) {
    /* Gain: '<S54>/mm2m' */
    Ball_and_Plate_MicroLabBox_student_B.mm2m =
      Ball_and_Plate_MicroLabBox_student_P.mm2m_Gain_o *
      Ball_and_Plate_MicroLabBox_student_B.AbsPosition;
  }

  /* End of Outputs for SubSystem: '<S21>/Position Measurement' */
  if (rtmIsMajorTimeStep(Ball_and_Plate_MicroLabBox_student_M) &&
      Ball_and_Plate_MicroLabBox_student_M->Timing.TaskCounters.TID[1] == 0) {
    /* MATLAB Function: '<S3>/PosToAngle ' */
    /* MATLAB Function 'Innerloop_Actuator/PosToAngle ': '<S22>:1' */
    /* '<S22>:1:6' */
    /* '<S22>:1:7' */
    /* '<S22>:1:13' */
    P1_global[2] = Ball_and_Plate_MicroLabBox_student_B.mm2m_po + 0.322;

    /* '<S22>:1:14' */
    /* '<S22>:1:15' */
    a21 = Ball_and_Plate_MicroLabBox_student_B.mm2m + 0.322;

    /* '<S22>:1:19' */
    center_idx_2 = (((Ball_and_Plate_MicroLabBox_student_B.mm2m_p + 0.322) +
                     P1_global[2]) + a21) / 3.0;

    /* '<S22>:1:23' */
    /* '<S22>:1:25' */
    P1_global[0] = 0.16999999999999998;
    P1_global[1] = 0.0;
    uX_idx_0 = P1_global[2];
    uX_idx_0 -= center_idx_2;
    a21 -= center_idx_2;
    P1_global[2] = uX_idx_0;

    /* '<S22>:1:28' */
    center_idx_2 = Ball_and_Plate_MicroLabBox_norm(P1_global);
    uX_idx_0 = 0.16999999999999998 / center_idx_2;
    uX_idx_1 = 0.0 / center_idx_2;
    uX_idx_2 = P1_global[2] / center_idx_2;

    /* '<S22>:1:29' */
    P1_global[0] = uX_idx_1 * a21 - uX_idx_2 * -0.14722431864335458;
    P1_global[1] = uX_idx_2 * -0.084999999999999992 - uX_idx_0 * a21;
    P1_global[2] = uX_idx_0 * -0.14722431864335458 - uX_idx_1 *
      -0.084999999999999992;

    /* '<S22>:1:30' */
    center_idx_2 = Ball_and_Plate_MicroLabBox_norm(P1_global);
    a21 = P1_global[0] / center_idx_2;
    center_idx_2 = P1_global[1] / center_idx_2;

    /* '<S22>:1:31' */
    /* '<S22>:1:34' */
    Ball_and_Plate_MicroLabBox_student_B.beta = -uX_idx_2;
    Ball_and_Plate_MicroLabBox_student_B.beta = asin
      (Ball_and_Plate_MicroLabBox_student_B.beta);

    /* '<S22>:1:35' */
    Ball_and_Plate_MicroLabBox_student_B.alpha = (a21 * uX_idx_1 - center_idx_2 *
      uX_idx_0) / sqrt(1.0 - uX_idx_2 * uX_idx_2);
    Ball_and_Plate_MicroLabBox_student_B.alpha = asin
      (Ball_and_Plate_MicroLabBox_student_B.alpha);

    /* '<S22>:1:36' */
    Ball_and_Plate_MicroLabBox_student_B.psi = uX_idx_1 / sqrt(1.0 - uX_idx_2 *
      uX_idx_2);
    Ball_and_Plate_MicroLabBox_student_B.psi = asin
      (Ball_and_Plate_MicroLabBox_student_B.psi);

    /* S-Function (xpcbytepacking): '<S2>/Byte Unpacking ' */

    /* Byte Unpacking: <S2>/Byte Unpacking  */
    (void)memcpy((uint8_T*)
                 &Ball_and_Plate_MicroLabBox_student_B.ByteUnpacking_o1,
                 (uint8_T*)
                 &Ball_and_Plate_MicroLabBox_student_B.SFunction1_o1_c[0] + 0, 4);
    (void)memcpy((uint8_T*)
                 &Ball_and_Plate_MicroLabBox_student_B.ByteUnpacking_o2,
                 (uint8_T*)
                 &Ball_and_Plate_MicroLabBox_student_B.SFunction1_o1_c[0] + 4, 4);
    (void)memcpy((uint8_T*)
                 &Ball_and_Plate_MicroLabBox_student_B.ByteUnpacking_o3,
                 (uint8_T*)
                 &Ball_and_Plate_MicroLabBox_student_B.SFunction1_o1_c[0] + 8, 4);
    (void)memcpy((uint8_T*)
                 &Ball_and_Plate_MicroLabBox_student_B.ByteUnpacking_o4,
                 (uint8_T*)
                 &Ball_and_Plate_MicroLabBox_student_B.SFunction1_o1_c[0] + 12,
                 1);
    (void)memcpy((uint8_T*)
                 &Ball_and_Plate_MicroLabBox_student_B.ByteUnpacking_o5[0],
                 (uint8_T*)
                 &Ball_and_Plate_MicroLabBox_student_B.SFunction1_o1_c[0] + 13,
                 19);

    /* SignalConversion generated from: '<S4>/ SFunction ' incorporates:
     *  MATLAB Function: '<Root>/MATLAB Function'
     */
    Ball_and_Plate_MicroLabBox_student_B.TmpSignalConversionAtSFunctionI[0] =
      Ball_and_Plate_MicroLabBox_student_B.FirstOrderHold;
    Ball_and_Plate_MicroLabBox_student_B.TmpSignalConversionAtSFunctionI[1] =
      Ball_and_Plate_MicroLabBox_student_B.FirstOrderHold1;

    /* SignalConversion generated from: '<S4>/ SFunction ' incorporates:
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
    /* MATLAB Function 'MATLAB Function': '<S4>:1' */
    /* '<S4>:1:5' */
    /* '<S4>:1:7' */
    /* '<S4>:1:34' */
    /* '<S4>:1:3' */
    /* '<S4>:1:4' */
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

    /* '<S4>:1:9' */
    /* '<S4>:1:28' */
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

    /* '<S4>:1:29' */
    for (r1 = 0; r1 < 4; r1++) {
      a21 = Ad[r1] * Ball_and_Plate_MicroLabBox_student_DW.x_hat[0];
      a21 += Ad[r1 + 4] * Ball_and_Plate_MicroLabBox_student_DW.x_hat[1];
      a21 += Ad[r1 + 8] * Ball_and_Plate_MicroLabBox_student_DW.x_hat[2];
      a21 += Ad[r1 + 12] * Ball_and_Plate_MicroLabBox_student_DW.x_hat[3];
      center_idx_2 = y[r1] *
        Ball_and_Plate_MicroLabBox_student_B.TmpSignalConversionAtSFunctio_e[0];
      center_idx_2 += y[r1 + 4] *
        Ball_and_Plate_MicroLabBox_student_B.TmpSignalConversionAtSFunctio_e[1];
      x_pred[r1] = a21 + center_idx_2;
      for (r2 = 0; r2 < 4; r2++) {
        Ad_0[r1 + (r2 << 2)] = 0.0;
        a21 = Ad_0[(r2 << 2) + r1];
        a21 += Ball_and_Plate_MicroLabBox_student_DW.P[r2 << 2] * Ad[r1];
        Ad_0[r1 + (r2 << 2)] = a21;
        a21 = Ad_0[(r2 << 2) + r1];
        a21 += Ball_and_Plate_MicroLabBox_student_DW.P[(r2 << 2) + 1] * Ad[r1 +
          4];
        Ad_0[r1 + (r2 << 2)] = a21;
        a21 = Ad_0[(r2 << 2) + r1];
        a21 += Ball_and_Plate_MicroLabBox_student_DW.P[(r2 << 2) + 2] * Ad[r1 +
          8];
        Ad_0[r1 + (r2 << 2)] = a21;
        a21 = Ad_0[(r2 << 2) + r1];
        a21 += Ball_and_Plate_MicroLabBox_student_DW.P[(r2 << 2) + 3] * Ad[r1 +
          12];
        Ad_0[r1 + (r2 << 2)] = a21;
      }

      for (r2 = 0; r2 < 4; r2++) {
        center_idx_2 = Ad_0[r1] * Ad[r2];
        center_idx_2 += Ad_0[r1 + 4] * Ad[r2 + 4];
        center_idx_2 += Ad_0[r1 + 8] * Ad[r2 + 8];
        center_idx_2 += Ad_0[r1 + 12] * Ad[r2 + 12];
        P_pred[r1 + (r2 << 2)] = Ball_and_Plate_MicroLabBox_student_P.Q_kf[(r2 <<
          2) + r1] + center_idx_2;
      }
    }

    if (Ball_and_Plate_MicroLabBox_student_B.ByteUnpacking_o4 > 0.5) {
      /* '<S4>:1:32' */
      /* '<S4>:1:34' */
      /* '<S4>:1:35' */
      for (r1 = 0; r1 < 2; r1++) {
        for (r2 = 0; r2 < 4; r2++) {
          K[r1 + (r2 << 1)] = 0.0;
          center_idx_2 = K[(r2 << 1) + r1];
          center_idx_2 += P_pred[r2 << 2] * (real_T)a[r1];
          K[r1 + (r2 << 1)] = center_idx_2;
          center_idx_2 = K[(r2 << 1) + r1];
          center_idx_2 += P_pred[(r2 << 2) + 1] * 0.0;
          K[r1 + (r2 << 1)] = center_idx_2;
          center_idx_2 = K[(r2 << 1) + r1];
          center_idx_2 += P_pred[(r2 << 2) + 2] * (real_T)a[r1 + 4];
          K[r1 + (r2 << 1)] = center_idx_2;
          center_idx_2 = K[(r2 << 1) + r1];
          center_idx_2 += P_pred[(r2 << 2) + 3] * 0.0;
          K[r1 + (r2 << 1)] = center_idx_2;
        }

        for (r2 = 0; r2 < 2; r2++) {
          center_idx_2 = (real_T)b_b[r2 << 2] * K[r1];
          center_idx_2 += K[r1 + 2] * 0.0;
          center_idx_2 += (real_T)b_b[(r2 << 2) + 2] * K[r1 + 4];
          center_idx_2 += (real_T)b_b[(r2 << 2) + 3] * K[r1 + 6];
          S[r1 + (r2 << 1)] = Ball_and_Plate_MicroLabBox_student_P.R_kf[(r2 << 1)
            + r1] + center_idx_2;
        }

        for (r2 = 0; r2 < 4; r2++) {
          y[r2 + (r1 << 2)] = 0.0;
          center_idx_2 = y[(r1 << 2) + r2];
          center_idx_2 += (real_T)b_b[r1 << 2] * P_pred[r2];
          y[r2 + (r1 << 2)] = center_idx_2;
          center_idx_2 = y[(r1 << 2) + r2];
          center_idx_2 += P_pred[r2 + 4] * 0.0;
          y[r2 + (r1 << 2)] = center_idx_2;
          center_idx_2 = y[(r1 << 2) + r2];
          center_idx_2 += (real_T)b_b[(r1 << 2) + 2] * P_pred[r2 + 8];
          y[r2 + (r1 << 2)] = center_idx_2;
          center_idx_2 = y[(r1 << 2) + r2];
          center_idx_2 += (real_T)b_b[(r1 << 2) + 3] * P_pred[r2 + 12];
          y[r2 + (r1 << 2)] = center_idx_2;
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
      center_idx_2 = S[r2 + 2] - S[r1 + 2] * a21;
      K[r1 << 2] = y[0] / S[r1];
      K[r2 << 2] = (y[4] - K[r1 << 2] * S[r1 + 2]) / center_idx_2;
      K[r1 << 2] -= K[r2 << 2] * a21;
      K[(r1 << 2) + 1] = y[1] / S[r1];
      K[(r2 << 2) + 1] = (y[5] - K[(r1 << 2) + 1] * S[r1 + 2]) / center_idx_2;
      K[(r1 << 2) + 1] -= K[(r2 << 2) + 1] * a21;
      K[(r1 << 2) + 2] = y[2] / S[r1];
      K[(r2 << 2) + 2] = (y[6] - K[(r1 << 2) + 2] * S[r1 + 2]) / center_idx_2;
      K[(r1 << 2) + 2] -= K[(r2 << 2) + 2] * a21;
      K[(r1 << 2) + 3] = y[3] / S[r1];
      K[(r2 << 2) + 3] = (y[7] - K[(r1 << 2) + 3] * S[r1 + 2]) / center_idx_2;
      K[(r1 << 2) + 3] -= K[(r2 << 2) + 3] * a21;

      /* '<S4>:1:37' */
      for (r1 = 0; r1 < 2; r1++) {
        center_idx_2 = (real_T)a[r1] * x_pred[0];
        center_idx_2 += 0.0 * x_pred[1];
        center_idx_2 += (real_T)a[r1 + 4] * x_pred[2];
        center_idx_2 += 0.0 * x_pred[3];
        tmp_7[r1] =
          Ball_and_Plate_MicroLabBox_student_B.TmpSignalConversionAtSFunctionI[r1]
          - center_idx_2;
      }

      for (r1 = 0; r1 < 4; r1++) {
        center_idx_2 = K[r1] * tmp_7[0];
        center_idx_2 += K[r1 + 4] * tmp_7[1];
        Ball_and_Plate_MicroLabBox_student_DW.x_hat[r1] = x_pred[r1] +
          center_idx_2;
      }

      /* '<S4>:1:38' */
      memset(&Ad[0], 0, sizeof(real_T) << 4U);
      Ad[0] = 1.0;
      Ad[5] = 1.0;
      Ad[10] = 1.0;
      Ad[15] = 1.0;
      for (r1 = 0; r1 < 4; r1++) {
        for (r2 = 0; r2 < 4; r2++) {
          center_idx_2 = (real_T)a[r2 << 1] * K[r1];
          center_idx_2 += (real_T)a[(r2 << 1) + 1] * K[r1 + 4];
          Ad_0[r1 + (r2 << 2)] = Ad[(r2 << 2) + r1] - center_idx_2;
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
      /* '<S4>:1:41' */
      Ball_and_Plate_MicroLabBox_student_DW.x_hat[0] = x_pred[0];
      Ball_and_Plate_MicroLabBox_student_DW.x_hat[1] = x_pred[1];
      Ball_and_Plate_MicroLabBox_student_DW.x_hat[2] = x_pred[2];
      Ball_and_Plate_MicroLabBox_student_DW.x_hat[3] = x_pred[3];

      /* '<S4>:1:42' */
      memcpy(&Ball_and_Plate_MicroLabBox_student_DW.P[0], &P_pred[0], sizeof
             (real_T) << 4U);
    }

    /* '<S4>:1:46' */
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
    center_idx_2 = Ball_and_Plate_MicroLabBox_student_B.x_est[0];
    uX_idx_0 = Ball_and_Plate_MicroLabBox_student_P.ball_pos_sat;
    if (center_idx_2 > uX_idx_0) {
      /* Outport: '<Root>/x_obs' */
      Ball_and_Plate_MicroLabBox_student_Y.x_obs = uX_idx_0;
    } else if (center_idx_2 < a21) {
      /* Outport: '<Root>/x_obs' */
      Ball_and_Plate_MicroLabBox_student_Y.x_obs = a21;
    } else {
      /* Outport: '<Root>/x_obs' */
      Ball_and_Plate_MicroLabBox_student_Y.x_obs = center_idx_2;
    }

    /* End of Saturate: '<Root>/Saturation2' */

    /* Saturate: '<Root>/Saturation1' */
    a21 = -Ball_and_Plate_MicroLabBox_student_P.ball_vel_sat;
    center_idx_2 = Ball_and_Plate_MicroLabBox_student_B.x_est[1];
    uX_idx_0 = Ball_and_Plate_MicroLabBox_student_P.ball_vel_sat;
    if (center_idx_2 > uX_idx_0) {
      /* Outport: '<Root>/x_dot_obs' */
      Ball_and_Plate_MicroLabBox_student_Y.x_dot_obs = uX_idx_0;
    } else if (center_idx_2 < a21) {
      /* Outport: '<Root>/x_dot_obs' */
      Ball_and_Plate_MicroLabBox_student_Y.x_dot_obs = a21;
    } else {
      /* Outport: '<Root>/x_dot_obs' */
      Ball_and_Plate_MicroLabBox_student_Y.x_dot_obs = center_idx_2;
    }

    /* End of Saturate: '<Root>/Saturation1' */

    /* Saturate: '<Root>/Saturation3' */
    a21 = -Ball_and_Plate_MicroLabBox_student_P.ball_pos_sat;
    center_idx_2 = Ball_and_Plate_MicroLabBox_student_B.x_est[2];
    uX_idx_0 = Ball_and_Plate_MicroLabBox_student_P.ball_pos_sat;
    if (center_idx_2 > uX_idx_0) {
      /* Outport: '<Root>/y_obs' */
      Ball_and_Plate_MicroLabBox_student_Y.y_obs = uX_idx_0;
    } else if (center_idx_2 < a21) {
      /* Outport: '<Root>/y_obs' */
      Ball_and_Plate_MicroLabBox_student_Y.y_obs = a21;
    } else {
      /* Outport: '<Root>/y_obs' */
      Ball_and_Plate_MicroLabBox_student_Y.y_obs = center_idx_2;
    }

    /* End of Saturate: '<Root>/Saturation3' */

    /* Saturate: '<Root>/Saturation' */
    a21 = -Ball_and_Plate_MicroLabBox_student_P.ball_vel_sat;
    center_idx_2 = Ball_and_Plate_MicroLabBox_student_B.x_est[3];
    uX_idx_0 = Ball_and_Plate_MicroLabBox_student_P.ball_vel_sat;
    if (center_idx_2 > uX_idx_0) {
      /* Outport: '<Root>/y_dot_obs' */
      Ball_and_Plate_MicroLabBox_student_Y.y_dot_obs = uX_idx_0;
    } else if (center_idx_2 < a21) {
      /* Outport: '<Root>/y_dot_obs' */
      Ball_and_Plate_MicroLabBox_student_Y.y_dot_obs = a21;
    } else {
      /* Outport: '<Root>/y_dot_obs' */
      Ball_and_Plate_MicroLabBox_student_Y.y_dot_obs = center_idx_2;
    }

    /* End of Saturate: '<Root>/Saturation' */

    /* Switch: '<Root>/Switch' incorporates:
     *  Constant: '<Root>/Constant3'
     *  Constant: '<Root>/enable_outer_controller'
     *  Outport: '<Root>/x_dot_obs'
     *  Outport: '<Root>/x_obs'
     *  Outport: '<Root>/y_dot_obs'
     *  Outport: '<Root>/y_obs'
     */
    if (Ball_and_Plate_MicroLabBox_student_P.enable_outer_controller_Value >
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

    /* MATLAB Function: '<Root>/circ_ref_player ' incorporates:
     *  Constant: '<Root>/Constant5'
     *  Constant: '<Root>/Constant6'
     *  Constant: '<Root>/enable_circ_ref  '
     */
    /* MATLAB Function 'circ_ref_player ': '<S10>:1' */
    /* '<S10>:1:14' */
    if (Ball_and_Plate_MicroLabBox_student_P.enable_circ_ref_Value == 1.0) {
      /* '<S10>:1:23' */
      if (Ball_and_Plate_MicroLabBox_student_DW.idx_ref_a <= 56001.0) {
        /* '<S10>:1:25' */
        /* '<S10>:1:26' */
        r1 = (int32_T)Ball_and_Plate_MicroLabBox_student_DW.idx_ref_a;
        Ball_and_Plate_MicroLabBox_student_B.rsample_n[0] =
          Ball_and_Plate_MicroLabBox_student_P.r_circ[(r1 - 1) << 2];
        Ball_and_Plate_MicroLabBox_student_B.rsample_n[1] =
          Ball_and_Plate_MicroLabBox_student_P.r_circ[((r1 - 1) << 2) + 1];
        Ball_and_Plate_MicroLabBox_student_B.rsample_n[2] =
          Ball_and_Plate_MicroLabBox_student_P.r_circ[((r1 - 1) << 2) + 2];
        Ball_and_Plate_MicroLabBox_student_B.rsample_n[3] =
          Ball_and_Plate_MicroLabBox_student_P.r_circ[((r1 - 1) << 2) + 3];

        /* '<S10>:1:27' */
        r1 = (int32_T)Ball_and_Plate_MicroLabBox_student_DW.idx_ref_a;
        Ball_and_Plate_MicroLabBox_student_B.asample_h[0] =
          Ball_and_Plate_MicroLabBox_student_P.a_circ[(r1 - 1) << 1];
        Ball_and_Plate_MicroLabBox_student_B.asample_h[1] =
          Ball_and_Plate_MicroLabBox_student_P.a_circ[((r1 - 1) << 1) + 1];
      } else {
        /* '<S10>:1:30' */
        Ball_and_Plate_MicroLabBox_student_B.rsample_n[0] =
          Ball_and_Plate_MicroLabBox_student_P.r_circ[224000];
        Ball_and_Plate_MicroLabBox_student_B.rsample_n[1] =
          Ball_and_Plate_MicroLabBox_student_P.r_circ[224001];
        Ball_and_Plate_MicroLabBox_student_B.rsample_n[2] =
          Ball_and_Plate_MicroLabBox_student_P.r_circ[224002];
        Ball_and_Plate_MicroLabBox_student_B.rsample_n[3] =
          Ball_and_Plate_MicroLabBox_student_P.r_circ[224003];

        /* '<S10>:1:31' */
        Ball_and_Plate_MicroLabBox_student_B.asample_h[0] =
          Ball_and_Plate_MicroLabBox_student_P.a_circ[112000];
        Ball_and_Plate_MicroLabBox_student_B.asample_h[1] =
          Ball_and_Plate_MicroLabBox_student_P.a_circ[112001];
      }

      /* '<S10>:1:1' */
      /* '<S10>:1:35' */
      for (r1 = 0; r1 < 25; r1++) {
        /* '<S10>:1:35' */
        /* '<S10>:1:36' */
        a21 = ((real_T)r1 + 1.0) +
          Ball_and_Plate_MicroLabBox_student_DW.idx_ref_a;
        if (a21 <= 56001.0) {
          /* '<S10>:1:38' */
          /* '<S10>:1:40' */
          r2 = (int32_T)a21;
          Ball_and_Plate_MicroLabBox_student_B.rpreview_a[r1] =
            Ball_and_Plate_MicroLabBox_student_P.r_circ[(r2 - 1) << 2];
          Ball_and_Plate_MicroLabBox_student_B.rpreview_a[r1 + 25] =
            Ball_and_Plate_MicroLabBox_student_P.r_circ[((r2 - 1) << 2) + 1];
          Ball_and_Plate_MicroLabBox_student_B.rpreview_a[r1 + 50] =
            Ball_and_Plate_MicroLabBox_student_P.r_circ[((r2 - 1) << 2) + 2];
          Ball_and_Plate_MicroLabBox_student_B.rpreview_a[r1 + 75] =
            Ball_and_Plate_MicroLabBox_student_P.r_circ[((r2 - 1) << 2) + 3];
        } else {
          /* '<S10>:1:43' */
          Ball_and_Plate_MicroLabBox_student_B.rpreview_a[r1] =
            Ball_and_Plate_MicroLabBox_student_P.r_circ[224000];
          Ball_and_Plate_MicroLabBox_student_B.rpreview_a[r1 + 25] =
            Ball_and_Plate_MicroLabBox_student_P.r_circ[224001];
          Ball_and_Plate_MicroLabBox_student_B.rpreview_a[r1 + 50] =
            Ball_and_Plate_MicroLabBox_student_P.r_circ[224002];
          Ball_and_Plate_MicroLabBox_student_B.rpreview_a[r1 + 75] =
            Ball_and_Plate_MicroLabBox_student_P.r_circ[224003];
        }
      }

      if (Ball_and_Plate_MicroLabBox_student_DW.idx_ref_a <= 56001.0) {
        /* '<S10>:1:48' */
        /* '<S10>:1:49' */
        Ball_and_Plate_MicroLabBox_student_DW.idx_ref_a++;
      }
    } else {
      /* '<S10>:1:54' */
      Ball_and_Plate_MicroLabBox_student_DW.idx_ref_a = 1.0;

      /* '<S10>:1:57' */
      Ball_and_Plate_MicroLabBox_student_B.rsample_n[0] =
        Ball_and_Plate_MicroLabBox_student_P.r_circ[0];
      Ball_and_Plate_MicroLabBox_student_B.rsample_n[1] =
        Ball_and_Plate_MicroLabBox_student_P.r_circ[1];
      Ball_and_Plate_MicroLabBox_student_B.rsample_n[2] =
        Ball_and_Plate_MicroLabBox_student_P.r_circ[2];
      Ball_and_Plate_MicroLabBox_student_B.rsample_n[3] =
        Ball_and_Plate_MicroLabBox_student_P.r_circ[3];

      /* '<S10>:1:58' */
      Ball_and_Plate_MicroLabBox_student_B.asample_h[0] =
        Ball_and_Plate_MicroLabBox_student_P.a_circ[0];
      Ball_and_Plate_MicroLabBox_student_B.asample_h[1] =
        Ball_and_Plate_MicroLabBox_student_P.a_circ[1];

      /* '<S10>:1:1' */
      /* '<S10>:1:61' */
      for (r1 = 0; r1 < 25; r1++) {
        /* '<S10>:1:61' */
        /* '<S10>:1:62' */
        /* '<S10>:1:64' */
        Ball_and_Plate_MicroLabBox_student_B.rpreview_a[r1] =
          Ball_and_Plate_MicroLabBox_student_P.r_circ[(r1 + 1) << 2];
        Ball_and_Plate_MicroLabBox_student_B.rpreview_a[r1 + 25] =
          Ball_and_Plate_MicroLabBox_student_P.r_circ[((r1 + 1) << 2) + 1];
        Ball_and_Plate_MicroLabBox_student_B.rpreview_a[r1 + 50] =
          Ball_and_Plate_MicroLabBox_student_P.r_circ[((r1 + 1) << 2) + 2];
        Ball_and_Plate_MicroLabBox_student_B.rpreview_a[r1 + 75] =
          Ball_and_Plate_MicroLabBox_student_P.r_circ[((r1 + 1) << 2) + 3];
      }
    }

    /* End of MATLAB Function: '<Root>/circ_ref_player ' */

    /* MATLAB Function: '<Root>/square_ref_player ' incorporates:
     *  Constant: '<Root>/Constant10'
     *  Constant: '<Root>/Constant9'
     *  Constant: '<Root>/enable_square_ref '
     */
    /* MATLAB Function 'square_ref_player ': '<S14>:1' */
    /* '<S14>:1:14' */
    if (Ball_and_Plate_MicroLabBox_student_P.enable_square_ref_Value == 1.0) {
      /* '<S14>:1:23' */
      if (Ball_and_Plate_MicroLabBox_student_DW.idx_ref <= 48001.0) {
        /* '<S14>:1:25' */
        /* '<S14>:1:26' */
        r1 = (int32_T)Ball_and_Plate_MicroLabBox_student_DW.idx_ref;
        Ball_and_Plate_MicroLabBox_student_B.rsample[0] =
          Ball_and_Plate_MicroLabBox_student_P.r_square[(r1 - 1) << 2];
        Ball_and_Plate_MicroLabBox_student_B.rsample[1] =
          Ball_and_Plate_MicroLabBox_student_P.r_square[((r1 - 1) << 2) + 1];
        Ball_and_Plate_MicroLabBox_student_B.rsample[2] =
          Ball_and_Plate_MicroLabBox_student_P.r_square[((r1 - 1) << 2) + 2];
        Ball_and_Plate_MicroLabBox_student_B.rsample[3] =
          Ball_and_Plate_MicroLabBox_student_P.r_square[((r1 - 1) << 2) + 3];

        /* '<S14>:1:27' */
        r1 = (int32_T)Ball_and_Plate_MicroLabBox_student_DW.idx_ref;
        Ball_and_Plate_MicroLabBox_student_B.asample[0] =
          Ball_and_Plate_MicroLabBox_student_P.a_square[(r1 - 1) << 1];
        Ball_and_Plate_MicroLabBox_student_B.asample[1] =
          Ball_and_Plate_MicroLabBox_student_P.a_square[((r1 - 1) << 1) + 1];
      } else {
        /* '<S14>:1:30' */
        Ball_and_Plate_MicroLabBox_student_B.rsample[0] =
          Ball_and_Plate_MicroLabBox_student_P.r_square[192000];
        Ball_and_Plate_MicroLabBox_student_B.rsample[1] =
          Ball_and_Plate_MicroLabBox_student_P.r_square[192001];
        Ball_and_Plate_MicroLabBox_student_B.rsample[2] =
          Ball_and_Plate_MicroLabBox_student_P.r_square[192002];
        Ball_and_Plate_MicroLabBox_student_B.rsample[3] =
          Ball_and_Plate_MicroLabBox_student_P.r_square[192003];

        /* '<S14>:1:31' */
        Ball_and_Plate_MicroLabBox_student_B.asample[0] =
          Ball_and_Plate_MicroLabBox_student_P.a_square[96000];
        Ball_and_Plate_MicroLabBox_student_B.asample[1] =
          Ball_and_Plate_MicroLabBox_student_P.a_square[96001];
      }

      /* '<S14>:1:1' */
      /* '<S14>:1:35' */
      for (r1 = 0; r1 < 25; r1++) {
        /* '<S14>:1:35' */
        /* '<S14>:1:36' */
        a21 = ((real_T)r1 + 1.0) + Ball_and_Plate_MicroLabBox_student_DW.idx_ref;
        if (a21 <= 48001.0) {
          /* '<S14>:1:38' */
          /* '<S14>:1:40' */
          r2 = (int32_T)a21;
          Ball_and_Plate_MicroLabBox_student_B.rpreview[r1] =
            Ball_and_Plate_MicroLabBox_student_P.r_square[(r2 - 1) << 2];
          Ball_and_Plate_MicroLabBox_student_B.rpreview[r1 + 25] =
            Ball_and_Plate_MicroLabBox_student_P.r_square[((r2 - 1) << 2) + 1];
          Ball_and_Plate_MicroLabBox_student_B.rpreview[r1 + 50] =
            Ball_and_Plate_MicroLabBox_student_P.r_square[((r2 - 1) << 2) + 2];
          Ball_and_Plate_MicroLabBox_student_B.rpreview[r1 + 75] =
            Ball_and_Plate_MicroLabBox_student_P.r_square[((r2 - 1) << 2) + 3];
        } else {
          /* '<S14>:1:43' */
          Ball_and_Plate_MicroLabBox_student_B.rpreview[r1] =
            Ball_and_Plate_MicroLabBox_student_P.r_square[192000];
          Ball_and_Plate_MicroLabBox_student_B.rpreview[r1 + 25] =
            Ball_and_Plate_MicroLabBox_student_P.r_square[192001];
          Ball_and_Plate_MicroLabBox_student_B.rpreview[r1 + 50] =
            Ball_and_Plate_MicroLabBox_student_P.r_square[192002];
          Ball_and_Plate_MicroLabBox_student_B.rpreview[r1 + 75] =
            Ball_and_Plate_MicroLabBox_student_P.r_square[192003];
        }
      }

      if (Ball_and_Plate_MicroLabBox_student_DW.idx_ref <= 48001.0) {
        /* '<S14>:1:48' */
        /* '<S14>:1:49' */
        Ball_and_Plate_MicroLabBox_student_DW.idx_ref++;
      }
    } else {
      /* '<S14>:1:54' */
      Ball_and_Plate_MicroLabBox_student_DW.idx_ref = 1.0;

      /* '<S14>:1:57' */
      Ball_and_Plate_MicroLabBox_student_B.rsample[0] =
        Ball_and_Plate_MicroLabBox_student_P.r_square[0];
      Ball_and_Plate_MicroLabBox_student_B.rsample[1] =
        Ball_and_Plate_MicroLabBox_student_P.r_square[1];
      Ball_and_Plate_MicroLabBox_student_B.rsample[2] =
        Ball_and_Plate_MicroLabBox_student_P.r_square[2];
      Ball_and_Plate_MicroLabBox_student_B.rsample[3] =
        Ball_and_Plate_MicroLabBox_student_P.r_square[3];

      /* '<S14>:1:58' */
      Ball_and_Plate_MicroLabBox_student_B.asample[0] =
        Ball_and_Plate_MicroLabBox_student_P.a_square[0];
      Ball_and_Plate_MicroLabBox_student_B.asample[1] =
        Ball_and_Plate_MicroLabBox_student_P.a_square[1];

      /* '<S14>:1:1' */
      /* '<S14>:1:61' */
      for (r1 = 0; r1 < 25; r1++) {
        /* '<S14>:1:61' */
        /* '<S14>:1:62' */
        /* '<S14>:1:64' */
        Ball_and_Plate_MicroLabBox_student_B.rpreview[r1] =
          Ball_and_Plate_MicroLabBox_student_P.r_square[(r1 + 1) << 2];
        Ball_and_Plate_MicroLabBox_student_B.rpreview[r1 + 25] =
          Ball_and_Plate_MicroLabBox_student_P.r_square[((r1 + 1) << 2) + 1];
        Ball_and_Plate_MicroLabBox_student_B.rpreview[r1 + 50] =
          Ball_and_Plate_MicroLabBox_student_P.r_square[((r1 + 1) << 2) + 2];
        Ball_and_Plate_MicroLabBox_student_B.rpreview[r1 + 75] =
          Ball_and_Plate_MicroLabBox_student_P.r_square[((r1 + 1) << 2) + 3];
      }
    }

    /* End of MATLAB Function: '<Root>/square_ref_player ' */

    /* Sum: '<Root>/Add4' */
    Ball_and_Plate_MicroLabBox_student_B.Add4[0] =
      Ball_and_Plate_MicroLabBox_student_B.rsample_n[0] +
      Ball_and_Plate_MicroLabBox_student_B.rsample[0];
    Ball_and_Plate_MicroLabBox_student_B.Add4[1] =
      Ball_and_Plate_MicroLabBox_student_B.rsample_n[1] +
      Ball_and_Plate_MicroLabBox_student_B.rsample[1];
    Ball_and_Plate_MicroLabBox_student_B.Add4[2] =
      Ball_and_Plate_MicroLabBox_student_B.rsample_n[2] +
      Ball_and_Plate_MicroLabBox_student_B.rsample[2];
    Ball_and_Plate_MicroLabBox_student_B.Add4[3] =
      Ball_and_Plate_MicroLabBox_student_B.rsample_n[3] +
      Ball_and_Plate_MicroLabBox_student_B.rsample[3];

    /* Sum: '<S9>/Sum' */
    Ball_and_Plate_MicroLabBox_student_B.Sum[0] =
      Ball_and_Plate_MicroLabBox_student_B.Add4[0] -
      Ball_and_Plate_MicroLabBox_student_B.Switch[0];
    Ball_and_Plate_MicroLabBox_student_B.Sum[1] =
      Ball_and_Plate_MicroLabBox_student_B.Add4[1] -
      Ball_and_Plate_MicroLabBox_student_B.Switch[1];
    Ball_and_Plate_MicroLabBox_student_B.Sum[2] =
      Ball_and_Plate_MicroLabBox_student_B.Add4[2] -
      Ball_and_Plate_MicroLabBox_student_B.Switch[2];
    Ball_and_Plate_MicroLabBox_student_B.Sum[3] =
      Ball_and_Plate_MicroLabBox_student_B.Add4[3] -
      Ball_and_Plate_MicroLabBox_student_B.Switch[3];

    /* RelationalOperator: '<S180>/Compare' incorporates:
     *  Constant: '<Root>/enable_outer_controller'
     *  Constant: '<S180>/Constant'
     */
    Ball_and_Plate_MicroLabBox_student_B.Compare =
      (Ball_and_Plate_MicroLabBox_student_P.enable_outer_controller_Value >
       Ball_and_Plate_MicroLabBox_student_P.Constant_Value_h);

    /* UnitDelay: '<S177>/Delay Input1' */
    Ball_and_Plate_MicroLabBox_student_B.Uk1_j =
      Ball_and_Plate_MicroLabBox_student_DW.DelayInput1_DSTATE;

    /* RelationalOperator: '<S177>/FixPt Relational Operator' */
    Ball_and_Plate_MicroLabBox_student_B.FixPtRelationalOperator = ((int32_T)
      Ball_and_Plate_MicroLabBox_student_B.Compare > (int32_T)
      Ball_and_Plate_MicroLabBox_student_B.Uk1_j);

    /* MATLAB Function: '<S9>/MATLAB Function1' incorporates:
     *  Constant: '<S9>/Constant'
     *  Constant: '<S9>/Constant1'
     *  Constant: '<S9>/Constant2'
     *  Constant: '<S9>/Constant3'
     *  Constant: '<S9>/Constant4'
     *  Constant: '<S9>/Constant5'
     *  Constant: '<S9>/Constant6'
     *  Constant: '<S9>/Constant7'
     */
    /* MATLAB Function 'Subsystem/MATLAB Function1': '<S179>:1' */
    /* '<S179>:1:70' */
    /* '<S179>:1:33' */
    Ball_and_Plate_MicroLabBox_student_B.u_raw[0] = 0.0;
    Ball_and_Plate_MicroLabBox_student_B.u_raw[1] = 0.0;

    /* '<S179>:1:34' */
    Ball_and_Plate_MicroLabBox_student_B.u_box[0] = 0.0;
    Ball_and_Plate_MicroLabBox_student_B.u_box[1] = 0.0;

    /* '<S179>:1:35' */
    Ball_and_Plate_MicroLabBox_student_B.u_cmd[0] = 0.0;
    Ball_and_Plate_MicroLabBox_student_B.u_cmd[1] = 0.0;

    /* '<S179>:1:36' */
    Ball_and_Plate_MicroLabBox_student_B.sat_flag = false;

    /* '<S179>:1:37' */
    Ball_and_Plate_MicroLabBox_student_B.xK_norm = 0.0;
    if (Ball_and_Plate_MicroLabBox_student_B.FixPtRelationalOperator) {
      /* '<S179>:1:40' */
      /* '<S179>:1:41' */
      memset(&Ball_and_Plate_MicroLabBox_student_DW.xK[0], 0, sizeof(real_T) <<
             4U);

      /* '<S179>:1:42' */
      Ball_and_Plate_MicroLabBox_student_DW.u_prev[0] = 0.0;
      Ball_and_Plate_MicroLabBox_student_DW.u_prev[1] = 0.0;
    } else {
      /* '<S179>:1:48' */
      x_pred[0] = Ball_and_Plate_MicroLabBox_student_B.Sum[0];
      x_pred[1] = Ball_and_Plate_MicroLabBox_student_B.Sum[1];
      x_pred[2] = Ball_and_Plate_MicroLabBox_student_B.Sum[2];
      x_pred[3] = Ball_and_Plate_MicroLabBox_student_B.Sum[3];

      /* '<S179>:1:50' */
      u_min[0] = Ball_and_Plate_MicroLabBox_student_P.u_min_robust[0];
      u_min[1] = Ball_and_Plate_MicroLabBox_student_P.u_min_robust[1];

      /* '<S179>:1:51' */
      a21 = Ball_and_Plate_MicroLabBox_student_P.u_max_robust[0];
      uX_idx_0 = Ball_and_Plate_MicroLabBox_student_P.u_max_robust[1];

      /* '<S179>:1:55' */
      for (r1 = 0; r1 < 2; r1++) {
        tmp_7[r1] = 0.0;
        for (r2 = 0; r2 < 16; r2++) {
          center_idx_2 = tmp_7[r1];
          center_idx_2 += Ball_and_Plate_MicroLabBox_student_P.C_robust[(r2 << 1)
            + r1] * Ball_and_Plate_MicroLabBox_student_DW.xK[r2];
          tmp_7[r1] = center_idx_2;
        }

        center_idx_2 = Ball_and_Plate_MicroLabBox_student_P.D_robust[r1] *
          x_pred[0];
        center_idx_2 += Ball_and_Plate_MicroLabBox_student_P.D_robust[r1 + 2] *
          x_pred[1];
        center_idx_2 += Ball_and_Plate_MicroLabBox_student_P.D_robust[r1 + 4] *
          x_pred[2];
        center_idx_2 += Ball_and_Plate_MicroLabBox_student_P.D_robust[r1 + 6] *
          x_pred[3];
        tmp_8[r1] = center_idx_2;
      }

      Ball_and_Plate_MicroLabBox_student_B.u_raw[0] = tmp_7[0] + tmp_8[0];
      Ball_and_Plate_MicroLabBox_student_B.u_raw[1] = tmp_7[1] + tmp_8[1];

      /* '<S179>:1:59' */
      center_idx_2 = Ball_and_Plate_MicroLabBox_student_B.u_raw[0];
      uX_idx_1 = u_min[0];
      if ((center_idx_2 > uX_idx_1) || rtIsNaN(uX_idx_1)) {
        uX_idx_1 = center_idx_2;
      }

      center_idx_2 = Ball_and_Plate_MicroLabBox_student_B.u_raw[1];
      uX_idx_2 = u_min[1];
      if ((center_idx_2 > uX_idx_2) || rtIsNaN(uX_idx_2)) {
        uX_idx_2 = center_idx_2;
      }

      if ((!(uX_idx_1 < a21)) && (!rtIsNaN(a21))) {
        uX_idx_1 = a21;
      }

      Ball_and_Plate_MicroLabBox_student_B.u_box[0] = uX_idx_1;
      if ((!(uX_idx_2 < uX_idx_0)) && (!rtIsNaN(uX_idx_0))) {
        uX_idx_2 = uX_idx_0;
      }

      Ball_and_Plate_MicroLabBox_student_B.u_box[1] = uX_idx_2;

      /* '<S179>:1:68' */
      uX_idx_2 = Ball_and_Plate_MicroLabBox_student_B.u_box[0] -
        Ball_and_Plate_MicroLabBox_student_DW.u_prev[0];
      uX_idx_1 = Ball_and_Plate_MicroLabBox_student_B.u_box[1] -
        Ball_and_Plate_MicroLabBox_student_DW.u_prev[1];

      /* '<S179>:1:70' */
      /* '<S179>:1:72' */
      /* '<S179>:1:75' */
      if (!(uX_idx_2 > -0.0069813170079773184)) {
        uX_idx_2 = -0.0069813170079773184;
      }

      if (!(uX_idx_2 < 0.0069813170079773184)) {
        uX_idx_2 = 0.0069813170079773184;
      }

      Ball_and_Plate_MicroLabBox_student_B.u_cmd[0] = uX_idx_2;
      Ball_and_Plate_MicroLabBox_student_B.u_cmd[0] +=
        Ball_and_Plate_MicroLabBox_student_DW.u_prev[0];
      center_idx_2 = Ball_and_Plate_MicroLabBox_student_B.u_cmd[0];
      uX_idx_2 = u_min[0];
      if ((center_idx_2 > uX_idx_2) || rtIsNaN(uX_idx_2)) {
        uX_idx_2 = center_idx_2;
      }

      if ((uX_idx_2 < a21) || rtIsNaN(a21)) {
        a21 = uX_idx_2;
      }

      Ball_and_Plate_MicroLabBox_student_B.u_cmd[0] = a21;
      if (uX_idx_1 > -0.0069813170079773184) {
        uX_idx_2 = uX_idx_1;
      } else {
        uX_idx_2 = -0.0069813170079773184;
      }

      if (!(uX_idx_2 < 0.0069813170079773184)) {
        uX_idx_2 = 0.0069813170079773184;
      }

      Ball_and_Plate_MicroLabBox_student_B.u_cmd[1] = uX_idx_2;
      Ball_and_Plate_MicroLabBox_student_B.u_cmd[1] +=
        Ball_and_Plate_MicroLabBox_student_DW.u_prev[1];
      center_idx_2 = Ball_and_Plate_MicroLabBox_student_B.u_cmd[1];
      uX_idx_2 = u_min[1];
      if ((center_idx_2 > uX_idx_2) || rtIsNaN(uX_idx_2)) {
        uX_idx_2 = center_idx_2;
      }

      if ((uX_idx_2 < uX_idx_0) || rtIsNaN(uX_idx_0)) {
        uX_idx_0 = uX_idx_2;
      }

      Ball_and_Plate_MicroLabBox_student_B.u_cmd[1] = uX_idx_0;

      /* '<S179>:1:80' */
      /* '<S179>:1:82' */
      /* '<S179>:1:83' */
      for (r1 = 0; r1 < 16; r1++) {
        Ad_0[r1] = 0.0;
        for (r2 = 0; r2 < 16; r2++) {
          center_idx_2 = Ad_0[r1];
          center_idx_2 += Ball_and_Plate_MicroLabBox_student_P.A_robust[(r2 << 4)
            + r1] * Ball_and_Plate_MicroLabBox_student_DW.xK[r2];
          Ad_0[r1] = center_idx_2;
        }

        center_idx_2 = Ball_and_Plate_MicroLabBox_student_P.B_robust[r1] *
          x_pred[0];
        center_idx_2 += Ball_and_Plate_MicroLabBox_student_P.B_robust[r1 + 16] *
          x_pred[1];
        center_idx_2 += Ball_and_Plate_MicroLabBox_student_P.B_robust[r1 + 32] *
          x_pred[2];
        center_idx_2 += Ball_and_Plate_MicroLabBox_student_P.B_robust[r1 + 48] *
          x_pred[3];
        P_pred[r1] = center_idx_2;
      }

      uX_idx_0 = Ball_and_Plate_MicroLabBox_student_B.u_cmd[0] -
        Ball_and_Plate_MicroLabBox_student_B.u_raw[0];
      uX_idx_1 = Ball_and_Plate_MicroLabBox_student_B.u_cmd[1] -
        Ball_and_Plate_MicroLabBox_student_B.u_raw[1];
      r2 = 16;
      for (r1 = 0; r1 < 16; r1++) {
        center_idx_2 = Ball_and_Plate_MicroLabBox_student_P.Baw_robust[r1] *
          uX_idx_0;
        center_idx_2 += Ball_and_Plate_MicroLabBox_student_P.Baw_robust[r1 + 16]
          * uX_idx_1;
        a21 = (Ad_0[r1] + P_pred[r1]) +
          Ball_and_Plate_MicroLabBox_student_P.Kaw_robust * center_idx_2;
        tmp_data[r1] = (rtIsInf(a21) || rtIsNaN(a21));
        Ad[r1] = a21;
      }

      if (Ball_and_Plate_MicroL_vectorAny(tmp_data, &r2) ||
          (Ball_and_Plate_MicroLabB_norm_m(Ad) > 1.0E+8)) {
        /* '<S179>:1:86' */
        /* '<S179>:1:87' */
        memset(&Ad[0], 0, sizeof(real_T) << 4U);

        /* '<S179>:1:88' */
        Ball_and_Plate_MicroLabBox_student_B.u_cmd[0] = 0.0;
        Ball_and_Plate_MicroLabBox_student_B.u_cmd[1] = 0.0;
      }

      /* '<S179>:1:91' */
      memcpy(&Ball_and_Plate_MicroLabBox_student_DW.xK[0], &Ad[0], sizeof(real_T)
             << 4U);

      /* '<S179>:1:92' */
      Ball_and_Plate_MicroLabBox_student_DW.u_prev[0] =
        Ball_and_Plate_MicroLabBox_student_B.u_cmd[0];
      Ball_and_Plate_MicroLabBox_student_DW.u_prev[1] =
        Ball_and_Plate_MicroLabBox_student_B.u_cmd[1];

      /* '<S179>:1:94' */
      uX_idx_2 = Ball_and_Plate_MicroLabBox_student_B.u_cmd[0] -
        Ball_and_Plate_MicroLabBox_student_B.u_raw[0];
      uX_idx_1 = Ball_and_Plate_MicroLabBox_student_B.u_cmd[1] -
        Ball_and_Plate_MicroLabBox_student_B.u_raw[1];
      r1 = 2;
      center_idx_2 = fabs(uX_idx_2);
      tmp_data[0] = (center_idx_2 > 1.0E-10);
      center_idx_2 = fabs(uX_idx_1);
      tmp_data[1] = (center_idx_2 > 1.0E-10);
      Ball_and_Plate_MicroLabBox_student_B.sat_flag =
        Ball_and_Plate_MicroL_vectorAny(tmp_data, &r1);

      /* '<S179>:1:95' */
      Ball_and_Plate_MicroLabBox_student_B.xK_norm =
        Ball_and_Plate_MicroLabB_norm_m(Ball_and_Plate_MicroLabBox_student_DW.xK);
    }

    /* End of MATLAB Function: '<S9>/MATLAB Function1' */

    /* MATLAB Function: '<S9>/MATLAB Function' */
    /* MATLAB Function 'Subsystem/MATLAB Function': '<S178>:1' */
    /* '<S178>:1:19' */
    /* '<S178>:1:21' */
    /* '<S178>:1:22' */
    /* '<S178>:1:25' */
    if ((!Ball_and_Plate_MicroLabBox_student_DW.initialized_not_empty) ||
        Ball_and_Plate_MicroLabBox_student_B.FixPtRelationalOperator) {
      /* '<S178>:1:27' */
      /* '<S178>:1:28' */
      Ball_and_Plate_MicroLabBox_student_DW.alpha_I = 0.0;

      /* '<S178>:1:29' */
      Ball_and_Plate_MicroLabBox_student_DW.beta_I = 0.0;

      /* '<S178>:1:30' */
      Ball_and_Plate_MicroLabBox_student_DW.initialized_not_empty = true;
    }

    if (fabs(Ball_and_Plate_MicroLabBox_student_B.Sum[2]) > 0.004) {
      /* '<S178>:1:34' */
      /* '<S178>:1:35' */
      Ball_and_Plate_MicroLabBox_student_DW.alpha_I += 0.002 *
        Ball_and_Plate_MicroLabBox_student_B.Sum[2] * 0.02;
    }

    if (fabs(Ball_and_Plate_MicroLabBox_student_B.Sum[0]) > 0.004) {
      /* '<S178>:1:38' */
      /* '<S178>:1:39' */
      Ball_and_Plate_MicroLabBox_student_DW.beta_I += 0.002 *
        Ball_and_Plate_MicroLabBox_student_B.Sum[0] * 0.02;
    }

    /* '<S178>:1:43' */
    center_idx_2 = Ball_and_Plate_MicroLabBox_student_DW.alpha_I;
    if (!(center_idx_2 > -0.017453292519943295)) {
      center_idx_2 = -0.017453292519943295;
    }

    if (center_idx_2 < 0.017453292519943295) {
      Ball_and_Plate_MicroLabBox_student_DW.alpha_I = center_idx_2;
    } else {
      Ball_and_Plate_MicroLabBox_student_DW.alpha_I = 0.017453292519943295;
    }

    /* '<S178>:1:44' */
    center_idx_2 = Ball_and_Plate_MicroLabBox_student_DW.beta_I;
    if (!(center_idx_2 > -0.017453292519943295)) {
      center_idx_2 = -0.017453292519943295;
    }

    if (center_idx_2 < 0.017453292519943295) {
      Ball_and_Plate_MicroLabBox_student_DW.beta_I = center_idx_2;
    } else {
      Ball_and_Plate_MicroLabBox_student_DW.beta_I = 0.017453292519943295;
    }

    /* '<S178>:1:47' */
    Ball_and_Plate_MicroLabBox_student_B.u_total[0] =
      Ball_and_Plate_MicroLabBox_student_B.u_cmd[0] +
      Ball_and_Plate_MicroLabBox_student_DW.alpha_I;
    Ball_and_Plate_MicroLabBox_student_B.u_total[1] =
      Ball_and_Plate_MicroLabBox_student_B.u_cmd[1] +
      Ball_and_Plate_MicroLabBox_student_DW.beta_I;

    /* End of MATLAB Function: '<S9>/MATLAB Function' */

    /* RateLimiter: '<S9>/Rate Limiter' */
    center_idx_2 = Ball_and_Plate_MicroLabBox_student_B.u_total[0] -
      Ball_and_Plate_MicroLabBox_student_DW.PrevY[0];
    if (center_idx_2 >
        Ball_and_Plate_MicroLabBox_student_P.RateLimiter_RisingLim) {
      Ball_and_Plate_MicroLabBox_student_B.RateLimiter[0] =
        Ball_and_Plate_MicroLabBox_student_DW.PrevY[0] +
        Ball_and_Plate_MicroLabBox_student_P.RateLimiter_RisingLim;
    } else if (center_idx_2 <
               Ball_and_Plate_MicroLabBox_student_P.RateLimiter_FallingLim) {
      Ball_and_Plate_MicroLabBox_student_B.RateLimiter[0] =
        Ball_and_Plate_MicroLabBox_student_DW.PrevY[0] +
        Ball_and_Plate_MicroLabBox_student_P.RateLimiter_FallingLim;
    } else {
      Ball_and_Plate_MicroLabBox_student_B.RateLimiter[0] =
        Ball_and_Plate_MicroLabBox_student_B.u_total[0];
    }

    Ball_and_Plate_MicroLabBox_student_DW.PrevY[0] =
      Ball_and_Plate_MicroLabBox_student_B.RateLimiter[0];
    center_idx_2 = Ball_and_Plate_MicroLabBox_student_B.u_total[1] -
      Ball_and_Plate_MicroLabBox_student_DW.PrevY[1];
    if (center_idx_2 >
        Ball_and_Plate_MicroLabBox_student_P.RateLimiter_RisingLim) {
      Ball_and_Plate_MicroLabBox_student_B.RateLimiter[1] =
        Ball_and_Plate_MicroLabBox_student_DW.PrevY[1] +
        Ball_and_Plate_MicroLabBox_student_P.RateLimiter_RisingLim;
    } else if (center_idx_2 <
               Ball_and_Plate_MicroLabBox_student_P.RateLimiter_FallingLim) {
      Ball_and_Plate_MicroLabBox_student_B.RateLimiter[1] =
        Ball_and_Plate_MicroLabBox_student_DW.PrevY[1] +
        Ball_and_Plate_MicroLabBox_student_P.RateLimiter_FallingLim;
    } else {
      Ball_and_Plate_MicroLabBox_student_B.RateLimiter[1] =
        Ball_and_Plate_MicroLabBox_student_B.u_total[1];
    }

    Ball_and_Plate_MicroLabBox_student_DW.PrevY[1] =
      Ball_and_Plate_MicroLabBox_student_B.RateLimiter[1];

    /* End of RateLimiter: '<S9>/Rate Limiter' */

    /* DataTypeConversion: '<S64>/mo or x Conversion' */
    Ball_and_Plate_MicroLabBox_student_B.moorxConversion[0] =
      Ball_and_Plate_MicroLabBox_student_B.Switch[0];
    Ball_and_Plate_MicroLabBox_student_B.moorxConversion[1] =
      Ball_and_Plate_MicroLabBox_student_B.Switch[1];
    Ball_and_Plate_MicroLabBox_student_B.moorxConversion[2] =
      Ball_and_Plate_MicroLabBox_student_B.Switch[2];
    Ball_and_Plate_MicroLabBox_student_B.moorxConversion[3] =
      Ball_and_Plate_MicroLabBox_student_B.Switch[3];

    /* Sum: '<Root>/Add6' */
    for (r1 = 0; r1 < 100; r1++) {
      Ball_and_Plate_MicroLabBox_student_B.Add6[r1] =
        Ball_and_Plate_MicroLabBox_student_B.rpreview_a[r1] +
        Ball_and_Plate_MicroLabBox_student_B.rpreview[r1];
    }

    /* End of Sum: '<Root>/Add6' */

    /* DataTypeConversion: '<S64>/Data Type Conversion1' */
    memcpy(&Ball_and_Plate_MicroLabBox_student_B.DataTypeConversion1[0],
           &Ball_and_Plate_MicroLabBox_student_B.Add6[0], 100U * sizeof(real_T));
  }

  if (rtmIsMajorTimeStep(Ball_and_Plate_MicroLabBox_student_M) &&
      Ball_and_Plate_MicroLabBox_student_M->Timing.TaskCounters.TID[2] == 0) {
    /* UnitDelay: '<S64>/last_mv' */
    Ball_and_Plate_MicroLabBox_student_B.last_mv[0] =
      Ball_and_Plate_MicroLabBox_student_DW.last_mv_DSTATE[0];
    Ball_and_Plate_MicroLabBox_student_B.last_mv[1] =
      Ball_and_Plate_MicroLabBox_student_DW.last_mv_DSTATE[1];

    /* DataTypeConversion: '<S64>/Data Type Conversion2' incorporates:
     *  Constant: '<S5>/md_zero'
     */
    Ball_and_Plate_MicroLabBox_student_B.DataTypeConversion2 =
      Ball_and_Plate_MicroLabBox_student_P.md_zero_Value;

    /* DataTypeConversion: '<S64>/Data Type Conversion4' incorporates:
     *  Constant: '<S5>/umin_zero'
     */
    Ball_and_Plate_MicroLabBox_student_B.DataTypeConversion4[0] =
      Ball_and_Plate_MicroLabBox_student_P.umin_zero_Value[0];
    Ball_and_Plate_MicroLabBox_student_B.DataTypeConversion4[1] =
      Ball_and_Plate_MicroLabBox_student_P.umin_zero_Value[1];

    /* Gain: '<S64>/umin_scale' */
    Ball_and_Plate_MicroLabBox_student_B.umin_scale[0] =
      Ball_and_Plate_MicroLabBox_student_P.umin_scale_Gain[0] *
      Ball_and_Plate_MicroLabBox_student_B.DataTypeConversion4[0];
    Ball_and_Plate_MicroLabBox_student_B.umin_scale[1] =
      Ball_and_Plate_MicroLabBox_student_P.umin_scale_Gain[1] *
      Ball_and_Plate_MicroLabBox_student_B.DataTypeConversion4[1];

    /* DataTypeConversion: '<S64>/Data Type Conversion5' incorporates:
     *  Constant: '<S5>/umax_zero'
     */
    Ball_and_Plate_MicroLabBox_student_B.DataTypeConversion5[0] =
      Ball_and_Plate_MicroLabBox_student_P.umax_zero_Value[0];
    Ball_and_Plate_MicroLabBox_student_B.DataTypeConversion5[1] =
      Ball_and_Plate_MicroLabBox_student_P.umax_zero_Value[1];

    /* Gain: '<S64>/umax_scale' */
    Ball_and_Plate_MicroLabBox_student_B.umax_scale[0] =
      Ball_and_Plate_MicroLabBox_student_P.umax_scale_Gain[0] *
      Ball_and_Plate_MicroLabBox_student_B.DataTypeConversion5[0];
    Ball_and_Plate_MicroLabBox_student_B.umax_scale[1] =
      Ball_and_Plate_MicroLabBox_student_P.umax_scale_Gain[1] *
      Ball_and_Plate_MicroLabBox_student_B.DataTypeConversion5[1];

    /* DataTypeConversion: '<S64>/Data Type Conversion6' incorporates:
     *  Constant: '<S5>/ymin_zero'
     */
    Ball_and_Plate_MicroLabBox_student_B.DataTypeConversion6[0] =
      Ball_and_Plate_MicroLabBox_student_P.ymin_zero_Value[0];
    Ball_and_Plate_MicroLabBox_student_B.DataTypeConversion6[1] =
      Ball_and_Plate_MicroLabBox_student_P.ymin_zero_Value[1];
    Ball_and_Plate_MicroLabBox_student_B.DataTypeConversion6[2] =
      Ball_and_Plate_MicroLabBox_student_P.ymin_zero_Value[2];
    Ball_and_Plate_MicroLabBox_student_B.DataTypeConversion6[3] =
      Ball_and_Plate_MicroLabBox_student_P.ymin_zero_Value[3];

    /* Gain: '<S64>/ymin_scale' */
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

    /* DataTypeConversion: '<S64>/Data Type Conversion7' incorporates:
     *  Constant: '<S5>/ymax_zero'
     */
    Ball_and_Plate_MicroLabBox_student_B.DataTypeConversion7[0] =
      Ball_and_Plate_MicroLabBox_student_P.ymax_zero_Value[0];
    Ball_and_Plate_MicroLabBox_student_B.DataTypeConversion7[1] =
      Ball_and_Plate_MicroLabBox_student_P.ymax_zero_Value[1];
    Ball_and_Plate_MicroLabBox_student_B.DataTypeConversion7[2] =
      Ball_and_Plate_MicroLabBox_student_P.ymax_zero_Value[2];
    Ball_and_Plate_MicroLabBox_student_B.DataTypeConversion7[3] =
      Ball_and_Plate_MicroLabBox_student_P.ymax_zero_Value[3];

    /* Gain: '<S64>/ymax_scale' */
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

    /* DataTypeConversion: '<S64>/E Conversion' incorporates:
     *  Constant: '<S5>/E_zero'
     */
    Ball_and_Plate_MicroLabBox_student_B.EConversion[0] =
      Ball_and_Plate_MicroLabBox_student_P.E_zero_Value[0];
    Ball_and_Plate_MicroLabBox_student_B.EConversion[1] =
      Ball_and_Plate_MicroLabBox_student_P.E_zero_Value[1];

    /* Gain: '<S64>/umin_scale4' */
    Ball_and_Plate_MicroLabBox_student_B.umin_scale4[0] =
      Ball_and_Plate_MicroLabBox_student_P.umin_scale4_Gain[0] *
      Ball_and_Plate_MicroLabBox_student_B.EConversion[0];

    /* Reshape: '<S64>/Reshape' */
    Ball_and_Plate_MicroLabBox_student_B.Reshape[0] =
      Ball_and_Plate_MicroLabBox_student_B.umin_scale4[0];

    /* Gain: '<S64>/umin_scale4' */
    Ball_and_Plate_MicroLabBox_student_B.umin_scale4[1] =
      Ball_and_Plate_MicroLabBox_student_P.umin_scale4_Gain[1] *
      Ball_and_Plate_MicroLabBox_student_B.EConversion[1];

    /* Reshape: '<S64>/Reshape' */
    Ball_and_Plate_MicroLabBox_student_B.Reshape[1] =
      Ball_and_Plate_MicroLabBox_student_B.umin_scale4[1];

    /* DataTypeConversion: '<S64>/F Conversion' incorporates:
     *  Constant: '<S5>/F_zero'
     */
    Ball_and_Plate_MicroLabBox_student_B.FConversion[0] =
      Ball_and_Plate_MicroLabBox_student_P.F_zero_Value[0];
    Ball_and_Plate_MicroLabBox_student_B.FConversion[1] =
      Ball_and_Plate_MicroLabBox_student_P.F_zero_Value[1];
    Ball_and_Plate_MicroLabBox_student_B.FConversion[2] =
      Ball_and_Plate_MicroLabBox_student_P.F_zero_Value[2];
    Ball_and_Plate_MicroLabBox_student_B.FConversion[3] =
      Ball_and_Plate_MicroLabBox_student_P.F_zero_Value[3];

    /* Gain: '<S64>/ymin_scale1' */
    Ball_and_Plate_MicroLabBox_student_B.ymin_scale1[0] =
      Ball_and_Plate_MicroLabBox_student_P.ymin_scale1_Gain[0] *
      Ball_and_Plate_MicroLabBox_student_B.FConversion[0];

    /* Reshape: '<S64>/Reshape1' */
    Ball_and_Plate_MicroLabBox_student_B.Reshape1[0] =
      Ball_and_Plate_MicroLabBox_student_B.ymin_scale1[0];

    /* Gain: '<S64>/ymin_scale1' */
    Ball_and_Plate_MicroLabBox_student_B.ymin_scale1[1] =
      Ball_and_Plate_MicroLabBox_student_P.ymin_scale1_Gain[1] *
      Ball_and_Plate_MicroLabBox_student_B.FConversion[1];

    /* Reshape: '<S64>/Reshape1' */
    Ball_and_Plate_MicroLabBox_student_B.Reshape1[1] =
      Ball_and_Plate_MicroLabBox_student_B.ymin_scale1[1];

    /* Gain: '<S64>/ymin_scale1' */
    Ball_and_Plate_MicroLabBox_student_B.ymin_scale1[2] =
      Ball_and_Plate_MicroLabBox_student_P.ymin_scale1_Gain[2] *
      Ball_and_Plate_MicroLabBox_student_B.FConversion[2];

    /* Reshape: '<S64>/Reshape1' */
    Ball_and_Plate_MicroLabBox_student_B.Reshape1[2] =
      Ball_and_Plate_MicroLabBox_student_B.ymin_scale1[2];

    /* Gain: '<S64>/ymin_scale1' */
    Ball_and_Plate_MicroLabBox_student_B.ymin_scale1[3] =
      Ball_and_Plate_MicroLabBox_student_P.ymin_scale1_Gain[3] *
      Ball_and_Plate_MicroLabBox_student_B.FConversion[3];

    /* Reshape: '<S64>/Reshape1' */
    Ball_and_Plate_MicroLabBox_student_B.Reshape1[3] =
      Ball_and_Plate_MicroLabBox_student_B.ymin_scale1[3];

    /* DataTypeConversion: '<S64>/G Conversion' incorporates:
     *  Constant: '<S5>/G_zero'
     */
    Ball_and_Plate_MicroLabBox_student_B.GConversion =
      Ball_and_Plate_MicroLabBox_student_P.G_zero_Value;

    /* DataTypeConversion: '<S64>/S Conversion' incorporates:
     *  Constant: '<S5>/S_zero'
     */
    Ball_and_Plate_MicroLabBox_student_B.SConversion =
      Ball_and_Plate_MicroLabBox_student_P.S_zero_Value;

    /* Gain: '<S64>/ymin_scale2' */
    Ball_and_Plate_MicroLabBox_student_B.ymin_scale2 =
      Ball_and_Plate_MicroLabBox_student_P.ymin_scale2_Gain *
      Ball_and_Plate_MicroLabBox_student_B.SConversion;

    /* Reshape: '<S64>/Reshape2' */
    Ball_and_Plate_MicroLabBox_student_B.Reshape2 =
      Ball_and_Plate_MicroLabBox_student_B.ymin_scale2;

    /* DataTypeConversion: '<S64>/Data Type Conversion8' incorporates:
     *  Constant: '<S5>/switch_zero'
     */
    Ball_and_Plate_MicroLabBox_student_B.DataTypeConversion8 =
      Ball_and_Plate_MicroLabBox_student_P.switch_zero_Value;

    /* DataTypeConversion: '<S64>/Data Type Conversion3' incorporates:
     *  Constant: '<S5>/ext.mv_zero'
     */
    Ball_and_Plate_MicroLabBox_student_B.DataTypeConversion3[0] =
      Ball_and_Plate_MicroLabBox_student_P.extmv_zero_Value[0];
    Ball_and_Plate_MicroLabBox_student_B.DataTypeConversion3[1] =
      Ball_and_Plate_MicroLabBox_student_P.extmv_zero_Value[1];

    /* Gain: '<S64>/ext.mv_scale' */
    Ball_and_Plate_MicroLabBox_student_B.extmv_scale[0] =
      Ball_and_Plate_MicroLabBox_student_P.extmv_scale_Gain[0] *
      Ball_and_Plate_MicroLabBox_student_B.DataTypeConversion3[0];
    Ball_and_Plate_MicroLabBox_student_B.extmv_scale[1] =
      Ball_and_Plate_MicroLabBox_student_P.extmv_scale_Gain[1] *
      Ball_and_Plate_MicroLabBox_student_B.DataTypeConversion3[1];

    /* DataTypeConversion: '<S64>/Data Type Conversion13' incorporates:
     *  Constant: '<S5>/mv.target_zero'
     */
    Ball_and_Plate_MicroLabBox_student_B.DataTypeConversion13[0] =
      Ball_and_Plate_MicroLabBox_student_P.mvtarget_zero_Value[0];
    Ball_and_Plate_MicroLabBox_student_B.DataTypeConversion13[1] =
      Ball_and_Plate_MicroLabBox_student_P.mvtarget_zero_Value[1];

    /* Gain: '<S64>/ext.mv_scale1' */
    Ball_and_Plate_MicroLabBox_student_B.extmv_scale1[0] =
      Ball_and_Plate_MicroLabBox_student_P.extmv_scale1_Gain[0] *
      Ball_and_Plate_MicroLabBox_student_B.DataTypeConversion13[0];
    Ball_and_Plate_MicroLabBox_student_B.extmv_scale1[1] =
      Ball_and_Plate_MicroLabBox_student_P.extmv_scale1_Gain[1] *
      Ball_and_Plate_MicroLabBox_student_B.DataTypeConversion13[1];

    /* DataTypeConversion: '<S64>/Data Type Conversion9' incorporates:
     *  Constant: '<S5>/y.wt_zero'
     */
    Ball_and_Plate_MicroLabBox_student_B.DataTypeConversion9[0] =
      Ball_and_Plate_MicroLabBox_student_P.ywt_zero_Value[0];
    Ball_and_Plate_MicroLabBox_student_B.DataTypeConversion9[1] =
      Ball_and_Plate_MicroLabBox_student_P.ywt_zero_Value[1];
    Ball_and_Plate_MicroLabBox_student_B.DataTypeConversion9[2] =
      Ball_and_Plate_MicroLabBox_student_P.ywt_zero_Value[2];
    Ball_and_Plate_MicroLabBox_student_B.DataTypeConversion9[3] =
      Ball_and_Plate_MicroLabBox_student_P.ywt_zero_Value[3];

    /* Math: '<S64>/Math Function' */
    Ball_and_Plate_MicroLabBox_student_B.MathFunction[0] =
      Ball_and_Plate_MicroLabBox_student_B.DataTypeConversion9[0];

    /* Reshape: '<S64>/Reshape3' */
    Ball_and_Plate_MicroLabBox_student_B.Reshape3[0] =
      Ball_and_Plate_MicroLabBox_student_B.MathFunction[0];

    /* Math: '<S64>/Math Function' */
    Ball_and_Plate_MicroLabBox_student_B.MathFunction[1] =
      Ball_and_Plate_MicroLabBox_student_B.DataTypeConversion9[1];

    /* Reshape: '<S64>/Reshape3' */
    Ball_and_Plate_MicroLabBox_student_B.Reshape3[1] =
      Ball_and_Plate_MicroLabBox_student_B.MathFunction[1];

    /* Math: '<S64>/Math Function' */
    Ball_and_Plate_MicroLabBox_student_B.MathFunction[2] =
      Ball_and_Plate_MicroLabBox_student_B.DataTypeConversion9[2];

    /* Reshape: '<S64>/Reshape3' */
    Ball_and_Plate_MicroLabBox_student_B.Reshape3[2] =
      Ball_and_Plate_MicroLabBox_student_B.MathFunction[2];

    /* Math: '<S64>/Math Function' */
    Ball_and_Plate_MicroLabBox_student_B.MathFunction[3] =
      Ball_and_Plate_MicroLabBox_student_B.DataTypeConversion9[3];

    /* Reshape: '<S64>/Reshape3' */
    Ball_and_Plate_MicroLabBox_student_B.Reshape3[3] =
      Ball_and_Plate_MicroLabBox_student_B.MathFunction[3];

    /* DataTypeConversion: '<S64>/Data Type Conversion10' incorporates:
     *  Constant: '<S5>/u.wt_zero'
     */
    Ball_and_Plate_MicroLabBox_student_B.DataTypeConversion10[0] =
      Ball_and_Plate_MicroLabBox_student_P.uwt_zero_Value[0];
    Ball_and_Plate_MicroLabBox_student_B.DataTypeConversion10[1] =
      Ball_and_Plate_MicroLabBox_student_P.uwt_zero_Value[1];

    /* Math: '<S64>/Math Function1' */
    Ball_and_Plate_MicroLabBox_student_B.MathFunction1[0] =
      Ball_and_Plate_MicroLabBox_student_B.DataTypeConversion10[0];

    /* Reshape: '<S64>/Reshape4' */
    Ball_and_Plate_MicroLabBox_student_B.Reshape4[0] =
      Ball_and_Plate_MicroLabBox_student_B.MathFunction1[0];

    /* Math: '<S64>/Math Function1' */
    Ball_and_Plate_MicroLabBox_student_B.MathFunction1[1] =
      Ball_and_Plate_MicroLabBox_student_B.DataTypeConversion10[1];

    /* Reshape: '<S64>/Reshape4' */
    Ball_and_Plate_MicroLabBox_student_B.Reshape4[1] =
      Ball_and_Plate_MicroLabBox_student_B.MathFunction1[1];

    /* DataTypeConversion: '<S64>/Data Type Conversion12' incorporates:
     *  Constant: '<S5>/du.wt_zero'
     */
    Ball_and_Plate_MicroLabBox_student_B.DataTypeConversion12[0] =
      Ball_and_Plate_MicroLabBox_student_P.duwt_zero_Value[0];
    Ball_and_Plate_MicroLabBox_student_B.DataTypeConversion12[1] =
      Ball_and_Plate_MicroLabBox_student_P.duwt_zero_Value[1];

    /* Math: '<S64>/Math Function2' */
    Ball_and_Plate_MicroLabBox_student_B.MathFunction2[0] =
      Ball_and_Plate_MicroLabBox_student_B.DataTypeConversion12[0];

    /* Reshape: '<S64>/Reshape5' */
    Ball_and_Plate_MicroLabBox_student_B.Reshape5[0] =
      Ball_and_Plate_MicroLabBox_student_B.MathFunction2[0];

    /* Math: '<S64>/Math Function2' */
    Ball_and_Plate_MicroLabBox_student_B.MathFunction2[1] =
      Ball_and_Plate_MicroLabBox_student_B.DataTypeConversion12[1];

    /* Reshape: '<S64>/Reshape5' */
    Ball_and_Plate_MicroLabBox_student_B.Reshape5[1] =
      Ball_and_Plate_MicroLabBox_student_B.MathFunction2[1];

    /* DataTypeConversion: '<S64>/Data Type Conversion11' incorporates:
     *  Constant: '<S5>/ecr.wt_zero'
     */
    Ball_and_Plate_MicroLabBox_student_B.DataTypeConversion11 =
      Ball_and_Plate_MicroLabBox_student_P.ecrwt_zero_Value;

    /* Memory: '<S64>/Memory' */
    memcpy(&Ball_and_Plate_MicroLabBox_student_B.Memory[0],
           &Ball_and_Plate_MicroLabBox_student_DW.Memory_PreviousInput[0], 212U *
           sizeof(boolean_T));

    /* MATLAB Function: '<S84>/optimizer' */
    /* MATLAB Function 'MPC Controller/MPC/optimizer/optimizer': '<S85>:1' */
    /* '<S85>:1:53' */
    /* '<S85>:1:139' */
    /* '<S85>:1:140' */
    /* '<S85>:1:143' */
    for (r1 = 0; r1 < 26; r1++) {
      vseq[r1] = 1.0;
    }

    memset(&rseq[0], 0, 100U * sizeof(real_T));
    for (r2 = 0; r2 < 4; r2++) {
      for (r1 = 0; r1 < 25; r1++) {
        rseq[r2 + r1 * (int32_T)Ball_and_Plate_MicroLabBox_s_ny] =
          Ball_and_Plate_MicroLabBox_student_B.DataTypeConversion1[25 * r2 + r1];
      }
    }

    /* '<S85>:1:58' */
    u_min[0] = Ball_and_Plate_MicroLabBox_student_B.last_mv[0];
    u_min[1] = Ball_and_Plate_MicroLabBox_student_B.last_mv[1];

    /* '<S85>:1:68' */
    Ball_and_Plate_MicroLabBox_student_B.xest[0] =
      Ball_and_Plate_MicroLabBox_student_B.moorxConversion[0];
    Ball_and_Plate_MicroLabBox_student_B.xest[1] =
      Ball_and_Plate_MicroLabBox_student_B.moorxConversion[1];
    Ball_and_Plate_MicroLabBox_student_B.xest[2] =
      Ball_and_Plate_MicroLabBox_student_B.moorxConversion[2];
    Ball_and_Plate_MicroLabBox_student_B.xest[3] =
      Ball_and_Plate_MicroLabBox_student_B.moorxConversion[3];

    /* '<S85>:1:137' */
    memcpy(&Ball_and_Plate_MicroLabBox_student_B.iAout[0],
           &Ball_and_Plate_MicroLabBox_student_B.Memory[0], 212U * sizeof
           (boolean_T));
    memset(&tmp[0], 0, 156U * sizeof(real_T));
    memset(&tmp_0[0], 0, 50U * sizeof(real_T));
    for (r1 = 0; r1 < 212; r1++) {
      center_idx_2 = b_Mx[r1] * Ball_and_Plate_MicroLabBox_student_B.xest[0];
      center_idx_2 += b_Mx[r1 + 212] *
        Ball_and_Plate_MicroLabBox_student_B.xest[1];
      center_idx_2 += b_Mx[r1 + 424] *
        Ball_and_Plate_MicroLabBox_student_B.xest[2];
      center_idx_2 += b_Mx[r1 + 636] *
        Ball_and_Plate_MicroLabBox_student_B.xest[3];
      b_Mlim_0[r1] = b_Mlim_1[r1] + center_idx_2;
    }

    for (r1 = 0; r1 < 212; r1++) {
      center_idx_2 = b_Mu1[r1] * u_min[0];
      center_idx_2 += b_Mu1[r1 + 212] * u_min[1];
      b_Mlim[r1] = -(b_Mlim_0[r1] + center_idx_2);
    }

    Ball_and_Plate_Micr_mpc_solveQP(Ball_and_Plate_MicroLabBox_student_B.xest,
      b_Kx, b_Kr, rseq, b_Ku1, u_min, tmp, vseq, b_Kut, tmp_0, b_Linv, b_Hinv,
      b_Ac, b_Mlim, Ball_and_Plate_MicroLabBox_student_B.iAout, zopt, f, &a21);
    Ball_and_Plate_MicroLabBox_student_B.u[0] = u_min[0] + zopt[0];
    Ball_and_Plate_MicroLabBox_student_B.u[1] = u_min[1] + zopt[1];

    /* '<S85>:1:137' */
    Ball_and_Plate_MicroLabBox_student_B.cost = 0.0;

    /* '<S85>:1:137' */
    memset(&Ball_and_Plate_MicroLabBox_student_B.useq[0], 0, 52U * sizeof(real_T));

    /* '<S85>:1:137' */
    Ball_and_Plate_MicroLabBox_student_B.status = a21;

    /* '<S85>:1:137' */
    /* '<S85>:1:153' */
    memset(&Ball_and_Plate_MicroLabBox_student_B.yseq[0], 0, 104U * sizeof
           (real_T));

    /* '<S85>:1:154' */
    memset(&Ball_and_Plate_MicroLabBox_student_B.xseq[0], 0, 104U * sizeof
           (real_T));

    /* '<S85>:1:163' */
    Ball_and_Plate_MicroLabBox_student_B.xk1[0] =
      Ball_and_Plate_MicroLabBox_student_B.xest[0];
    Ball_and_Plate_MicroLabBox_student_B.xk1[1] =
      Ball_and_Plate_MicroLabBox_student_B.xest[1];
    Ball_and_Plate_MicroLabBox_student_B.xk1[2] =
      Ball_and_Plate_MicroLabBox_student_B.xest[2];
    Ball_and_Plate_MicroLabBox_student_B.xk1[3] =
      Ball_and_Plate_MicroLabBox_student_B.xest[3];

    /* End of MATLAB Function: '<S84>/optimizer' */

    /* Gain: '<S64>/umin_scale1' */
    /* '<S85>:1:166' */
    Ball_and_Plate_MicroLabBox_student_B.umin_scale1[0] =
      Ball_and_Plate_MicroLabBox_student_P.umin_scale1_Gain[0] *
      Ball_and_Plate_MicroLabBox_student_B.u[0];
    Ball_and_Plate_MicroLabBox_student_B.umin_scale1[1] =
      Ball_and_Plate_MicroLabBox_student_P.umin_scale1_Gain[1] *
      Ball_and_Plate_MicroLabBox_student_B.u[1];
  }

  if (rtmIsMajorTimeStep(Ball_and_Plate_MicroLabBox_student_M) &&
      Ball_and_Plate_MicroLabBox_student_M->Timing.TaskCounters.TID[1] == 0) {
    /* Sum: '<Root>/Add3' */
    Ball_and_Plate_MicroLabBox_student_B.Add3[0] =
      Ball_and_Plate_MicroLabBox_student_B.Add4[0] -
      Ball_and_Plate_MicroLabBox_student_B.Switch[0];
    Ball_and_Plate_MicroLabBox_student_B.Add3[1] =
      Ball_and_Plate_MicroLabBox_student_B.Add4[1] -
      Ball_and_Plate_MicroLabBox_student_B.Switch[1];
    Ball_and_Plate_MicroLabBox_student_B.Add3[2] =
      Ball_and_Plate_MicroLabBox_student_B.Add4[2] -
      Ball_and_Plate_MicroLabBox_student_B.Switch[2];
    Ball_and_Plate_MicroLabBox_student_B.Add3[3] =
      Ball_and_Plate_MicroLabBox_student_B.Add4[3] -
      Ball_and_Plate_MicroLabBox_student_B.Switch[3];

    /* Gain: '<S164>/Proportional Gain' */
    Ball_and_Plate_MicroLabBox_student_B.ProportionalGain =
      Ball_and_Plate_MicroLabBox_student_P.PIDController1_P *
      Ball_and_Plate_MicroLabBox_student_B.Add3[2];

    /* DiscreteIntegrator: '<S159>/Integrator' incorporates:
     *  Constant: '<Root>/enable_outer_controller'
     */
    if ((Ball_and_Plate_MicroLabBox_student_P.enable_outer_controller_Value >
         0.0) &&
        (Ball_and_Plate_MicroLabBox_student_DW.Integrator_PrevResetState <= 0))
    {
      Ball_and_Plate_MicroLabBox_student_DW.Integrator_DSTATE =
        Ball_and_Plate_MicroLabBox_student_P.PIDController1_InitialConditi_b;
    }

    Ball_and_Plate_MicroLabBox_student_B.Integrator =
      Ball_and_Plate_MicroLabBox_student_DW.Integrator_DSTATE;

    /* End of DiscreteIntegrator: '<S159>/Integrator' */

    /* Gain: '<S153>/Derivative Gain' */
    Ball_and_Plate_MicroLabBox_student_B.DerivativeGain =
      Ball_and_Plate_MicroLabBox_student_P.PIDController1_D *
      Ball_and_Plate_MicroLabBox_student_B.Add3[2];

    /* DiscreteIntegrator: '<S154>/Filter' incorporates:
     *  Constant: '<Root>/enable_outer_controller'
     */
    if ((Ball_and_Plate_MicroLabBox_student_P.enable_outer_controller_Value >
         0.0) && (Ball_and_Plate_MicroLabBox_student_DW.Filter_PrevResetState <=
                  0)) {
      Ball_and_Plate_MicroLabBox_student_DW.Filter_DSTATE =
        Ball_and_Plate_MicroLabBox_student_P.PIDController1_InitialCondition;
    }

    Ball_and_Plate_MicroLabBox_student_B.Filter =
      Ball_and_Plate_MicroLabBox_student_DW.Filter_DSTATE;

    /* End of DiscreteIntegrator: '<S154>/Filter' */

    /* Sum: '<S154>/SumD' */
    Ball_and_Plate_MicroLabBox_student_B.SumD =
      Ball_and_Plate_MicroLabBox_student_B.DerivativeGain -
      Ball_and_Plate_MicroLabBox_student_B.Filter;

    /* Gain: '<S162>/Filter Coefficient' */
    Ball_and_Plate_MicroLabBox_student_B.FilterCoefficient =
      Ball_and_Plate_MicroLabBox_student_P.PIDController1_N *
      Ball_and_Plate_MicroLabBox_student_B.SumD;

    /* Gain: '<S120>/Proportional Gain' */
    Ball_and_Plate_MicroLabBox_student_B.ProportionalGain_e =
      Ball_and_Plate_MicroLabBox_student_P.PIDController_P *
      Ball_and_Plate_MicroLabBox_student_B.Add3[0];

    /* DiscreteIntegrator: '<S115>/Integrator' incorporates:
     *  Constant: '<Root>/enable_outer_controller'
     */
    if ((Ball_and_Plate_MicroLabBox_student_P.enable_outer_controller_Value >
         0.0) &&
        (Ball_and_Plate_MicroLabBox_student_DW.Integrator_PrevResetState_n <= 0))
    {
      Ball_and_Plate_MicroLabBox_student_DW.Integrator_DSTATE_n =
        Ball_and_Plate_MicroLabBox_student_P.PIDController_InitialConditio_l;
    }

    Ball_and_Plate_MicroLabBox_student_B.Integrator_l =
      Ball_and_Plate_MicroLabBox_student_DW.Integrator_DSTATE_n;

    /* End of DiscreteIntegrator: '<S115>/Integrator' */

    /* Gain: '<S109>/Derivative Gain' */
    Ball_and_Plate_MicroLabBox_student_B.DerivativeGain_j =
      Ball_and_Plate_MicroLabBox_student_P.PIDController_D *
      Ball_and_Plate_MicroLabBox_student_B.Add3[0];

    /* DiscreteIntegrator: '<S110>/Filter' incorporates:
     *  Constant: '<Root>/enable_outer_controller'
     */
    if ((Ball_and_Plate_MicroLabBox_student_P.enable_outer_controller_Value >
         0.0) && (Ball_and_Plate_MicroLabBox_student_DW.Filter_PrevResetState_h <=
                  0)) {
      Ball_and_Plate_MicroLabBox_student_DW.Filter_DSTATE_p =
        Ball_and_Plate_MicroLabBox_student_P.PIDController_InitialConditionF;
    }

    Ball_and_Plate_MicroLabBox_student_B.Filter_e =
      Ball_and_Plate_MicroLabBox_student_DW.Filter_DSTATE_p;

    /* End of DiscreteIntegrator: '<S110>/Filter' */

    /* Sum: '<S110>/SumD' */
    Ball_and_Plate_MicroLabBox_student_B.SumD_d =
      Ball_and_Plate_MicroLabBox_student_B.DerivativeGain_j -
      Ball_and_Plate_MicroLabBox_student_B.Filter_e;

    /* Gain: '<S118>/Filter Coefficient' */
    Ball_and_Plate_MicroLabBox_student_B.FilterCoefficient_l =
      Ball_and_Plate_MicroLabBox_student_P.PIDController_N *
      Ball_and_Plate_MicroLabBox_student_B.SumD_d;

    /* MultiPortSwitch: '<Root>/Multiport Switch' incorporates:
     *  Constant: '<Root>/Controller_select'
     */
    switch ((int32_T)
            Ball_and_Plate_MicroLabBox_student_P.Controller_select_Value) {
     case 1:
      /* Sum: '<Root>/Add' */
      Ball_and_Plate_MicroLabBox_student_B.Add_n[0] =
        Ball_and_Plate_MicroLabBox_student_B.Switch[0] -
        Ball_and_Plate_MicroLabBox_student_B.Add4[0];
      Ball_and_Plate_MicroLabBox_student_B.Add_n[1] =
        Ball_and_Plate_MicroLabBox_student_B.Switch[1] -
        Ball_and_Plate_MicroLabBox_student_B.Add4[1];
      Ball_and_Plate_MicroLabBox_student_B.Add_n[2] =
        Ball_and_Plate_MicroLabBox_student_B.Switch[2] -
        Ball_and_Plate_MicroLabBox_student_B.Add4[2];
      Ball_and_Plate_MicroLabBox_student_B.Add_n[3] =
        Ball_and_Plate_MicroLabBox_student_B.Switch[3] -
        Ball_and_Plate_MicroLabBox_student_B.Add4[3];

      /* Gain: '<Root>/Gain1' */
      for (r1 = 0; r1 < 2; r1++) {
        Ball_and_Plate_MicroLabBox_student_B.Gain1_j[r1] = 0.0;
        Ball_and_Plate_MicroLabBox_student_B.Gain1_j[r1] +=
          Ball_and_Plate_MicroLabBox_student_P.K_lqr[r1] *
          Ball_and_Plate_MicroLabBox_student_B.Add_n[0];
        Ball_and_Plate_MicroLabBox_student_B.Gain1_j[r1] +=
          Ball_and_Plate_MicroLabBox_student_P.K_lqr[r1 + 2] *
          Ball_and_Plate_MicroLabBox_student_B.Add_n[1];
        Ball_and_Plate_MicroLabBox_student_B.Gain1_j[r1] +=
          Ball_and_Plate_MicroLabBox_student_P.K_lqr[r1 + 4] *
          Ball_and_Plate_MicroLabBox_student_B.Add_n[2];
        Ball_and_Plate_MicroLabBox_student_B.Gain1_j[r1] +=
          Ball_and_Plate_MicroLabBox_student_P.K_lqr[r1 + 6] *
          Ball_and_Plate_MicroLabBox_student_B.Add_n[3];
      }

      /* End of Gain: '<Root>/Gain1' */
      Ball_and_Plate_MicroLabBox_student_B.MultiportSwitch[0] =
        Ball_and_Plate_MicroLabBox_student_B.Gain1_j[0];
      Ball_and_Plate_MicroLabBox_student_B.MultiportSwitch[1] =
        Ball_and_Plate_MicroLabBox_student_B.Gain1_j[1];
      break;

     case 2:
      Ball_and_Plate_MicroLabBox_student_B.MultiportSwitch[0] =
        Ball_and_Plate_MicroLabBox_student_B.RateLimiter[0];
      Ball_and_Plate_MicroLabBox_student_B.MultiportSwitch[1] =
        Ball_and_Plate_MicroLabBox_student_B.RateLimiter[1];
      break;

     case 3:
      Ball_and_Plate_MicroLabBox_student_B.MultiportSwitch[0] =
        Ball_and_Plate_MicroLabBox_student_B.umin_scale1[0];
      Ball_and_Plate_MicroLabBox_student_B.MultiportSwitch[1] =
        Ball_and_Plate_MicroLabBox_student_B.umin_scale1[1];
      break;

     default:
      /* Sum: '<S168>/Sum' */
      Ball_and_Plate_MicroLabBox_student_B.Sum_f =
        (Ball_and_Plate_MicroLabBox_student_B.ProportionalGain +
         Ball_and_Plate_MicroLabBox_student_B.Integrator) +
        Ball_and_Plate_MicroLabBox_student_B.FilterCoefficient;

      /* Gain: '<Root>/Gain8' */
      Ball_and_Plate_MicroLabBox_student_B.Gain8 =
        Ball_and_Plate_MicroLabBox_student_P.Gain8_Gain *
        Ball_and_Plate_MicroLabBox_student_B.Sum_f;

      /* Sum: '<S124>/Sum' */
      Ball_and_Plate_MicroLabBox_student_B.Sum_n =
        (Ball_and_Plate_MicroLabBox_student_B.ProportionalGain_e +
         Ball_and_Plate_MicroLabBox_student_B.Integrator_l) +
        Ball_and_Plate_MicroLabBox_student_B.FilterCoefficient_l;
      Ball_and_Plate_MicroLabBox_student_B.MultiportSwitch[0] =
        Ball_and_Plate_MicroLabBox_student_B.Gain8;
      Ball_and_Plate_MicroLabBox_student_B.MultiportSwitch[1] =
        Ball_and_Plate_MicroLabBox_student_B.Sum_n;
      break;
    }

    /* End of MultiPortSwitch: '<Root>/Multiport Switch' */

    /* Sum: '<Root>/Add5' */
    Ball_and_Plate_MicroLabBox_student_B.Add5[0] =
      Ball_and_Plate_MicroLabBox_student_B.asample_h[0] +
      Ball_and_Plate_MicroLabBox_student_B.asample[0];
    Ball_and_Plate_MicroLabBox_student_B.Add5[1] =
      Ball_and_Plate_MicroLabBox_student_B.asample_h[1] +
      Ball_and_Plate_MicroLabBox_student_B.asample[1];

    /* Switch: '<Root>/Switch1' incorporates:
     *  Constant: '<Root>/Constant7'
     *  Constant: '<Root>/ff_enable'
     */
    if (Ball_and_Plate_MicroLabBox_student_P.ff_enable_Value >
        Ball_and_Plate_MicroLabBox_student_P.Switch1_Threshold) {
      /* Gain: '<Root>/Gain_ff_x' */
      a21 = 7.0 / (5.0 * Ball_and_Plate_MicroLabBox_student_P.g);
      Ball_and_Plate_MicroLabBox_student_B.Gain_ff_x = a21 *
        Ball_and_Plate_MicroLabBox_student_B.Add5[0];
      Ball_and_Plate_MicroLabBox_student_B.Switch1 =
        Ball_and_Plate_MicroLabBox_student_B.Gain_ff_x;
    } else {
      Ball_and_Plate_MicroLabBox_student_B.Switch1 =
        Ball_and_Plate_MicroLabBox_student_P.Constant7_Value;
    }

    /* End of Switch: '<Root>/Switch1' */

    /* Sum: '<Root>/Add1' */
    Ball_and_Plate_MicroLabBox_student_B.Add1 =
      Ball_and_Plate_MicroLabBox_student_B.MultiportSwitch[0] +
      Ball_and_Plate_MicroLabBox_student_B.Switch1;

    /* Switch: '<Root>/Switch2' incorporates:
     *  Constant: '<Root>/Constant8'
     *  Constant: '<Root>/ff_enable'
     */
    if (Ball_and_Plate_MicroLabBox_student_P.ff_enable_Value >
        Ball_and_Plate_MicroLabBox_student_P.Switch2_Threshold) {
      /* Gain: '<Root>/Gain_ff_y ' */
      a21 = -7.0 / (5.0 * Ball_and_Plate_MicroLabBox_student_P.g);
      Ball_and_Plate_MicroLabBox_student_B.Gain_ff_y = a21 *
        Ball_and_Plate_MicroLabBox_student_B.Add5[1];
      Ball_and_Plate_MicroLabBox_student_B.Switch2 =
        Ball_and_Plate_MicroLabBox_student_B.Gain_ff_y;
    } else {
      Ball_and_Plate_MicroLabBox_student_B.Switch2 =
        Ball_and_Plate_MicroLabBox_student_P.Constant8_Value;
    }

    /* End of Switch: '<Root>/Switch2' */

    /* Sum: '<Root>/Add2' */
    Ball_and_Plate_MicroLabBox_student_B.Add2 =
      Ball_and_Plate_MicroLabBox_student_B.MultiportSwitch[1] +
      Ball_and_Plate_MicroLabBox_student_B.Switch2;

    /* Gain: '<Root>/Gain4' */
    Ball_and_Plate_MicroLabBox_student_B.Gain4 =
      Ball_and_Plate_MicroLabBox_student_P.Gain4_Gain *
      Ball_and_Plate_MicroLabBox_student_B.Add2;

    /* Gain: '<S11>/Gain1' */
    Ball_and_Plate_MicroLabBox_student_B.Gain1 =
      Ball_and_Plate_MicroLabBox_student_P.Gain1_Gain *
      Ball_and_Plate_MicroLabBox_student_B.Gain4;

    /* S-Function (dlowpass1): '<S11>/Dct1lowpass2' */

    /* Level2 S-Function Block: '<S11>/Dct1lowpass2' (dlowpass1) */
    {
      SimStruct *rts = Ball_and_Plate_MicroLabBox_student_M->childSfunctions[0];
      sfcnOutputs(rts,1);
    }

    /* Saturate: '<Root>/Alpha_sat ' */
    a21 = -Ball_and_Plate_MicroLabBox_student_P.plate_angle_sat;
    center_idx_2 = Ball_and_Plate_MicroLabBox_student_B.Dct1lowpass2;
    uX_idx_0 = Ball_and_Plate_MicroLabBox_student_P.plate_angle_sat;
    if (center_idx_2 > uX_idx_0) {
      Ball_and_Plate_MicroLabBox_student_B.Alpha_sat = uX_idx_0;
    } else if (center_idx_2 < a21) {
      Ball_and_Plate_MicroLabBox_student_B.Alpha_sat = a21;
    } else {
      Ball_and_Plate_MicroLabBox_student_B.Alpha_sat = center_idx_2;
    }

    /* End of Saturate: '<Root>/Alpha_sat ' */

    /* Gain: '<Root>/Gain5' */
    Ball_and_Plate_MicroLabBox_student_B.Gain5 =
      Ball_and_Plate_MicroLabBox_student_P.Gain5_Gain *
      Ball_and_Plate_MicroLabBox_student_B.Add1;

    /* Gain: '<S12>/Gain1' */
    Ball_and_Plate_MicroLabBox_student_B.Gain1_d =
      Ball_and_Plate_MicroLabBox_student_P.Gain1_Gain_b *
      Ball_and_Plate_MicroLabBox_student_B.Gain5;

    /* S-Function (dlowpass1): '<S12>/Dct1lowpass2' */

    /* Level2 S-Function Block: '<S12>/Dct1lowpass2' (dlowpass1) */
    {
      SimStruct *rts = Ball_and_Plate_MicroLabBox_student_M->childSfunctions[1];
      sfcnOutputs(rts,1);
    }

    /* Saturate: '<Root>/Beta_sat ' */
    a21 = -Ball_and_Plate_MicroLabBox_student_P.plate_angle_sat;
    center_idx_2 = Ball_and_Plate_MicroLabBox_student_B.Dct1lowpass2_g;
    uX_idx_0 = Ball_and_Plate_MicroLabBox_student_P.plate_angle_sat;
    if (center_idx_2 > uX_idx_0) {
      Ball_and_Plate_MicroLabBox_student_B.Beta_sat = uX_idx_0;
    } else if (center_idx_2 < a21) {
      Ball_and_Plate_MicroLabBox_student_B.Beta_sat = a21;
    } else {
      Ball_and_Plate_MicroLabBox_student_B.Beta_sat = center_idx_2;
    }

    /* End of Saturate: '<Root>/Beta_sat ' */

    /* Gain: '<Root>/Gain2' */
    Ball_and_Plate_MicroLabBox_student_B.Gain2_j =
      Ball_and_Plate_MicroLabBox_student_P.Gain2_Gain_p *
      Ball_and_Plate_MicroLabBox_student_B.ByteUnpacking_o1;

    /* Gain: '<Root>/Gain3' */
    Ball_and_Plate_MicroLabBox_student_B.Gain3 =
      Ball_and_Plate_MicroLabBox_student_P.Gain3_Gain *
      Ball_and_Plate_MicroLabBox_student_B.ByteUnpacking_o2;
  }

  /* RateTransition: '<Root>/Rate Transition4' incorporates:
   *  RateTransition: '<Root>/Rate Transition5'
   */
  if (rtmIsMajorTimeStep(Ball_and_Plate_MicroLabBox_student_M) &&
      Ball_and_Plate_MicroLabBox_student_M->Timing.TaskCounters.TID[2] == 0) {
    Ball_and_Plate_MicroLabBox_student_B.RateTransition4 =
      Ball_and_Plate_MicroLabBox_student_B.Gain2_j;

    /* DataTypeConversion: '<Root>/Cast To Double' */
    Ball_and_Plate_MicroLabBox_student_B.CastToDouble =
      Ball_and_Plate_MicroLabBox_student_B.RateTransition4;
    Ball_and_Plate_MicroLabBox_student_B.RateTransition5 =
      Ball_and_Plate_MicroLabBox_student_B.Gain3;

    /* DataTypeConversion: '<Root>/Cast To Double1' */
    Ball_and_Plate_MicroLabBox_student_B.CastToDouble1 =
      Ball_and_Plate_MicroLabBox_student_B.RateTransition5;
  }

  /* End of RateTransition: '<Root>/Rate Transition4' */
  if (rtmIsMajorTimeStep(Ball_and_Plate_MicroLabBox_student_M) &&
      Ball_and_Plate_MicroLabBox_student_M->Timing.TaskCounters.TID[1] == 0) {
    /* Delay: '<Root>/Delay' */
    Ball_and_Plate_MicroLabBox_student_B.Delay[0] =
      Ball_and_Plate_MicroLabBox_student_DW.Delay_DSTATE[0];
    Ball_and_Plate_MicroLabBox_student_B.Delay[1] =
      Ball_and_Plate_MicroLabBox_student_DW.Delay_DSTATE[1];

    /* S-Function (rti_commonblock): '<S15>/S-Function1' */
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

    /* S-Function (rti_commonblock): '<S17>/S-Function1' */
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
     *  Constant: '<Root>/Psi_ref '
     */
    /* MATLAB Function 'Innerloop_Actuator/AngleToPos ': '<S18>:1' */
    /* '<S18>:1:12' */
    /* '<S18>:1:16' */
    /* '<S18>:1:17' */
    /* '<S18>:1:18' */
    /* '<S18>:1:23' */
    /* '<S18>:1:7' */
    /* '<S18>:1:27' */
    /* '<S18>:1:31' */
    /* '<S18>:1:19' */
    center_idx_2 = cos(Ball_and_Plate_MicroLabBox_student_P.Psi_ref_Value);
    a21 = sin(Ball_and_Plate_MicroLabBox_student_P.Psi_ref_Value);
    uX_idx_0 = sin(Ball_and_Plate_MicroLabBox_student_P.Psi_ref_Value);
    uX_idx_1 = cos(Ball_and_Plate_MicroLabBox_student_P.Psi_ref_Value);
    uX_idx_2 = cos(Ball_and_Plate_MicroLabBox_student_B.Beta_sat);
    tmp_1 = sin(Ball_and_Plate_MicroLabBox_student_B.Beta_sat);
    tmp_2 = sin(Ball_and_Plate_MicroLabBox_student_B.Beta_sat);
    tmp_3 = cos(Ball_and_Plate_MicroLabBox_student_B.Beta_sat);
    tmp_4[0] = center_idx_2;
    tmp_4[3] = -a21;
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
    center_idx_2 = cos(Ball_and_Plate_MicroLabBox_student_B.Alpha_sat);
    a21 = sin(Ball_and_Plate_MicroLabBox_student_B.Alpha_sat);
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
    e[4] = center_idx_2;
    e[7] = -a21;
    e[2] = 0.0;
    e[5] = uX_idx_0;
    e[8] = uX_idx_1;

    /* '<S18>:1:23' */
    /* '<S18>:1:24' */
    for (r1 = 0; r1 < 3; r1++) {
      a21 = T_1[r1];
      center_idx_2 = 0.0;
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
        center_idx_2 += R_BtoP[3 * r2 + r1] * b[r2];
      }

      P1_global[r1] = (a21 + center_idx_2) - b[r1];
      center_idx_2 = R_BtoP[r1] * -0.085000000000000075;
      center_idx_2 += R_BtoP[r1 + 3] * -0.14722431864335456;
      center_idx_2 += R_BtoP[r1 + 6] * 0.0;
      T_0[r1] = (a21 + center_idx_2) - c[r1];
      center_idx_2 = R_BtoP[r1] * -0.084999999999999964;
      center_idx_2 += R_BtoP[r1 + 3] * 0.14722431864335458;
      center_idx_2 += R_BtoP[r1 + 6] * 0.0;
      T[r1] = (a21 + center_idx_2) - d[r1];
    }

    Ball_and_Plate_MicroLabBox_student_B.pos1 = Ball_and_Plate_MicroLabBox_norm
      (P1_global);

    /* '<S18>:1:27' */
    /* '<S18>:1:28' */
    Ball_and_Plate_MicroLabBox_student_B.pos2 = Ball_and_Plate_MicroLabBox_norm
      (T_0);

    /* '<S18>:1:31' */
    /* '<S18>:1:32' */
    Ball_and_Plate_MicroLabBox_student_B.pos3 = Ball_and_Plate_MicroLabBox_norm
      (T);

    /* End of MATLAB Function: '<S3>/AngleToPos ' */

    /* Sum: '<S3>/Add1' incorporates:
     *  Constant: '<S3>/Constant'
     */
    Ball_and_Plate_MicroLabBox_student_B.Add1_m =
      Ball_and_Plate_MicroLabBox_student_B.pos1 -
      Ball_and_Plate_MicroLabBox_student_P.Constant_Value_o;

    /* Switch: '<S19>/enable_ref ' incorporates:
     *  Constant: '<S19>/Constant2'
     *  Constant: '<S19>/Constant4'
     *  Constant: '<S19>/step'
     */
    if (Ball_and_Plate_MicroLabBox_student_P.step_Value >
        Ball_and_Plate_MicroLabBox_student_P.enable_ref_Threshold) {
      Ball_and_Plate_MicroLabBox_student_B.enable_ref =
        Ball_and_Plate_MicroLabBox_student_P.Constant4_Value;
    } else {
      Ball_and_Plate_MicroLabBox_student_B.enable_ref =
        Ball_and_Plate_MicroLabBox_student_P.Constant2_Value;
    }

    /* End of Switch: '<S19>/enable_ref ' */

    /* Sum: '<S19>/Sum1' */
    Ball_and_Plate_MicroLabBox_student_B.Sum1 =
      Ball_and_Plate_MicroLabBox_student_B.enable_ref -
      Ball_and_Plate_MicroLabBox_student_B.mm2m_po;

    /* Gain: '<S27>/Gain1' */
    Ball_and_Plate_MicroLabBox_student_B.Gain1_b =
      Ball_and_Plate_MicroLabBox_student_P.Gain1_Gain_j *
      Ball_and_Plate_MicroLabBox_student_B.Sum1;

    /* S-Function (dleadlag): '<S27>/Dctleadlag2' */

    /* Level2 S-Function Block: '<S27>/Dctleadlag2' (dleadlag) */
    {
      SimStruct *rts = Ball_and_Plate_MicroLabBox_student_M->childSfunctions[2];
      sfcnOutputs(rts,1);
    }

    /* S-Function (dlowpass1): '<S27>/Dct1lowpass3' */

    /* Level2 S-Function Block: '<S27>/Dct1lowpass3' (dlowpass1) */
    {
      SimStruct *rts = Ball_and_Plate_MicroLabBox_student_M->childSfunctions[3];
      sfcnOutputs(rts,1);
    }

    /* DiscreteIntegrator: '<S27>/Discrete-Time Integrator' incorporates:
     *  Constant: '<S3>/enable_inner_controller'
     */
    if ((Ball_and_Plate_MicroLabBox_student_P.enable_inner_controller_Value >
         0.0) &&
        (Ball_and_Plate_MicroLabBox_student_DW.DiscreteTimeIntegrator_PrevRese <=
         0)) {
      Ball_and_Plate_MicroLabBox_student_DW.DiscreteTimeIntegrator_DSTATE =
        Ball_and_Plate_MicroLabBox_student_P.DiscreteTimeIntegrator_IC;
    }

    if (Ball_and_Plate_MicroLabBox_student_DW.DiscreteTimeIntegrator_DSTATE >=
        Ball_and_Plate_MicroLabBox_student_P.inner_I_sat) {
      Ball_and_Plate_MicroLabBox_student_DW.DiscreteTimeIntegrator_DSTATE =
        Ball_and_Plate_MicroLabBox_student_P.inner_I_sat;
    } else {
      a21 = -Ball_and_Plate_MicroLabBox_student_P.inner_I_sat;
      if (Ball_and_Plate_MicroLabBox_student_DW.DiscreteTimeIntegrator_DSTATE <=
          a21) {
        Ball_and_Plate_MicroLabBox_student_DW.DiscreteTimeIntegrator_DSTATE =
          -Ball_and_Plate_MicroLabBox_student_P.inner_I_sat;
      }
    }

    Ball_and_Plate_MicroLabBox_student_B.DiscreteTimeIntegrator =
      Ball_and_Plate_MicroLabBox_student_DW.DiscreteTimeIntegrator_DSTATE;

    /* End of DiscreteIntegrator: '<S27>/Discrete-Time Integrator' */

    /* Switch: '<S19>/Switch2' incorporates:
     *  Constant: '<S19>/Constant'
     *  Constant: '<S3>/enable_inner_controller'
     */
    if (Ball_and_Plate_MicroLabBox_student_P.enable_inner_controller_Value >=
        Ball_and_Plate_MicroLabBox_student_P.Switch2_Threshold_g) {
      /* Sum: '<S27>/Add' */
      Ball_and_Plate_MicroLabBox_student_B.Add_h =
        Ball_and_Plate_MicroLabBox_student_B.Dct1lowpass3 +
        Ball_and_Plate_MicroLabBox_student_B.DiscreteTimeIntegrator;
      Ball_and_Plate_MicroLabBox_student_B.Switch2_k =
        Ball_and_Plate_MicroLabBox_student_B.Add_h;
    } else {
      Ball_and_Plate_MicroLabBox_student_B.Switch2_k =
        Ball_and_Plate_MicroLabBox_student_P.Constant_Value;
    }

    /* End of Switch: '<S19>/Switch2' */

    /* UniformRandomNumber: '<S19>/Uniform Random Number' */
    Ball_and_Plate_MicroLabBox_student_B.UniformRandomNumber =
      Ball_and_Plate_MicroLabBox_student_DW.UniformRandomNumber_NextOutput;

    /* Gain: '<S19>/enable_white_noise_A ' */
    Ball_and_Plate_MicroLabBox_student_B.dA =
      Ball_and_Plate_MicroLabBox_student_P.enable_white_noise_A_Gain *
      Ball_and_Plate_MicroLabBox_student_B.UniformRandomNumber;

    /* Sum: '<S19>/Sum' */
    Ball_and_Plate_MicroLabBox_student_B.uA =
      Ball_and_Plate_MicroLabBox_student_B.Switch2_k +
      Ball_and_Plate_MicroLabBox_student_B.dA;
  }

  /* Switch: '<S19>/Switch3' incorporates:
   *  Constant: '<S3>/enable_inner_closeLoop'
   */
  if (Ball_and_Plate_MicroLabBox_student_P.enable_inner_closeLoop_Value >=
      Ball_and_Plate_MicroLabBox_student_P.Switch3_Threshold) {
    Ball_and_Plate_MicroLabBox_student_B.Switch3 =
      Ball_and_Plate_MicroLabBox_student_B.uA;
  } else {
    Ball_and_Plate_MicroLabBox_student_B.Switch3 =
      Ball_and_Plate_MicroLabBox_student_B.TransferFcn_f;
  }

  /* End of Switch: '<S19>/Switch3' */

  /* Outputs for Atomic SubSystem: '<S19>/Outputs to Amplifier' */

  /* Saturate: '<S25>/Saturation' */
  center_idx_2 = Ball_and_Plate_MicroLabBox_student_B.Switch3;
  uX_idx_1 = Ball_and_Plate_MicroLabBox_student_P.Saturation_LowerSat;
  uX_idx_0 = Ball_and_Plate_MicroLabBox_student_P.Saturation_UpperSat;
  if (center_idx_2 > uX_idx_0) {
    Ball_and_Plate_MicroLabBox_student_B.Saturation_a = uX_idx_0;
  } else if (center_idx_2 < uX_idx_1) {
    Ball_and_Plate_MicroLabBox_student_B.Saturation_a = uX_idx_1;
  } else {
    Ball_and_Plate_MicroLabBox_student_B.Saturation_a = center_idx_2;
  }

  /* End of Saturate: '<S25>/Saturation' */

  /* Gain: '<S25>/Current2V' */
  Ball_and_Plate_MicroLabBox_student_B.Current2V_h =
    Ball_and_Plate_MicroLabBox_student_P.Current2V_Gain *
    Ball_and_Plate_MicroLabBox_student_B.Saturation_a;

  /* Gain: '<S25>/DSPscale' */
  Ball_and_Plate_MicroLabBox_student_B.DSPscale_m =
    Ball_and_Plate_MicroLabBox_student_P.DSPscale_Gain *
    Ball_and_Plate_MicroLabBox_student_B.Current2V_h;
  if (rtmIsMajorTimeStep(Ball_and_Plate_MicroLabBox_student_M) &&
      Ball_and_Plate_MicroLabBox_student_M->Timing.TaskCounters.TID[1] == 0) {
    /* S-Function (rti_commonblock): '<S29>/S-Function1' */
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

  /* End of Outputs for SubSystem: '<S19>/Outputs to Amplifier' */
  if (rtmIsMajorTimeStep(Ball_and_Plate_MicroLabBox_student_M) &&
      Ball_and_Plate_MicroLabBox_student_M->Timing.TaskCounters.TID[1] == 0) {
    /* Switch: '<S19>/Switch' incorporates:
     *  Constant: '<S19>/Constant3'
     *  Constant: '<S3>/enable_quintic  '
     *  Constant: '<S3>/quintic_ref '
     */
    if (Ball_and_Plate_MicroLabBox_student_P.enable_quintic_Value >
        Ball_and_Plate_MicroLabBox_student_P.Switch_Threshold_j) {
      Ball_and_Plate_MicroLabBox_student_B.Switch_h =
        Ball_and_Plate_MicroLabBox_student_P.quintic_ref_Value;
    } else {
      Ball_and_Plate_MicroLabBox_student_B.Switch_h =
        Ball_and_Plate_MicroLabBox_student_P.Constant3_Value_d;
    }

    /* End of Switch: '<S19>/Switch' */

    /* MATLAB Function: '<S19>/MATLAB Function1' incorporates:
     *  Constant: '<S19>/Constant1'
     *  Constant: '<S19>/end_time_A'
     */
    Ball_and_Plate__MATLABFunction1
      (Ball_and_Plate_MicroLabBox_student_B.Switch_h,
       Ball_and_Plate_MicroLabBox_student_P.end_time_A_Value,
       Ball_and_Plate_MicroLabBox_student_P.Constant1_Value_na,
       &Ball_and_Plate_MicroLabBox_student_B.sf_MATLABFunction1,
       &Ball_and_Plate_MicroLabBox_student_DW.sf_MATLABFunction1);

    /* MATLAB Function: '<S24>/MATLAB Function' incorporates:
     *  Constant: '<S19>/enable_ID_A '
     *  Constant: '<S24>/Constant'
     */
    Ball_and_Plate_M_MATLABFunction
      (Ball_and_Plate_MicroLabBox_student_P.enable_ID_A_Value,
       Ball_and_Plate_MicroLabBox_student_P.uA,
       &Ball_and_Plate_MicroLabBox_student_B.sf_MATLABFunction,
       &Ball_and_Plate_MicroLabBox_student_DW.sf_MATLABFunction);

    /* Switch: '<S20>/Switch1' incorporates:
     *  Constant: '<S20>/Constant5'
     *  Constant: '<S3>/enable_quintic  '
     *  Constant: '<S3>/quintic_ref '
     */
    if (Ball_and_Plate_MicroLabBox_student_P.enable_quintic_Value >
        Ball_and_Plate_MicroLabBox_student_P.Switch1_Threshold_j) {
      Ball_and_Plate_MicroLabBox_student_B.Switch1_j =
        Ball_and_Plate_MicroLabBox_student_P.quintic_ref_Value;
    } else {
      Ball_and_Plate_MicroLabBox_student_B.Switch1_j =
        Ball_and_Plate_MicroLabBox_student_P.Constant5_Value;
    }

    /* End of Switch: '<S20>/Switch1' */

    /* MATLAB Function: '<S20>/MATLAB Function2' incorporates:
     *  Constant: '<S20>/Constant2'
     *  Constant: '<S20>/end_time_b '
     */
    Ball_and_Plate__MATLABFunction1
      (Ball_and_Plate_MicroLabBox_student_B.Switch1_j,
       Ball_and_Plate_MicroLabBox_student_P.end_time_b_Value,
       Ball_and_Plate_MicroLabBox_student_P.Constant2_Value_c,
       &Ball_and_Plate_MicroLabBox_student_B.sf_MATLABFunction2_d,
       &Ball_and_Plate_MicroLabBox_student_DW.sf_MATLABFunction2_d);

    /* Switch: '<S20>/Switch' incorporates:
     *  Constant: '<S3>/enable_outerloop '
     */
    if (Ball_and_Plate_MicroLabBox_student_P.enable_outerloop_Value >
        Ball_and_Plate_MicroLabBox_student_P.Switch_Threshold_a) {
      /* Sum: '<S3>/Add2' incorporates:
       *  Constant: '<S3>/Constant1'
       */
      Ball_and_Plate_MicroLabBox_student_B.Add2_h =
        Ball_and_Plate_MicroLabBox_student_B.pos2 -
        Ball_and_Plate_MicroLabBox_student_P.Constant1_Value_n;
      Ball_and_Plate_MicroLabBox_student_B.Switch_j =
        Ball_and_Plate_MicroLabBox_student_B.Add2_h;
    } else {
      Ball_and_Plate_MicroLabBox_student_B.Switch_j =
        Ball_and_Plate_MicroLabBox_student_B.sf_MATLABFunction2_d.path;
    }

    /* End of Switch: '<S20>/Switch' */

    /* Sum: '<S20>/Sum1' */
    Ball_and_Plate_MicroLabBox_student_B.Sum1_f =
      Ball_and_Plate_MicroLabBox_student_B.Switch_j -
      Ball_and_Plate_MicroLabBox_student_B.mm2m_p;

    /* Gain: '<S40>/Gain1' */
    Ball_and_Plate_MicroLabBox_student_B.Gain1_f =
      Ball_and_Plate_MicroLabBox_student_P.Gain1_Gain_l *
      Ball_and_Plate_MicroLabBox_student_B.Sum1_f;

    /* S-Function (dleadlag): '<S40>/Dctleadlag2' */

    /* Level2 S-Function Block: '<S40>/Dctleadlag2' (dleadlag) */
    {
      SimStruct *rts = Ball_and_Plate_MicroLabBox_student_M->childSfunctions[4];
      sfcnOutputs(rts,1);
    }

    /* S-Function (dlowpass1): '<S40>/Dct1lowpass3' */

    /* Level2 S-Function Block: '<S40>/Dct1lowpass3' (dlowpass1) */
    {
      SimStruct *rts = Ball_and_Plate_MicroLabBox_student_M->childSfunctions[5];
      sfcnOutputs(rts,1);
    }

    /* DiscreteIntegrator: '<S40>/Discrete-Time Integrator' incorporates:
     *  Constant: '<S3>/enable_inner_controller'
     */
    if ((Ball_and_Plate_MicroLabBox_student_P.enable_inner_controller_Value >
         0.0) &&
        (Ball_and_Plate_MicroLabBox_student_DW.DiscreteTimeIntegrator_PrevRe_h <=
         0)) {
      Ball_and_Plate_MicroLabBox_student_DW.DiscreteTimeIntegrator_DSTATE_p =
        Ball_and_Plate_MicroLabBox_student_P.DiscreteTimeIntegrator_IC_m;
    }

    if (Ball_and_Plate_MicroLabBox_student_DW.DiscreteTimeIntegrator_DSTATE_p >=
        Ball_and_Plate_MicroLabBox_student_P.inner_I_sat) {
      Ball_and_Plate_MicroLabBox_student_DW.DiscreteTimeIntegrator_DSTATE_p =
        Ball_and_Plate_MicroLabBox_student_P.inner_I_sat;
    } else {
      a21 = -Ball_and_Plate_MicroLabBox_student_P.inner_I_sat;
      if (Ball_and_Plate_MicroLabBox_student_DW.DiscreteTimeIntegrator_DSTATE_p <=
          a21) {
        Ball_and_Plate_MicroLabBox_student_DW.DiscreteTimeIntegrator_DSTATE_p =
          -Ball_and_Plate_MicroLabBox_student_P.inner_I_sat;
      }
    }

    Ball_and_Plate_MicroLabBox_student_B.DiscreteTimeIntegrator_m =
      Ball_and_Plate_MicroLabBox_student_DW.DiscreteTimeIntegrator_DSTATE_p;

    /* End of DiscreteIntegrator: '<S40>/Discrete-Time Integrator' */

    /* Switch: '<S20>/Switch2' incorporates:
     *  Constant: '<S20>/Constant1'
     *  Constant: '<S3>/enable_inner_controller'
     */
    if (Ball_and_Plate_MicroLabBox_student_P.enable_inner_controller_Value >=
        Ball_and_Plate_MicroLabBox_student_P.Switch2_Threshold_k) {
      /* Sum: '<S40>/Add' */
      Ball_and_Plate_MicroLabBox_student_B.Add_f =
        Ball_and_Plate_MicroLabBox_student_B.Dct1lowpass3_c +
        Ball_and_Plate_MicroLabBox_student_B.DiscreteTimeIntegrator_m;
      Ball_and_Plate_MicroLabBox_student_B.Switch2_p =
        Ball_and_Plate_MicroLabBox_student_B.Add_f;
    } else {
      Ball_and_Plate_MicroLabBox_student_B.Switch2_p =
        Ball_and_Plate_MicroLabBox_student_P.Constant1_Value;
    }

    /* End of Switch: '<S20>/Switch2' */

    /* UniformRandomNumber: '<S20>/Uniform Random Number' */
    Ball_and_Plate_MicroLabBox_student_B.UniformRandomNumber_j =
      Ball_and_Plate_MicroLabBox_student_DW.UniformRandomNumber_NextOutpu_n;

    /* Gain: '<S20>/enable_white_noise_B ' */
    Ball_and_Plate_MicroLabBox_student_B.dB =
      Ball_and_Plate_MicroLabBox_student_P.enable_white_noise_B_Gain *
      Ball_and_Plate_MicroLabBox_student_B.UniformRandomNumber_j;

    /* Sum: '<S20>/Sum' */
    Ball_and_Plate_MicroLabBox_student_B.uB =
      Ball_and_Plate_MicroLabBox_student_B.Switch2_p +
      Ball_and_Plate_MicroLabBox_student_B.dB;
  }

  /* Switch: '<S20>/Switch3' incorporates:
   *  Constant: '<S3>/enable_inner_closeLoop'
   */
  if (Ball_and_Plate_MicroLabBox_student_P.enable_inner_closeLoop_Value >=
      Ball_and_Plate_MicroLabBox_student_P.Switch3_Threshold_m) {
    Ball_and_Plate_MicroLabBox_student_B.Switch3_d =
      Ball_and_Plate_MicroLabBox_student_B.uB;
  } else {
    Ball_and_Plate_MicroLabBox_student_B.Switch3_d =
      Ball_and_Plate_MicroLabBox_student_B.TransferFcn_o;
  }

  /* End of Switch: '<S20>/Switch3' */

  /* Outputs for Atomic SubSystem: '<S20>/Outputs to Amplifier' */

  /* Saturate: '<S38>/Saturation' */
  center_idx_2 = Ball_and_Plate_MicroLabBox_student_B.Switch3_d;
  uX_idx_1 = Ball_and_Plate_MicroLabBox_student_P.Saturation_LowerSat_f;
  uX_idx_0 = Ball_and_Plate_MicroLabBox_student_P.Saturation_UpperSat_e;
  if (center_idx_2 > uX_idx_0) {
    Ball_and_Plate_MicroLabBox_student_B.Saturation_l = uX_idx_0;
  } else if (center_idx_2 < uX_idx_1) {
    Ball_and_Plate_MicroLabBox_student_B.Saturation_l = uX_idx_1;
  } else {
    Ball_and_Plate_MicroLabBox_student_B.Saturation_l = center_idx_2;
  }

  /* End of Saturate: '<S38>/Saturation' */

  /* Gain: '<S38>/Current2V' */
  Ball_and_Plate_MicroLabBox_student_B.Current2V_f =
    Ball_and_Plate_MicroLabBox_student_P.Current2V_Gain_f *
    Ball_and_Plate_MicroLabBox_student_B.Saturation_l;

  /* Gain: '<S38>/DSPscale' */
  Ball_and_Plate_MicroLabBox_student_B.DSPscale_o =
    Ball_and_Plate_MicroLabBox_student_P.DSPscale_Gain_d *
    Ball_and_Plate_MicroLabBox_student_B.Current2V_f;
  if (rtmIsMajorTimeStep(Ball_and_Plate_MicroLabBox_student_M) &&
      Ball_and_Plate_MicroLabBox_student_M->Timing.TaskCounters.TID[1] == 0) {
    /* S-Function (rti_commonblock): '<S44>/S-Function1' */
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

  /* End of Outputs for SubSystem: '<S20>/Outputs to Amplifier' */
  if (rtmIsMajorTimeStep(Ball_and_Plate_MicroLabBox_student_M) &&
      Ball_and_Plate_MicroLabBox_student_M->Timing.TaskCounters.TID[1] == 0) {
    /* MATLAB Function: '<S35>/MATLAB Function' */
    /* MATLAB Function 'Innerloop_Actuator/Motor_B/Init_motion/MATLAB Function': '<S41>:1' */
    /* '<S41>:1:14' */
    /* '<S41>:1:36' */
    Ball_and_Plate_MicroLabBox_student_B.y = -0.0289;

    /* '<S41>:1:39' */
    Ball_and_Plate_MicroLabBox_student_DW.previous_enable = 0.0;

    /* MATLAB Function: '<S35>/MATLAB Function2' incorporates:
     *  Constant: '<S20>/1_no_0_init_motion'
     *  Constant: '<S20>/Constant4'
     */
    /* MATLAB Function 'Innerloop_Actuator/Motor_B/Init_motion/MATLAB Function2': '<S42>:1' */
    if (Ball_and_Plate_MicroLabBox_student_P.u_no_0_init_motion_Value > 0.5) {
      /* '<S42>:1:3' */
      /* '<S42>:1:4' */
      Ball_and_Plate_MicroLabBox_student_B.path =
        Ball_and_Plate_MicroLabBox_student_P.Constant4_Value_b;
    } else if (Ball_and_Plate_MicroLabBox_student_P.u_no_0_init_motion_Value <
               0.5) {
      /* '<S42>:1:5' */
      /* '<S42>:1:6' */
      Ball_and_Plate_MicroLabBox_student_B.path = 0.0;
    } else {
      /* '<S42>:1:8' */
      Ball_and_Plate_MicroLabBox_student_B.path = 0.0;
    }

    /* End of MATLAB Function: '<S35>/MATLAB Function2' */

    /* Switch: '<S35>/Switch' incorporates:
     *  Constant: '<S20>/1_no_0_init_motion'
     *  Constant: '<S20>/Constant4'
     *  Constant: '<S35>/Constant3'
     */
    if (Ball_and_Plate_MicroLabBox_student_P.u_no_0_init_motion_Value >
        Ball_and_Plate_MicroLabBox_student_P.Switch_Threshold_k) {
      Ball_and_Plate_MicroLabBox_student_B.Switch_n =
        Ball_and_Plate_MicroLabBox_student_P.Constant4_Value_b;
    } else {
      Ball_and_Plate_MicroLabBox_student_B.Switch_n =
        Ball_and_Plate_MicroLabBox_student_P.Constant3_Value_i;
    }

    /* End of Switch: '<S35>/Switch' */

    /* Constant: '<S35>/Constant1' */
    Ball_and_Plate_MicroLabBox_student_B.Constant1 =
      Ball_and_Plate_MicroLabBox_student_P.Constant1_Value_p;

    /* Constant: '<S35>/Constant2' */
    Ball_and_Plate_MicroLabBox_student_B.Constant2 =
      Ball_and_Plate_MicroLabBox_student_P.Constant2_Value_g;

    /* MATLAB Function: '<S37>/MATLAB Function' incorporates:
     *  Constant: '<S20>/enable_ID_B '
     *  Constant: '<S37>/Constant'
     */
    Ball_and_Plate_M_MATLABFunction
      (Ball_and_Plate_MicroLabBox_student_P.enable_ID_B_Value,
       Ball_and_Plate_MicroLabBox_student_P.uB,
       &Ball_and_Plate_MicroLabBox_student_B.sf_MATLABFunction_ko,
       &Ball_and_Plate_MicroLabBox_student_DW.sf_MATLABFunction_ko);

    /* RandomNumber: '<S20>/Random Number' */
    Ball_and_Plate_MicroLabBox_student_B.RandomNumber =
      Ball_and_Plate_MicroLabBox_student_DW.NextOutput;

    /* Switch: '<S21>/Switch1' incorporates:
     *  Constant: '<S21>/Constant3'
     *  Constant: '<S3>/enable_quintic  '
     *  Constant: '<S3>/quintic_ref '
     */
    if (Ball_and_Plate_MicroLabBox_student_P.enable_quintic_Value >
        Ball_and_Plate_MicroLabBox_student_P.Switch1_Threshold_jm) {
      Ball_and_Plate_MicroLabBox_student_B.Switch1_k =
        Ball_and_Plate_MicroLabBox_student_P.quintic_ref_Value;
    } else {
      Ball_and_Plate_MicroLabBox_student_B.Switch1_k =
        Ball_and_Plate_MicroLabBox_student_P.Constant3_Value_n;
    }

    /* End of Switch: '<S21>/Switch1' */

    /* MATLAB Function: '<S21>/MATLAB Function1' incorporates:
     *  Constant: '<S21>/Constant1'
     *  Constant: '<S21>/end_time_c '
     */
    Ball_and_Plate__MATLABFunction1
      (Ball_and_Plate_MicroLabBox_student_B.Switch1_k,
       Ball_and_Plate_MicroLabBox_student_P.end_time_c_Value,
       Ball_and_Plate_MicroLabBox_student_P.Constant1_Value_c,
       &Ball_and_Plate_MicroLabBox_student_B.sf_MATLABFunction1_h,
       &Ball_and_Plate_MicroLabBox_student_DW.sf_MATLABFunction1_h);

    /* Switch: '<S21>/Switch' incorporates:
     *  Constant: '<S3>/enable_outerloop '
     */
    if (Ball_and_Plate_MicroLabBox_student_P.enable_outerloop_Value >
        Ball_and_Plate_MicroLabBox_student_P.Switch_Threshold_e) {
      /* Sum: '<S3>/Add3' incorporates:
       *  Constant: '<S3>/Constant2'
       */
      Ball_and_Plate_MicroLabBox_student_B.Add3_n =
        Ball_and_Plate_MicroLabBox_student_B.pos3 -
        Ball_and_Plate_MicroLabBox_student_P.Constant2_Value_n;
      Ball_and_Plate_MicroLabBox_student_B.Switch_jo =
        Ball_and_Plate_MicroLabBox_student_B.Add3_n;
    } else {
      Ball_and_Plate_MicroLabBox_student_B.Switch_jo =
        Ball_and_Plate_MicroLabBox_student_B.sf_MATLABFunction1_h.path;
    }

    /* End of Switch: '<S21>/Switch' */

    /* Sum: '<S21>/Sum1' */
    Ball_and_Plate_MicroLabBox_student_B.Sum1_h =
      Ball_and_Plate_MicroLabBox_student_B.Switch_jo -
      Ball_and_Plate_MicroLabBox_student_B.mm2m;

    /* Gain: '<S55>/Gain2' */
    Ball_and_Plate_MicroLabBox_student_B.Gain2 =
      Ball_and_Plate_MicroLabBox_student_P.Gain2_Gain *
      Ball_and_Plate_MicroLabBox_student_B.Sum1_h;

    /* S-Function (dleadlag): '<S55>/Dctleadlag1' */

    /* Level2 S-Function Block: '<S55>/Dctleadlag1' (dleadlag) */
    {
      SimStruct *rts = Ball_and_Plate_MicroLabBox_student_M->childSfunctions[6];
      sfcnOutputs(rts,1);
    }

    /* S-Function (dlowpass1): '<S55>/Dct1lowpass1' */

    /* Level2 S-Function Block: '<S55>/Dct1lowpass1' (dlowpass1) */
    {
      SimStruct *rts = Ball_and_Plate_MicroLabBox_student_M->childSfunctions[7];
      sfcnOutputs(rts,1);
    }

    /* DiscreteIntegrator: '<S55>/Discrete-Time Integrator' incorporates:
     *  Constant: '<S3>/enable_inner_controller'
     */
    if ((Ball_and_Plate_MicroLabBox_student_P.enable_inner_controller_Value >
         0.0) &&
        (Ball_and_Plate_MicroLabBox_student_DW.DiscreteTimeIntegrator_PrevRe_e <=
         0)) {
      Ball_and_Plate_MicroLabBox_student_DW.DiscreteTimeIntegrator_DSTATE_o =
        Ball_and_Plate_MicroLabBox_student_P.DiscreteTimeIntegrator_IC_j;
    }

    if (Ball_and_Plate_MicroLabBox_student_DW.DiscreteTimeIntegrator_DSTATE_o >=
        Ball_and_Plate_MicroLabBox_student_P.inner_I_sat) {
      Ball_and_Plate_MicroLabBox_student_DW.DiscreteTimeIntegrator_DSTATE_o =
        Ball_and_Plate_MicroLabBox_student_P.inner_I_sat;
    } else {
      a21 = -Ball_and_Plate_MicroLabBox_student_P.inner_I_sat;
      if (Ball_and_Plate_MicroLabBox_student_DW.DiscreteTimeIntegrator_DSTATE_o <=
          a21) {
        Ball_and_Plate_MicroLabBox_student_DW.DiscreteTimeIntegrator_DSTATE_o =
          -Ball_and_Plate_MicroLabBox_student_P.inner_I_sat;
      }
    }

    Ball_and_Plate_MicroLabBox_student_B.DiscreteTimeIntegrator_h =
      Ball_and_Plate_MicroLabBox_student_DW.DiscreteTimeIntegrator_DSTATE_o;

    /* End of DiscreteIntegrator: '<S55>/Discrete-Time Integrator' */

    /* Switch: '<S21>/Switch2' incorporates:
     *  Constant: '<S21>/Constant'
     *  Constant: '<S3>/enable_inner_controller'
     */
    if (Ball_and_Plate_MicroLabBox_student_P.enable_inner_controller_Value >=
        Ball_and_Plate_MicroLabBox_student_P.Switch2_Threshold_b) {
      /* Sum: '<S55>/Add' */
      Ball_and_Plate_MicroLabBox_student_B.Add =
        Ball_and_Plate_MicroLabBox_student_B.Dct1lowpass1 +
        Ball_and_Plate_MicroLabBox_student_B.DiscreteTimeIntegrator_h;
      Ball_and_Plate_MicroLabBox_student_B.Switch2_h =
        Ball_and_Plate_MicroLabBox_student_B.Add;
    } else {
      Ball_and_Plate_MicroLabBox_student_B.Switch2_h =
        Ball_and_Plate_MicroLabBox_student_P.Constant_Value_e;
    }

    /* End of Switch: '<S21>/Switch2' */

    /* UniformRandomNumber: '<S21>/Uniform Random Number' */
    Ball_and_Plate_MicroLabBox_student_B.UniformRandomNumber_l =
      Ball_and_Plate_MicroLabBox_student_DW.UniformRandomNumber_NextOutpu_m;

    /* Gain: '<S21>/enable_white_noise_C ' */
    Ball_and_Plate_MicroLabBox_student_B.dC =
      Ball_and_Plate_MicroLabBox_student_P.enable_white_noise_C_Gain *
      Ball_and_Plate_MicroLabBox_student_B.UniformRandomNumber_l;

    /* Sum: '<S21>/Sum' */
    Ball_and_Plate_MicroLabBox_student_B.uC =
      Ball_and_Plate_MicroLabBox_student_B.Switch2_h +
      Ball_and_Plate_MicroLabBox_student_B.dC;
  }

  /* Switch: '<S21>/Switch3' incorporates:
   *  Constant: '<S3>/enable_inner_closeLoop'
   */
  if (Ball_and_Plate_MicroLabBox_student_P.enable_inner_closeLoop_Value >=
      Ball_and_Plate_MicroLabBox_student_P.Switch3_Threshold_g) {
    Ball_and_Plate_MicroLabBox_student_B.Switch3_l =
      Ball_and_Plate_MicroLabBox_student_B.uC;
  } else {
    Ball_and_Plate_MicroLabBox_student_B.Switch3_l =
      Ball_and_Plate_MicroLabBox_student_B.TransferFcn;
  }

  /* End of Switch: '<S21>/Switch3' */

  /* Outputs for Atomic SubSystem: '<S21>/Outputs to Amplifier' */

  /* Saturate: '<S53>/Saturation' */
  center_idx_2 = Ball_and_Plate_MicroLabBox_student_B.Switch3_l;
  uX_idx_1 = Ball_and_Plate_MicroLabBox_student_P.Saturation_LowerSat_p;
  uX_idx_0 = Ball_and_Plate_MicroLabBox_student_P.Saturation_UpperSat_h;
  if (center_idx_2 > uX_idx_0) {
    Ball_and_Plate_MicroLabBox_student_B.Saturation_i = uX_idx_0;
  } else if (center_idx_2 < uX_idx_1) {
    Ball_and_Plate_MicroLabBox_student_B.Saturation_i = uX_idx_1;
  } else {
    Ball_and_Plate_MicroLabBox_student_B.Saturation_i = center_idx_2;
  }

  /* End of Saturate: '<S53>/Saturation' */

  /* Gain: '<S53>/Current2V' */
  Ball_and_Plate_MicroLabBox_student_B.Current2V =
    Ball_and_Plate_MicroLabBox_student_P.Current2V_Gain_p *
    Ball_and_Plate_MicroLabBox_student_B.Saturation_i;

  /* Gain: '<S53>/DSPscale' */
  Ball_and_Plate_MicroLabBox_student_B.DSPscale =
    Ball_and_Plate_MicroLabBox_student_P.DSPscale_Gain_h *
    Ball_and_Plate_MicroLabBox_student_B.Current2V;
  if (rtmIsMajorTimeStep(Ball_and_Plate_MicroLabBox_student_M) &&
      Ball_and_Plate_MicroLabBox_student_M->Timing.TaskCounters.TID[1] == 0) {
    /* S-Function (rti_commonblock): '<S58>/S-Function1' */
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

  /* End of Outputs for SubSystem: '<S21>/Outputs to Amplifier' */
  if (rtmIsMajorTimeStep(Ball_and_Plate_MicroLabBox_student_M) &&
      Ball_and_Plate_MicroLabBox_student_M->Timing.TaskCounters.TID[1] == 0) {
    /* MATLAB Function: '<S51>/MATLAB Function' incorporates:
     *  Constant: '<S21>/1_to_enable_id'
     *  Constant: '<S51>/Constant'
     */
    Ball_and_Plate_M_MATLABFunction
      (Ball_and_Plate_MicroLabBox_student_P.u_to_enable_id_Value,
       Ball_and_Plate_MicroLabBox_student_P.uA,
       &Ball_and_Plate_MicroLabBox_student_B.sf_MATLABFunction_g,
       &Ball_and_Plate_MicroLabBox_student_DW.sf_MATLABFunction_g);

    /* MATLAB Function: '<S52>/MATLAB Function' incorporates:
     *  Constant: '<S21>/Enable_ID_C '
     *  Constant: '<S52>/Constant'
     */
    Ball_and_Plate_M_MATLABFunction
      (Ball_and_Plate_MicroLabBox_student_P.Enable_ID_C_Value,
       Ball_and_Plate_MicroLabBox_student_P.uC,
       &Ball_and_Plate_MicroLabBox_student_B.sf_MATLABFunction_h,
       &Ball_and_Plate_MicroLabBox_student_DW.sf_MATLABFunction_h);

    /* RandomNumber: '<S21>/Random Number' */
    Ball_and_Plate_MicroLabBox_student_B.RandomNumber_n =
      Ball_and_Plate_MicroLabBox_student_DW.NextOutput_p;

    /* Constant: '<S3>/reser_integrator' */
    Ball_and_Plate_MicroLabBox_student_B.reser_integrator =
      Ball_and_Plate_MicroLabBox_student_P.reser_integrator_Value;

    /* DataTypeConversion: '<S64>/Data Type Conversion14' incorporates:
     *  Constant: '<S5>/p_zero'
     */
    Ball_and_Plate_MicroLabBox_student_B.DataTypeConversion14 =
      Ball_and_Plate_MicroLabBox_student_P.p_zero_Value;

    /* DataTypeConversion: '<S64>/Data Type Conversion15' incorporates:
     *  Constant: '<S5>/m_zero'
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
    /* Memory: '<S64>/last_x' */
    Ball_and_Plate_MicroLabBox_student_B.last_x[0] =
      Ball_and_Plate_MicroLabBox_student_DW.last_x_PreviousInput[0];
    Ball_and_Plate_MicroLabBox_student_B.last_x[1] =
      Ball_and_Plate_MicroLabBox_student_DW.last_x_PreviousInput[1];
    Ball_and_Plate_MicroLabBox_student_B.last_x[2] =
      Ball_and_Plate_MicroLabBox_student_DW.last_x_PreviousInput[2];
    Ball_and_Plate_MicroLabBox_student_B.last_x[3] =
      Ball_and_Plate_MicroLabBox_student_DW.last_x_PreviousInput[3];

    /* Gain: '<S64>/umin_scale3' */
    for (r1 = 0; r1 < 52; r1++) {
      Ball_and_Plate_MicroLabBox_student_B.umin_scale3[r1] =
        Ball_and_Plate_MicroLabBox_student_P.umin_scale3_Gain[r1] *
        Ball_and_Plate_MicroLabBox_student_B.useq[r1];
    }

    /* End of Gain: '<S64>/umin_scale3' */

    /* Gain: '<S64>/umin_scale5' */
    for (r1 = 0; r1 < 104; r1++) {
      Ball_and_Plate_MicroLabBox_student_B.umin_scale5[r1] =
        Ball_and_Plate_MicroLabBox_student_P.umin_scale5_Gain[r1] *
        Ball_and_Plate_MicroLabBox_student_B.yseq[r1];
    }

    /* End of Gain: '<S64>/umin_scale5' */

    /* Gain: '<S64>/umin_scale2' incorporates:
     *  Constant: '<S64>/constant'
     */
    Ball_and_Plate_MicroLabBox_student_B.umin_scale2[0] =
      Ball_and_Plate_MicroLabBox_student_P.umin_scale2_Gain[0] *
      Ball_and_Plate_MicroLabBox_student_P.constant_Value[0];
    Ball_and_Plate_MicroLabBox_student_B.umin_scale2[1] =
      Ball_and_Plate_MicroLabBox_student_P.umin_scale2_Gain[1] *
      Ball_and_Plate_MicroLabBox_student_P.constant_Value[1];
  }

  if (rtmIsMajorTimeStep(Ball_and_Plate_MicroLabBox_student_M) &&
      Ball_and_Plate_MicroLabBox_student_M->Timing.TaskCounters.TID[1] == 0) {
    /* Gain: '<S112>/Integral Gain' */
    Ball_and_Plate_MicroLabBox_student_B.IntegralGain =
      Ball_and_Plate_MicroLabBox_student_P.PIDController_I *
      Ball_and_Plate_MicroLabBox_student_B.Add3[0];

    /* Gain: '<S156>/Integral Gain' */
    Ball_and_Plate_MicroLabBox_student_B.IntegralGain_f =
      Ball_and_Plate_MicroLabBox_student_P.PIDController1_I *
      Ball_and_Plate_MicroLabBox_student_B.Add3[2];

    /* Gain: '<S13>/Gain1' */
    Ball_and_Plate_MicroLabBox_student_B.Gain1_k =
      Ball_and_Plate_MicroLabBox_student_P.Gain1_Gain_i * 0.0;

    /* S-Function (dlowpass1): '<S13>/Dct1lowpass2' */

    /* Level2 S-Function Block: '<S13>/Dct1lowpass2' (dlowpass1) */
    {
      SimStruct *rts = Ball_and_Plate_MicroLabBox_student_M->childSfunctions[8];
      sfcnOutputs(rts,1);
    }
  }
}

/* Model update function */
void Ball_and_Plate_MicroLabBox_student_update(void)
{
  boolean_T resetCoeff;
  real_T err;
  real_T tmin;
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
      if ((Ball_and_Plate_MicroLabBox_student_B.CastToDouble >= -1.0) &&
          (Ball_and_Plate_MicroLabBox_student_B.CastToDouble <= 1.0)) {
        tmin = Ball_and_Plate_MicroLabBox_student_P.FirstOrderHold_ErrTol;
      } else if (Ball_and_Plate_MicroLabBox_student_B.CastToDouble > 1.0) {
        tmin = Ball_and_Plate_MicroLabBox_student_B.CastToDouble *
          Ball_and_Plate_MicroLabBox_student_P.FirstOrderHold_ErrTol;
      } else {
        tmin = -(Ball_and_Plate_MicroLabBox_student_B.CastToDouble *
                 Ball_and_Plate_MicroLabBox_student_P.FirstOrderHold_ErrTol);
      }

      err = Ball_and_Plate_MicroLabBox_student_B.FirstOrderHold -
        Ball_and_Plate_MicroLabBox_student_B.CastToDouble;
      if ((err > tmin) || (err < -tmin)) {
        guard1 = true;
      } else {
        tmin = Ball_and_Plate_MicroLabBox_student_M->Timing.t[0] -
          Ball_and_Plate_MicroLabBox_student_DW.Tk;
        Ball_and_Plate_MicroLabBox_student_DW.Mk =
          (Ball_and_Plate_MicroLabBox_student_B.CastToDouble -
           Ball_and_Plate_MicroLabBox_student_DW.Uk) / tmin;
        Ball_and_Plate_MicroLabBox_student_DW.Ck =
          Ball_and_Plate_MicroLabBox_student_B.FirstOrderHold;
      }
    } else {
      guard1 = true;
    }

    if (guard1) {
      if (Ball_and_Plate_MicroLabBox_student_B.CastToDouble !=
          Ball_and_Plate_MicroLabBox_student_B.FirstOrderHold) {
        rtsiSetBlockStateForSolverChangedAtMajorStep
          (&Ball_and_Plate_MicroLabBox_student_M->solverInfo, true);
        rtsiSetContTimeOutputInconsistentWithStateAtMajorStep
          (&Ball_and_Plate_MicroLabBox_student_M->solverInfo, true);
      }

      Ball_and_Plate_MicroLabBox_student_DW.Ck =
        Ball_and_Plate_MicroLabBox_student_B.CastToDouble;
      Ball_and_Plate_MicroLabBox_student_DW.Mk = 0.0;
    }

    Ball_and_Plate_MicroLabBox_student_DW.Uk =
      Ball_and_Plate_MicroLabBox_student_B.CastToDouble;
    Ball_and_Plate_MicroLabBox_student_DW.Tk =
      Ball_and_Plate_MicroLabBox_student_M->Timing.t[0];

    /* End of Update for FirstOrderHold: '<Root>/First Order Hold' */

    /* Update for FirstOrderHold: '<Root>/First Order Hold1' */
    resetCoeff = (Ball_and_Plate_MicroLabBox_student_DW.Tk_c == (rtInf));
    guard1 = false;
    if (!resetCoeff) {
      if ((Ball_and_Plate_MicroLabBox_student_B.CastToDouble1 >= -1.0) &&
          (Ball_and_Plate_MicroLabBox_student_B.CastToDouble1 <= 1.0)) {
        tmin = Ball_and_Plate_MicroLabBox_student_P.FirstOrderHold1_ErrTol;
      } else if (Ball_and_Plate_MicroLabBox_student_B.CastToDouble1 > 1.0) {
        tmin = Ball_and_Plate_MicroLabBox_student_B.CastToDouble1 *
          Ball_and_Plate_MicroLabBox_student_P.FirstOrderHold1_ErrTol;
      } else {
        tmin = -(Ball_and_Plate_MicroLabBox_student_B.CastToDouble1 *
                 Ball_and_Plate_MicroLabBox_student_P.FirstOrderHold1_ErrTol);
      }

      err = Ball_and_Plate_MicroLabBox_student_B.FirstOrderHold1 -
        Ball_and_Plate_MicroLabBox_student_B.CastToDouble1;
      if ((err > tmin) || (err < -tmin)) {
        guard1 = true;
      } else {
        tmin = Ball_and_Plate_MicroLabBox_student_M->Timing.t[0] -
          Ball_and_Plate_MicroLabBox_student_DW.Tk_c;
        Ball_and_Plate_MicroLabBox_student_DW.Mk_g =
          (Ball_and_Plate_MicroLabBox_student_B.CastToDouble1 -
           Ball_and_Plate_MicroLabBox_student_DW.Uk_i) / tmin;
        Ball_and_Plate_MicroLabBox_student_DW.Ck_h =
          Ball_and_Plate_MicroLabBox_student_B.FirstOrderHold1;
      }
    } else {
      guard1 = true;
    }

    if (guard1) {
      if (Ball_and_Plate_MicroLabBox_student_B.CastToDouble1 !=
          Ball_and_Plate_MicroLabBox_student_B.FirstOrderHold1) {
        rtsiSetBlockStateForSolverChangedAtMajorStep
          (&Ball_and_Plate_MicroLabBox_student_M->solverInfo, true);
        rtsiSetContTimeOutputInconsistentWithStateAtMajorStep
          (&Ball_and_Plate_MicroLabBox_student_M->solverInfo, true);
      }

      Ball_and_Plate_MicroLabBox_student_DW.Ck_h =
        Ball_and_Plate_MicroLabBox_student_B.CastToDouble1;
      Ball_and_Plate_MicroLabBox_student_DW.Mk_g = 0.0;
    }

    Ball_and_Plate_MicroLabBox_student_DW.Uk_i =
      Ball_and_Plate_MicroLabBox_student_B.CastToDouble1;
    Ball_and_Plate_MicroLabBox_student_DW.Tk_c =
      Ball_and_Plate_MicroLabBox_student_M->Timing.t[0];

    /* End of Update for FirstOrderHold: '<Root>/First Order Hold1' */
  }

  if (rtmIsMajorTimeStep(Ball_and_Plate_MicroLabBox_student_M) &&
      Ball_and_Plate_MicroLabBox_student_M->Timing.TaskCounters.TID[1] == 0) {
    /* Update for UnitDelay: '<S177>/Delay Input1' */
    Ball_and_Plate_MicroLabBox_student_DW.DelayInput1_DSTATE =
      Ball_and_Plate_MicroLabBox_student_B.Compare;

    /* Update for DiscreteIntegrator: '<S159>/Integrator' incorporates:
     *  Constant: '<Root>/enable_outer_controller'
     *  DiscreteIntegrator: '<S154>/Filter'
     */
    Ball_and_Plate_MicroLabBox_student_DW.Integrator_DSTATE +=
      Ball_and_Plate_MicroLabBox_student_P.Integrator_gainval *
      Ball_and_Plate_MicroLabBox_student_B.IntegralGain_f;
    if (Ball_and_Plate_MicroLabBox_student_P.enable_outer_controller_Value > 0.0)
    {
      Ball_and_Plate_MicroLabBox_student_DW.Integrator_PrevResetState = 1;
      Ball_and_Plate_MicroLabBox_student_DW.Filter_PrevResetState = 1;
    } else {
      if (Ball_and_Plate_MicroLabBox_student_P.enable_outer_controller_Value <
          0.0) {
        Ball_and_Plate_MicroLabBox_student_DW.Integrator_PrevResetState = -1;
      } else if
          (Ball_and_Plate_MicroLabBox_student_P.enable_outer_controller_Value ==
           0.0) {
        Ball_and_Plate_MicroLabBox_student_DW.Integrator_PrevResetState = 0;
      } else {
        Ball_and_Plate_MicroLabBox_student_DW.Integrator_PrevResetState = 2;
      }

      if (Ball_and_Plate_MicroLabBox_student_P.enable_outer_controller_Value <
          0.0) {
        Ball_and_Plate_MicroLabBox_student_DW.Filter_PrevResetState = -1;
      } else if
          (Ball_and_Plate_MicroLabBox_student_P.enable_outer_controller_Value ==
           0.0) {
        Ball_and_Plate_MicroLabBox_student_DW.Filter_PrevResetState = 0;
      } else {
        Ball_and_Plate_MicroLabBox_student_DW.Filter_PrevResetState = 2;
      }
    }

    /* End of Update for DiscreteIntegrator: '<S159>/Integrator' */

    /* Update for DiscreteIntegrator: '<S154>/Filter' */
    Ball_and_Plate_MicroLabBox_student_DW.Filter_DSTATE +=
      Ball_and_Plate_MicroLabBox_student_P.Filter_gainval *
      Ball_and_Plate_MicroLabBox_student_B.FilterCoefficient;

    /* Update for DiscreteIntegrator: '<S115>/Integrator' incorporates:
     *  Constant: '<Root>/enable_outer_controller'
     *  DiscreteIntegrator: '<S110>/Filter'
     */
    Ball_and_Plate_MicroLabBox_student_DW.Integrator_DSTATE_n +=
      Ball_and_Plate_MicroLabBox_student_P.Integrator_gainval_b *
      Ball_and_Plate_MicroLabBox_student_B.IntegralGain;
    if (Ball_and_Plate_MicroLabBox_student_P.enable_outer_controller_Value > 0.0)
    {
      Ball_and_Plate_MicroLabBox_student_DW.Integrator_PrevResetState_n = 1;
      Ball_and_Plate_MicroLabBox_student_DW.Filter_PrevResetState_h = 1;
    } else {
      if (Ball_and_Plate_MicroLabBox_student_P.enable_outer_controller_Value <
          0.0) {
        Ball_and_Plate_MicroLabBox_student_DW.Integrator_PrevResetState_n = -1;
      } else if
          (Ball_and_Plate_MicroLabBox_student_P.enable_outer_controller_Value ==
           0.0) {
        Ball_and_Plate_MicroLabBox_student_DW.Integrator_PrevResetState_n = 0;
      } else {
        Ball_and_Plate_MicroLabBox_student_DW.Integrator_PrevResetState_n = 2;
      }

      if (Ball_and_Plate_MicroLabBox_student_P.enable_outer_controller_Value <
          0.0) {
        Ball_and_Plate_MicroLabBox_student_DW.Filter_PrevResetState_h = -1;
      } else if
          (Ball_and_Plate_MicroLabBox_student_P.enable_outer_controller_Value ==
           0.0) {
        Ball_and_Plate_MicroLabBox_student_DW.Filter_PrevResetState_h = 0;
      } else {
        Ball_and_Plate_MicroLabBox_student_DW.Filter_PrevResetState_h = 2;
      }
    }

    /* End of Update for DiscreteIntegrator: '<S115>/Integrator' */

    /* Update for DiscreteIntegrator: '<S110>/Filter' */
    Ball_and_Plate_MicroLabBox_student_DW.Filter_DSTATE_p +=
      Ball_and_Plate_MicroLabBox_student_P.Filter_gainval_g *
      Ball_and_Plate_MicroLabBox_student_B.FilterCoefficient_l;

    /* Update for Delay: '<Root>/Delay' */
    Ball_and_Plate_MicroLabBox_student_DW.Delay_DSTATE[0] =
      Ball_and_Plate_MicroLabBox_student_B.MultiportSwitch[0];
    Ball_and_Plate_MicroLabBox_student_DW.Delay_DSTATE[1] =
      Ball_and_Plate_MicroLabBox_student_B.MultiportSwitch[1];

    /* Update for DiscreteIntegrator: '<S27>/Discrete-Time Integrator' incorporates:
     *  Constant: '<S3>/enable_inner_controller'
     */
    Ball_and_Plate_MicroLabBox_student_DW.DiscreteTimeIntegrator_DSTATE +=
      Ball_and_Plate_MicroLabBox_student_P.DiscreteTimeIntegrator_gainval *
      Ball_and_Plate_MicroLabBox_student_B.Sum1;
    if (Ball_and_Plate_MicroLabBox_student_DW.DiscreteTimeIntegrator_DSTATE >=
        Ball_and_Plate_MicroLabBox_student_P.inner_I_sat) {
      Ball_and_Plate_MicroLabBox_student_DW.DiscreteTimeIntegrator_DSTATE =
        Ball_and_Plate_MicroLabBox_student_P.inner_I_sat;
    } else {
      tmin = -Ball_and_Plate_MicroLabBox_student_P.inner_I_sat;
      if (Ball_and_Plate_MicroLabBox_student_DW.DiscreteTimeIntegrator_DSTATE <=
          tmin) {
        Ball_and_Plate_MicroLabBox_student_DW.DiscreteTimeIntegrator_DSTATE =
          -Ball_and_Plate_MicroLabBox_student_P.inner_I_sat;
      }
    }

    if (Ball_and_Plate_MicroLabBox_student_P.enable_inner_controller_Value > 0.0)
    {
      Ball_and_Plate_MicroLabBox_student_DW.DiscreteTimeIntegrator_PrevRese = 1;
    } else if
        (Ball_and_Plate_MicroLabBox_student_P.enable_inner_controller_Value <
         0.0) {
      Ball_and_Plate_MicroLabBox_student_DW.DiscreteTimeIntegrator_PrevRese = -1;
    } else if
        (Ball_and_Plate_MicroLabBox_student_P.enable_inner_controller_Value ==
         0.0) {
      Ball_and_Plate_MicroLabBox_student_DW.DiscreteTimeIntegrator_PrevRese = 0;
    } else {
      Ball_and_Plate_MicroLabBox_student_DW.DiscreteTimeIntegrator_PrevRese = 2;
    }

    /* End of Update for DiscreteIntegrator: '<S27>/Discrete-Time Integrator' */

    /* Update for UniformRandomNumber: '<S19>/Uniform Random Number' */
    tmin = Ball_and_Plate_MicroLabBox_student_P.UniformRandomNumber_Minimum;
    Ball_and_Plate_MicroLabBox_student_DW.UniformRandomNumber_NextOutput =
      (Ball_and_Plate_MicroLabBox_student_P.UniformRandomNumber_Maximum - tmin) *
      rt_urand_Upu32_Yd_f_pw_snf(&Ball_and_Plate_MicroLabBox_student_DW.RandSeed)
      + tmin;

    /* Update for DiscreteIntegrator: '<S40>/Discrete-Time Integrator' incorporates:
     *  Constant: '<S3>/enable_inner_controller'
     */
    Ball_and_Plate_MicroLabBox_student_DW.DiscreteTimeIntegrator_DSTATE_p +=
      Ball_and_Plate_MicroLabBox_student_P.DiscreteTimeIntegrator_gainva_m *
      Ball_and_Plate_MicroLabBox_student_B.Sum1_f;
    if (Ball_and_Plate_MicroLabBox_student_DW.DiscreteTimeIntegrator_DSTATE_p >=
        Ball_and_Plate_MicroLabBox_student_P.inner_I_sat) {
      Ball_and_Plate_MicroLabBox_student_DW.DiscreteTimeIntegrator_DSTATE_p =
        Ball_and_Plate_MicroLabBox_student_P.inner_I_sat;
    } else {
      tmin = -Ball_and_Plate_MicroLabBox_student_P.inner_I_sat;
      if (Ball_and_Plate_MicroLabBox_student_DW.DiscreteTimeIntegrator_DSTATE_p <=
          tmin) {
        Ball_and_Plate_MicroLabBox_student_DW.DiscreteTimeIntegrator_DSTATE_p =
          -Ball_and_Plate_MicroLabBox_student_P.inner_I_sat;
      }
    }

    if (Ball_and_Plate_MicroLabBox_student_P.enable_inner_controller_Value > 0.0)
    {
      Ball_and_Plate_MicroLabBox_student_DW.DiscreteTimeIntegrator_PrevRe_h = 1;
    } else if
        (Ball_and_Plate_MicroLabBox_student_P.enable_inner_controller_Value <
         0.0) {
      Ball_and_Plate_MicroLabBox_student_DW.DiscreteTimeIntegrator_PrevRe_h = -1;
    } else if
        (Ball_and_Plate_MicroLabBox_student_P.enable_inner_controller_Value ==
         0.0) {
      Ball_and_Plate_MicroLabBox_student_DW.DiscreteTimeIntegrator_PrevRe_h = 0;
    } else {
      Ball_and_Plate_MicroLabBox_student_DW.DiscreteTimeIntegrator_PrevRe_h = 2;
    }

    /* End of Update for DiscreteIntegrator: '<S40>/Discrete-Time Integrator' */

    /* Update for UniformRandomNumber: '<S20>/Uniform Random Number' */
    tmin = Ball_and_Plate_MicroLabBox_student_P.UniformRandomNumber_Minimum_m;
    Ball_and_Plate_MicroLabBox_student_DW.UniformRandomNumber_NextOutpu_n =
      (Ball_and_Plate_MicroLabBox_student_P.UniformRandomNumber_Maximum_l - tmin)
      * rt_urand_Upu32_Yd_f_pw_snf
      (&Ball_and_Plate_MicroLabBox_student_DW.RandSeed_e) + tmin;

    /* Update for RandomNumber: '<S20>/Random Number' */
    Ball_and_Plate_MicroLabBox_student_DW.NextOutput =
      rt_nrand_Upu32_Yd_f_pw_snf
      (&Ball_and_Plate_MicroLabBox_student_DW.RandSeed_g) *
      Ball_and_Plate_MicroLabBox_student_P.RandomNumber_StdDev +
      Ball_and_Plate_MicroLabBox_student_P.RandomNumber_Mean;

    /* Update for DiscreteIntegrator: '<S55>/Discrete-Time Integrator' incorporates:
     *  Constant: '<S3>/enable_inner_controller'
     */
    Ball_and_Plate_MicroLabBox_student_DW.DiscreteTimeIntegrator_DSTATE_o +=
      Ball_and_Plate_MicroLabBox_student_P.DiscreteTimeIntegrator_gainva_g *
      Ball_and_Plate_MicroLabBox_student_B.Sum1_h;
    if (Ball_and_Plate_MicroLabBox_student_DW.DiscreteTimeIntegrator_DSTATE_o >=
        Ball_and_Plate_MicroLabBox_student_P.inner_I_sat) {
      Ball_and_Plate_MicroLabBox_student_DW.DiscreteTimeIntegrator_DSTATE_o =
        Ball_and_Plate_MicroLabBox_student_P.inner_I_sat;
    } else {
      tmin = -Ball_and_Plate_MicroLabBox_student_P.inner_I_sat;
      if (Ball_and_Plate_MicroLabBox_student_DW.DiscreteTimeIntegrator_DSTATE_o <=
          tmin) {
        Ball_and_Plate_MicroLabBox_student_DW.DiscreteTimeIntegrator_DSTATE_o =
          -Ball_and_Plate_MicroLabBox_student_P.inner_I_sat;
      }
    }

    if (Ball_and_Plate_MicroLabBox_student_P.enable_inner_controller_Value > 0.0)
    {
      Ball_and_Plate_MicroLabBox_student_DW.DiscreteTimeIntegrator_PrevRe_e = 1;
    } else if
        (Ball_and_Plate_MicroLabBox_student_P.enable_inner_controller_Value <
         0.0) {
      Ball_and_Plate_MicroLabBox_student_DW.DiscreteTimeIntegrator_PrevRe_e = -1;
    } else if
        (Ball_and_Plate_MicroLabBox_student_P.enable_inner_controller_Value ==
         0.0) {
      Ball_and_Plate_MicroLabBox_student_DW.DiscreteTimeIntegrator_PrevRe_e = 0;
    } else {
      Ball_and_Plate_MicroLabBox_student_DW.DiscreteTimeIntegrator_PrevRe_e = 2;
    }

    /* End of Update for DiscreteIntegrator: '<S55>/Discrete-Time Integrator' */

    /* Update for UniformRandomNumber: '<S21>/Uniform Random Number' */
    tmin = Ball_and_Plate_MicroLabBox_student_P.UniformRandomNumber_Minimum_c;
    Ball_and_Plate_MicroLabBox_student_DW.UniformRandomNumber_NextOutpu_m =
      (Ball_and_Plate_MicroLabBox_student_P.UniformRandomNumber_Maximum_o - tmin)
      * rt_urand_Upu32_Yd_f_pw_snf
      (&Ball_and_Plate_MicroLabBox_student_DW.RandSeed_c) + tmin;

    /* Update for RandomNumber: '<S21>/Random Number' */
    Ball_and_Plate_MicroLabBox_student_DW.NextOutput_p =
      rt_nrand_Upu32_Yd_f_pw_snf
      (&Ball_and_Plate_MicroLabBox_student_DW.RandSeed_e2) *
      Ball_and_Plate_MicroLabBox_student_P.RandomNumber_StdDev_e +
      Ball_and_Plate_MicroLabBox_student_P.RandomNumber_Mean_o;
  }

  if (rtmIsMajorTimeStep(Ball_and_Plate_MicroLabBox_student_M) &&
      Ball_and_Plate_MicroLabBox_student_M->Timing.TaskCounters.TID[2] == 0) {
    /* Update for UnitDelay: '<S64>/last_mv' */
    Ball_and_Plate_MicroLabBox_student_DW.last_mv_DSTATE[0] =
      Ball_and_Plate_MicroLabBox_student_B.u[0];
    Ball_and_Plate_MicroLabBox_student_DW.last_mv_DSTATE[1] =
      Ball_and_Plate_MicroLabBox_student_B.u[1];

    /* Update for Memory: '<S64>/Memory' */
    memcpy(&Ball_and_Plate_MicroLabBox_student_DW.Memory_PreviousInput[0],
           &Ball_and_Plate_MicroLabBox_student_B.iAout[0], 212U * sizeof
           (boolean_T));

    /* Update for Memory: '<S64>/last_x' */
    Ball_and_Plate_MicroLabBox_student_DW.last_x_PreviousInput[0] =
      Ball_and_Plate_MicroLabBox_student_B.xk1[0];
    Ball_and_Plate_MicroLabBox_student_DW.last_x_PreviousInput[1] =
      Ball_and_Plate_MicroLabBox_student_B.xk1[1];
    Ball_and_Plate_MicroLabBox_student_DW.last_x_PreviousInput[2] =
      Ball_and_Plate_MicroLabBox_student_B.xk1[2];
    Ball_and_Plate_MicroLabBox_student_DW.last_x_PreviousInput[3] =
      Ball_and_Plate_MicroLabBox_student_B.xk1[3];
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

  /* Derivatives for Atomic SubSystem: '<S19>/Position Measurement' */
  /* Derivatives for TransferFcn: '<S32>/Transfer Fcn' */
  _rtXdot->TransferFcn_CSTATE_l = 0.0;
  _rtXdot->TransferFcn_CSTATE_l +=
    Ball_and_Plate_MicroLabBox_student_P.TransferFcn_A *
    Ball_and_Plate_MicroLabBox_student_X.TransferFcn_CSTATE_l;
  _rtXdot->TransferFcn_CSTATE_l +=
    Ball_and_Plate_MicroLabBox_student_B.EnabledSubsystem.OutportBufferForOut1;

  /* End of Derivatives for SubSystem: '<S19>/Position Measurement' */

  /* Derivatives for Atomic SubSystem: '<S20>/Position Measurement' */
  /* Derivatives for TransferFcn: '<S47>/Transfer Fcn' */
  _rtXdot->TransferFcn_CSTATE_h = 0.0;
  _rtXdot->TransferFcn_CSTATE_h +=
    Ball_and_Plate_MicroLabBox_student_P.TransferFcn_A_g *
    Ball_and_Plate_MicroLabBox_student_X.TransferFcn_CSTATE_h;
  _rtXdot->TransferFcn_CSTATE_h +=
    Ball_and_Plate_MicroLabBox_student_B.EnabledSubsystem_a.OutportBufferForOut1;

  /* End of Derivatives for SubSystem: '<S20>/Position Measurement' */

  /* Derivatives for Atomic SubSystem: '<S21>/Position Measurement' */
  /* Derivatives for TransferFcn: '<S61>/Transfer Fcn' */
  _rtXdot->TransferFcn_CSTATE = 0.0;
  _rtXdot->TransferFcn_CSTATE +=
    Ball_and_Plate_MicroLabBox_student_P.TransferFcn_A_m *
    Ball_and_Plate_MicroLabBox_student_X.TransferFcn_CSTATE;
  _rtXdot->TransferFcn_CSTATE +=
    Ball_and_Plate_MicroLabBox_student_B.EnabledSubsystem_n.OutportBufferForOut1;

  /* End of Derivatives for SubSystem: '<S21>/Position Measurement' */
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

  Ball_and_Plate_MicroLabBox_student_M->Sizes.numSFcns = (9);

  /* register each child */
  {
    (void) memset((void *)
                  &Ball_and_Plate_MicroLabBox_student_M->NonInlinedSFcns.childSFunctions
                  [0], 0,
                  9*sizeof(SimStruct));
    Ball_and_Plate_MicroLabBox_student_M->childSfunctions =
      (&Ball_and_Plate_MicroLabBox_student_M->
       NonInlinedSFcns.childSFunctionPtrs[0]);

    {
      int_T i;
      for (i = 0; i < 9; i++) {
        Ball_and_Plate_MicroLabBox_student_M->childSfunctions[i] =
          (&Ball_and_Plate_MicroLabBox_student_M->NonInlinedSFcns.childSFunctions
           [i]);
      }
    }

    /* Level2 S-Function Block: Ball_and_Plate_MicroLabBox_student/<S11>/Dct1lowpass2 (dlowpass1) */
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
      ssSetPath(rts, "Ball_and_Plate_MicroLabBox_student/lowpass 2/Dct1lowpass2");
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

    /* Level2 S-Function Block: Ball_and_Plate_MicroLabBox_student/<S12>/Dct1lowpass2 (dlowpass1) */
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
          sfcnUPtrs[0] = &Ball_and_Plate_MicroLabBox_student_B.Gain1_d;
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
          &Ball_and_Plate_MicroLabBox_student_M->NonInlinedSFcns.Sfcn1.params;
        ssSetSFcnParamsCount(rts, 2);
        ssSetSFcnParamsPtr(rts, &sfcnParams[0]);
        ssSetSFcnParam(rts, 0, (mxArray*)
                       Ball_and_Plate_MicroLabBox_student_P.Dct1lowpass2_P1_Size_p);
        ssSetSFcnParam(rts, 1, (mxArray*)
                       Ball_and_Plate_MicroLabBox_student_P.Dct1lowpass2_P2_Size_c);
      }

      /* work vectors */
      ssSetRWork(rts, (real_T *)
                 &Ball_and_Plate_MicroLabBox_student_DW.Dct1lowpass2_RWORK_g[0]);

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

    /* Level2 S-Function Block: Ball_and_Plate_MicroLabBox_student/<S27>/Dctleadlag2 (dleadlag) */
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
          &Ball_and_Plate_MicroLabBox_student_M->NonInlinedSFcns.Sfcn2.params;
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

    /* Level2 S-Function Block: Ball_and_Plate_MicroLabBox_student/<S27>/Dct1lowpass3 (dlowpass1) */
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
          sfcnUPtrs[0] = &Ball_and_Plate_MicroLabBox_student_B.Dctleadlag2;
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
          &Ball_and_Plate_MicroLabBox_student_M->NonInlinedSFcns.Sfcn3.params;
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

    /* Level2 S-Function Block: Ball_and_Plate_MicroLabBox_student/<S40>/Dctleadlag2 (dleadlag) */
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
          sfcnUPtrs[0] = &Ball_and_Plate_MicroLabBox_student_B.Gain1_f;
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
          &Ball_and_Plate_MicroLabBox_student_M->NonInlinedSFcns.Sfcn4.params;
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

    /* Level2 S-Function Block: Ball_and_Plate_MicroLabBox_student/<S40>/Dct1lowpass3 (dlowpass1) */
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
          sfcnUPtrs[0] = &Ball_and_Plate_MicroLabBox_student_B.Dctleadlag2_k;
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
          &Ball_and_Plate_MicroLabBox_student_M->NonInlinedSFcns.Sfcn5.params;
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

    /* Level2 S-Function Block: Ball_and_Plate_MicroLabBox_student/<S55>/Dctleadlag1 (dleadlag) */
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
          sfcnUPtrs[0] = &Ball_and_Plate_MicroLabBox_student_B.Gain2;
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
            &Ball_and_Plate_MicroLabBox_student_B.Dctleadlag1));
        }
      }

      /* path info */
      ssSetModelName(rts, "Dctleadlag1");
      ssSetPath(rts,
                "Ball_and_Plate_MicroLabBox_student/Innerloop_Actuator/Motor_C/motorC/Dctleadlag1");
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
                       Ball_and_Plate_MicroLabBox_student_P.Dctleadlag1_P1_Size);
        ssSetSFcnParam(rts, 1, (mxArray*)
                       Ball_and_Plate_MicroLabBox_student_P.Dctleadlag1_P2_Size);
        ssSetSFcnParam(rts, 2, (mxArray*)
                       Ball_and_Plate_MicroLabBox_student_P.Dctleadlag1_P3_Size);
      }

      /* work vectors */
      ssSetRWork(rts, (real_T *)
                 &Ball_and_Plate_MicroLabBox_student_DW.Dctleadlag1_RWORK[0]);

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
                   &Ball_and_Plate_MicroLabBox_student_DW.Dctleadlag1_RWORK[0]);
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

    /* Level2 S-Function Block: Ball_and_Plate_MicroLabBox_student/<S55>/Dct1lowpass1 (dlowpass1) */
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
          sfcnUPtrs[0] = &Ball_and_Plate_MicroLabBox_student_B.Dctleadlag1;
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
            &Ball_and_Plate_MicroLabBox_student_B.Dct1lowpass1));
        }
      }

      /* path info */
      ssSetModelName(rts, "Dct1lowpass1");
      ssSetPath(rts,
                "Ball_and_Plate_MicroLabBox_student/Innerloop_Actuator/Motor_C/motorC/Dct1lowpass1");
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
                       Ball_and_Plate_MicroLabBox_student_P.Dct1lowpass1_P1_Size);
        ssSetSFcnParam(rts, 1, (mxArray*)
                       Ball_and_Plate_MicroLabBox_student_P.Dct1lowpass1_P2_Size);
      }

      /* work vectors */
      ssSetRWork(rts, (real_T *)
                 &Ball_and_Plate_MicroLabBox_student_DW.Dct1lowpass1_RWORK[0]);

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
                   &Ball_and_Plate_MicroLabBox_student_DW.Dct1lowpass1_RWORK[0]);
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
          sfcnUPtrs[0] = &Ball_and_Plate_MicroLabBox_student_B.Gain1_k;
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
            &Ball_and_Plate_MicroLabBox_student_B.Dct1lowpass2_k));
        }
      }

      /* path info */
      ssSetModelName(rts, "Dct1lowpass2");
      ssSetPath(rts, "Ball_and_Plate_MicroLabBox_student/lowpass 6/Dct1lowpass2");
      ssSetRTModel(rts,Ball_and_Plate_MicroLabBox_student_M);
      ssSetParentSS(rts, (NULL));
      ssSetRootSS(rts, rts);
      ssSetVersion(rts, SIMSTRUCT_VERSION_LEVEL2);

      /* parameters */
      {
        mxArray **sfcnParams = (mxArray **)
          &Ball_and_Plate_MicroLabBox_student_M->NonInlinedSFcns.Sfcn8.params;
        ssSetSFcnParamsCount(rts, 2);
        ssSetSFcnParamsPtr(rts, &sfcnParams[0]);
        ssSetSFcnParam(rts, 0, (mxArray*)
                       Ball_and_Plate_MicroLabBox_student_P.Dct1lowpass2_P1_Size_i);
        ssSetSFcnParam(rts, 1, (mxArray*)
                       Ball_and_Plate_MicroLabBox_student_P.Dct1lowpass2_P2_Size_a);
      }

      /* work vectors */
      ssSetRWork(rts, (real_T *)
                 &Ball_and_Plate_MicroLabBox_student_DW.Dct1lowpass2_RWORK_j[0]);

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
                   &Ball_and_Plate_MicroLabBox_student_DW.Dct1lowpass2_RWORK_j[0]);
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

  /* Start for Enabled SubSystem: '<S32>/Enabled Subsystem' */
  Ball_and_EnabledSubsystem_Start
    (&Ball_and_Plate_MicroLabBox_student_DW.EnabledSubsystem);

  /* End of Start for SubSystem: '<S32>/Enabled Subsystem' */

  /* Start for Enabled SubSystem: '<S32>/Enabled Subsystem1' */
  Ball_an_EnabledSubsystem1_Start
    (&Ball_and_Plate_MicroLabBox_student_DW.EnabledSubsystem1);

  /* End of Start for SubSystem: '<S32>/Enabled Subsystem1' */
  /* End of Start for SubSystem: '<S19>/Position Measurement' */

  /* Start for Enabled SubSystem: '<S47>/Enabled Subsystem' */
  Ball_and_EnabledSubsystem_Start
    (&Ball_and_Plate_MicroLabBox_student_DW.EnabledSubsystem_a);

  /* End of Start for SubSystem: '<S47>/Enabled Subsystem' */

  /* Start for Enabled SubSystem: '<S47>/Enabled Subsystem1' */
  Ball_an_EnabledSubsystem1_Start
    (&Ball_and_Plate_MicroLabBox_student_DW.EnabledSubsystem1_f);

  /* End of Start for SubSystem: '<S47>/Enabled Subsystem1' */
  /* End of Start for SubSystem: '<S20>/Position Measurement' */

  /* Start for Enabled SubSystem: '<S61>/Enabled Subsystem' */
  Ball_and_EnabledSubsystem_Start
    (&Ball_and_Plate_MicroLabBox_student_DW.EnabledSubsystem_n);

  /* End of Start for SubSystem: '<S61>/Enabled Subsystem' */

  /* Start for Enabled SubSystem: '<S61>/Enabled Subsystem1' */
  Ball_an_EnabledSubsystem1_Start
    (&Ball_and_Plate_MicroLabBox_student_DW.EnabledSubsystem1_b);

  /* End of Start for SubSystem: '<S61>/Enabled Subsystem1' */
  /* End of Start for SubSystem: '<S21>/Position Measurement' */

  /* Start for S-Function (dlowpass1): '<S11>/Dct1lowpass2' */
  /* Level2 S-Function Block: '<S11>/Dct1lowpass2' (dlowpass1) */
  {
    SimStruct *rts = Ball_and_Plate_MicroLabBox_student_M->childSfunctions[0];
    sfcnStart(rts);
    if (ssGetErrorStatus(rts) != (NULL))
      return;
  }

  /* Start for S-Function (dlowpass1): '<S12>/Dct1lowpass2' */
  /* Level2 S-Function Block: '<S12>/Dct1lowpass2' (dlowpass1) */
  {
    SimStruct *rts = Ball_and_Plate_MicroLabBox_student_M->childSfunctions[1];
    sfcnStart(rts);
    if (ssGetErrorStatus(rts) != (NULL))
      return;
  }

  /* Start for S-Function (dleadlag): '<S27>/Dctleadlag2' */
  /* Level2 S-Function Block: '<S27>/Dctleadlag2' (dleadlag) */
  {
    SimStruct *rts = Ball_and_Plate_MicroLabBox_student_M->childSfunctions[2];
    sfcnStart(rts);
    if (ssGetErrorStatus(rts) != (NULL))
      return;
  }

  /* Start for S-Function (dlowpass1): '<S27>/Dct1lowpass3' */
  /* Level2 S-Function Block: '<S27>/Dct1lowpass3' (dlowpass1) */
  {
    SimStruct *rts = Ball_and_Plate_MicroLabBox_student_M->childSfunctions[3];
    sfcnStart(rts);
    if (ssGetErrorStatus(rts) != (NULL))
      return;
  }

  /* Start for S-Function (dleadlag): '<S40>/Dctleadlag2' */
  /* Level2 S-Function Block: '<S40>/Dctleadlag2' (dleadlag) */
  {
    SimStruct *rts = Ball_and_Plate_MicroLabBox_student_M->childSfunctions[4];
    sfcnStart(rts);
    if (ssGetErrorStatus(rts) != (NULL))
      return;
  }

  /* Start for S-Function (dlowpass1): '<S40>/Dct1lowpass3' */
  /* Level2 S-Function Block: '<S40>/Dct1lowpass3' (dlowpass1) */
  {
    SimStruct *rts = Ball_and_Plate_MicroLabBox_student_M->childSfunctions[5];
    sfcnStart(rts);
    if (ssGetErrorStatus(rts) != (NULL))
      return;
  }

  /* Start for S-Function (dleadlag): '<S55>/Dctleadlag1' */
  /* Level2 S-Function Block: '<S55>/Dctleadlag1' (dleadlag) */
  {
    SimStruct *rts = Ball_and_Plate_MicroLabBox_student_M->childSfunctions[6];
    sfcnStart(rts);
    if (ssGetErrorStatus(rts) != (NULL))
      return;
  }

  /* Start for S-Function (dlowpass1): '<S55>/Dct1lowpass1' */
  /* Level2 S-Function Block: '<S55>/Dct1lowpass1' (dlowpass1) */
  {
    SimStruct *rts = Ball_and_Plate_MicroLabBox_student_M->childSfunctions[7];
    sfcnStart(rts);
    if (ssGetErrorStatus(rts) != (NULL))
      return;
  }

  /* Start for S-Function (dlowpass1): '<S13>/Dct1lowpass2' */
  /* Level2 S-Function Block: '<S13>/Dct1lowpass2' (dlowpass1) */
  {
    SimStruct *rts = Ball_and_Plate_MicroLabBox_student_M->childSfunctions[8];
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
    uint32_T tseed;
    int32_T r;
    int32_T t;
    real_T tmin;
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

    /* InitializeConditions for UnitDelay: '<S177>/Delay Input1' */
    Ball_and_Plate_MicroLabBox_student_DW.DelayInput1_DSTATE =
      Ball_and_Plate_MicroLabBox_student_P.DetectRisePositive_vinit;

    /* InitializeConditions for RateLimiter: '<S9>/Rate Limiter' */
    Ball_and_Plate_MicroLabBox_student_DW.PrevY[0] =
      Ball_and_Plate_MicroLabBox_student_P.RateLimiter_IC;

    /* InitializeConditions for UnitDelay: '<S64>/last_mv' */
    Ball_and_Plate_MicroLabBox_student_DW.last_mv_DSTATE[0] =
      Ball_and_Plate_MicroLabBox_student_P.last_mv_InitialCondition[0];

    /* InitializeConditions for RateLimiter: '<S9>/Rate Limiter' */
    Ball_and_Plate_MicroLabBox_student_DW.PrevY[1] =
      Ball_and_Plate_MicroLabBox_student_P.RateLimiter_IC;

    /* InitializeConditions for UnitDelay: '<S64>/last_mv' */
    Ball_and_Plate_MicroLabBox_student_DW.last_mv_DSTATE[1] =
      Ball_and_Plate_MicroLabBox_student_P.last_mv_InitialCondition[1];

    /* InitializeConditions for Memory: '<S64>/Memory' */
    memcpy(&Ball_and_Plate_MicroLabBox_student_DW.Memory_PreviousInput[0],
           &Ball_and_Plate_MicroLabBox_student_P.Memory_InitialCondition[0],
           212U * sizeof(boolean_T));

    /* InitializeConditions for DiscreteIntegrator: '<S159>/Integrator' */
    Ball_and_Plate_MicroLabBox_student_DW.Integrator_DSTATE =
      Ball_and_Plate_MicroLabBox_student_P.PIDController1_InitialConditi_b;
    Ball_and_Plate_MicroLabBox_student_DW.Integrator_PrevResetState = 2;

    /* InitializeConditions for DiscreteIntegrator: '<S154>/Filter' */
    Ball_and_Plate_MicroLabBox_student_DW.Filter_DSTATE =
      Ball_and_Plate_MicroLabBox_student_P.PIDController1_InitialCondition;
    Ball_and_Plate_MicroLabBox_student_DW.Filter_PrevResetState = 2;

    /* InitializeConditions for DiscreteIntegrator: '<S115>/Integrator' */
    Ball_and_Plate_MicroLabBox_student_DW.Integrator_DSTATE_n =
      Ball_and_Plate_MicroLabBox_student_P.PIDController_InitialConditio_l;
    Ball_and_Plate_MicroLabBox_student_DW.Integrator_PrevResetState_n = 2;

    /* InitializeConditions for DiscreteIntegrator: '<S110>/Filter' */
    Ball_and_Plate_MicroLabBox_student_DW.Filter_DSTATE_p =
      Ball_and_Plate_MicroLabBox_student_P.PIDController_InitialConditionF;
    Ball_and_Plate_MicroLabBox_student_DW.Filter_PrevResetState_h = 2;

    /* InitializeConditions for Delay: '<Root>/Delay' */
    Ball_and_Plate_MicroLabBox_student_DW.Delay_DSTATE[0] =
      Ball_and_Plate_MicroLabBox_student_P.Delay_InitialCondition;
    Ball_and_Plate_MicroLabBox_student_DW.Delay_DSTATE[1] =
      Ball_and_Plate_MicroLabBox_student_P.Delay_InitialCondition;

    /* InitializeConditions for DiscreteIntegrator: '<S27>/Discrete-Time Integrator' */
    Ball_and_Plate_MicroLabBox_student_DW.DiscreteTimeIntegrator_DSTATE =
      Ball_and_Plate_MicroLabBox_student_P.DiscreteTimeIntegrator_IC;
    Ball_and_Plate_MicroLabBox_student_DW.DiscreteTimeIntegrator_PrevRese = 2;

    /* InitializeConditions for UniformRandomNumber: '<S19>/Uniform Random Number' */
    tmin = floor(Ball_and_Plate_MicroLabBox_student_P.UniformRandomNumber_Seed);
    if (rtIsNaN(tmin) || rtIsInf(tmin)) {
      tmin = 0.0;
    } else {
      tmin = fmod(tmin, 4.294967296E+9);
    }

    tseed = tmin < 0.0 ? (uint32_T)-(int32_T)(uint32_T)-tmin : (uint32_T)tmin;
    r = (int32_T)(tseed >> 16U);
    t = (int32_T)(tseed & 32768U);
    tseed = ((((tseed - ((uint32_T)r << 16U)) + t) << 16U) + t) + r;
    if (tseed < 1U) {
      tseed = 1144108930U;
    } else {
      if (tseed > 2147483646U) {
        tseed = 2147483646U;
      }
    }

    Ball_and_Plate_MicroLabBox_student_DW.RandSeed = tseed;
    tmin = Ball_and_Plate_MicroLabBox_student_P.UniformRandomNumber_Minimum;
    Ball_and_Plate_MicroLabBox_student_DW.UniformRandomNumber_NextOutput =
      (Ball_and_Plate_MicroLabBox_student_P.UniformRandomNumber_Maximum - tmin) *
      rt_urand_Upu32_Yd_f_pw_snf(&Ball_and_Plate_MicroLabBox_student_DW.RandSeed)
      + tmin;

    /* End of InitializeConditions for UniformRandomNumber: '<S19>/Uniform Random Number' */

    /* InitializeConditions for DiscreteIntegrator: '<S40>/Discrete-Time Integrator' */
    Ball_and_Plate_MicroLabBox_student_DW.DiscreteTimeIntegrator_DSTATE_p =
      Ball_and_Plate_MicroLabBox_student_P.DiscreteTimeIntegrator_IC_m;
    Ball_and_Plate_MicroLabBox_student_DW.DiscreteTimeIntegrator_PrevRe_h = 2;

    /* InitializeConditions for UniformRandomNumber: '<S20>/Uniform Random Number' */
    tmin = floor(Ball_and_Plate_MicroLabBox_student_P.UniformRandomNumber_Seed_e);
    if (rtIsNaN(tmin) || rtIsInf(tmin)) {
      tmin = 0.0;
    } else {
      tmin = fmod(tmin, 4.294967296E+9);
    }

    tseed = tmin < 0.0 ? (uint32_T)-(int32_T)(uint32_T)-tmin : (uint32_T)tmin;
    r = (int32_T)(tseed >> 16U);
    t = (int32_T)(tseed & 32768U);
    tseed = ((((tseed - ((uint32_T)r << 16U)) + t) << 16U) + t) + r;
    if (tseed < 1U) {
      tseed = 1144108930U;
    } else {
      if (tseed > 2147483646U) {
        tseed = 2147483646U;
      }
    }

    Ball_and_Plate_MicroLabBox_student_DW.RandSeed_e = tseed;
    tmin = Ball_and_Plate_MicroLabBox_student_P.UniformRandomNumber_Minimum_m;
    Ball_and_Plate_MicroLabBox_student_DW.UniformRandomNumber_NextOutpu_n =
      (Ball_and_Plate_MicroLabBox_student_P.UniformRandomNumber_Maximum_l - tmin)
      * rt_urand_Upu32_Yd_f_pw_snf
      (&Ball_and_Plate_MicroLabBox_student_DW.RandSeed_e) + tmin;

    /* End of InitializeConditions for UniformRandomNumber: '<S20>/Uniform Random Number' */

    /* InitializeConditions for RandomNumber: '<S20>/Random Number' */
    tmin = floor(Ball_and_Plate_MicroLabBox_student_P.RandomNumber_Seed);
    if (rtIsNaN(tmin) || rtIsInf(tmin)) {
      tmin = 0.0;
    } else {
      tmin = fmod(tmin, 4.294967296E+9);
    }

    tseed = tmin < 0.0 ? (uint32_T)-(int32_T)(uint32_T)-tmin : (uint32_T)tmin;
    r = (int32_T)(tseed >> 16U);
    t = (int32_T)(tseed & 32768U);
    tseed = ((((tseed - ((uint32_T)r << 16U)) + t) << 16U) + t) + r;
    if (tseed < 1U) {
      tseed = 1144108930U;
    } else {
      if (tseed > 2147483646U) {
        tseed = 2147483646U;
      }
    }

    Ball_and_Plate_MicroLabBox_student_DW.RandSeed_g = tseed;
    Ball_and_Plate_MicroLabBox_student_DW.NextOutput =
      rt_nrand_Upu32_Yd_f_pw_snf
      (&Ball_and_Plate_MicroLabBox_student_DW.RandSeed_g) *
      Ball_and_Plate_MicroLabBox_student_P.RandomNumber_StdDev +
      Ball_and_Plate_MicroLabBox_student_P.RandomNumber_Mean;

    /* End of InitializeConditions for RandomNumber: '<S20>/Random Number' */

    /* InitializeConditions for DiscreteIntegrator: '<S55>/Discrete-Time Integrator' */
    Ball_and_Plate_MicroLabBox_student_DW.DiscreteTimeIntegrator_DSTATE_o =
      Ball_and_Plate_MicroLabBox_student_P.DiscreteTimeIntegrator_IC_j;
    Ball_and_Plate_MicroLabBox_student_DW.DiscreteTimeIntegrator_PrevRe_e = 2;

    /* InitializeConditions for UniformRandomNumber: '<S21>/Uniform Random Number' */
    tmin = floor(Ball_and_Plate_MicroLabBox_student_P.UniformRandomNumber_Seed_b);
    if (rtIsNaN(tmin) || rtIsInf(tmin)) {
      tmin = 0.0;
    } else {
      tmin = fmod(tmin, 4.294967296E+9);
    }

    tseed = tmin < 0.0 ? (uint32_T)-(int32_T)(uint32_T)-tmin : (uint32_T)tmin;
    r = (int32_T)(tseed >> 16U);
    t = (int32_T)(tseed & 32768U);
    tseed = ((((tseed - ((uint32_T)r << 16U)) + t) << 16U) + t) + r;
    if (tseed < 1U) {
      tseed = 1144108930U;
    } else {
      if (tseed > 2147483646U) {
        tseed = 2147483646U;
      }
    }

    Ball_and_Plate_MicroLabBox_student_DW.RandSeed_c = tseed;
    tmin = Ball_and_Plate_MicroLabBox_student_P.UniformRandomNumber_Minimum_c;
    Ball_and_Plate_MicroLabBox_student_DW.UniformRandomNumber_NextOutpu_m =
      (Ball_and_Plate_MicroLabBox_student_P.UniformRandomNumber_Maximum_o - tmin)
      * rt_urand_Upu32_Yd_f_pw_snf
      (&Ball_and_Plate_MicroLabBox_student_DW.RandSeed_c) + tmin;

    /* End of InitializeConditions for UniformRandomNumber: '<S21>/Uniform Random Number' */

    /* InitializeConditions for RandomNumber: '<S21>/Random Number' */
    tmin = floor(Ball_and_Plate_MicroLabBox_student_P.RandomNumber_Seed_g);
    if (rtIsNaN(tmin) || rtIsInf(tmin)) {
      tmin = 0.0;
    } else {
      tmin = fmod(tmin, 4.294967296E+9);
    }

    tseed = tmin < 0.0 ? (uint32_T)-(int32_T)(uint32_T)-tmin : (uint32_T)tmin;
    r = (int32_T)(tseed >> 16U);
    t = (int32_T)(tseed & 32768U);
    tseed = ((((tseed - ((uint32_T)r << 16U)) + t) << 16U) + t) + r;
    if (tseed < 1U) {
      tseed = 1144108930U;
    } else {
      if (tseed > 2147483646U) {
        tseed = 2147483646U;
      }
    }

    Ball_and_Plate_MicroLabBox_student_DW.RandSeed_e2 = tseed;
    Ball_and_Plate_MicroLabBox_student_DW.NextOutput_p =
      rt_nrand_Upu32_Yd_f_pw_snf
      (&Ball_and_Plate_MicroLabBox_student_DW.RandSeed_e2) *
      Ball_and_Plate_MicroLabBox_student_P.RandomNumber_StdDev_e +
      Ball_and_Plate_MicroLabBox_student_P.RandomNumber_Mean_o;

    /* End of InitializeConditions for RandomNumber: '<S21>/Random Number' */

    /* InitializeConditions for Memory: '<S64>/last_x' */
    Ball_and_Plate_MicroLabBox_student_DW.last_x_PreviousInput[0] =
      Ball_and_Plate_MicroLabBox_student_P.last_x_InitialCondition[0];
    Ball_and_Plate_MicroLabBox_student_DW.last_x_PreviousInput[1] =
      Ball_and_Plate_MicroLabBox_student_P.last_x_InitialCondition[1];
    Ball_and_Plate_MicroLabBox_student_DW.last_x_PreviousInput[2] =
      Ball_and_Plate_MicroLabBox_student_P.last_x_InitialCondition[2];
    Ball_and_Plate_MicroLabBox_student_DW.last_x_PreviousInput[3] =
      Ball_and_Plate_MicroLabBox_student_P.last_x_InitialCondition[3];

    /* SystemInitialize for Atomic SubSystem: '<S19>/Position Measurement' */
    /* InitializeConditions for TransferFcn: '<S32>/Transfer Fcn' */
    Ball_and_Plate_MicroLabBox_student_X.TransferFcn_CSTATE_l = 0.0;

    /* SystemInitialize for Enabled SubSystem: '<S32>/Enabled Subsystem' */
    Ball_and__EnabledSubsystem_Init
      (&Ball_and_Plate_MicroLabBox_student_B.EnabledSubsystem,
       &Ball_and_Plate_MicroLabBox_student_P.EnabledSubsystem);

    /* End of SystemInitialize for SubSystem: '<S32>/Enabled Subsystem' */

    /* SystemInitialize for Enabled SubSystem: '<S32>/Enabled Subsystem1' */
    Ball_and_EnabledSubsystem1_Init
      (&Ball_and_Plate_MicroLabBox_student_B.EnabledSubsystem1,
       &Ball_and_Plate_MicroLabBox_student_P.EnabledSubsystem1);

    /* End of SystemInitialize for SubSystem: '<S32>/Enabled Subsystem1' */
    /* End of SystemInitialize for SubSystem: '<S19>/Position Measurement' */

    /* SystemInitialize for Atomic SubSystem: '<S20>/Position Measurement' */
    /* InitializeConditions for TransferFcn: '<S47>/Transfer Fcn' */
    Ball_and_Plate_MicroLabBox_student_X.TransferFcn_CSTATE_h = 0.0;

    /* SystemInitialize for Enabled SubSystem: '<S47>/Enabled Subsystem' */
    Ball_and__EnabledSubsystem_Init
      (&Ball_and_Plate_MicroLabBox_student_B.EnabledSubsystem_a,
       &Ball_and_Plate_MicroLabBox_student_P.EnabledSubsystem_a);

    /* End of SystemInitialize for SubSystem: '<S47>/Enabled Subsystem' */

    /* SystemInitialize for Enabled SubSystem: '<S47>/Enabled Subsystem1' */
    Ball_and_EnabledSubsystem1_Init
      (&Ball_and_Plate_MicroLabBox_student_B.EnabledSubsystem1_f,
       &Ball_and_Plate_MicroLabBox_student_P.EnabledSubsystem1_f);

    /* End of SystemInitialize for SubSystem: '<S47>/Enabled Subsystem1' */
    /* End of SystemInitialize for SubSystem: '<S20>/Position Measurement' */

    /* SystemInitialize for Atomic SubSystem: '<S21>/Position Measurement' */
    /* InitializeConditions for TransferFcn: '<S61>/Transfer Fcn' */
    Ball_and_Plate_MicroLabBox_student_X.TransferFcn_CSTATE = 0.0;

    /* SystemInitialize for Enabled SubSystem: '<S61>/Enabled Subsystem' */
    Ball_and__EnabledSubsystem_Init
      (&Ball_and_Plate_MicroLabBox_student_B.EnabledSubsystem_n,
       &Ball_and_Plate_MicroLabBox_student_P.EnabledSubsystem_n);

    /* End of SystemInitialize for SubSystem: '<S61>/Enabled Subsystem' */

    /* SystemInitialize for Enabled SubSystem: '<S61>/Enabled Subsystem1' */
    Ball_and_EnabledSubsystem1_Init
      (&Ball_and_Plate_MicroLabBox_student_B.EnabledSubsystem1_b,
       &Ball_and_Plate_MicroLabBox_student_P.EnabledSubsystem1_b);

    /* End of SystemInitialize for SubSystem: '<S61>/Enabled Subsystem1' */
    /* End of SystemInitialize for SubSystem: '<S21>/Position Measurement' */

    /* SystemInitialize for MATLAB Function: '<Root>/MATLAB Function' */
    Ball_and_Plate_MicroLabBox_student_DW.x_hat[0] = 0.0;
    Ball_and_Plate_MicroLabBox_student_DW.x_hat[1] = 0.0;
    Ball_and_Plate_MicroLabBox_student_DW.x_hat[2] = 0.0;
    Ball_and_Plate_MicroLabBox_student_DW.x_hat[3] = 0.0;

    /* SystemInitialize for MATLAB Function: '<Root>/circ_ref_player ' */
    Ball_and_Plate_MicroLabBox_student_DW.idx_ref_a = 1.0;

    /* SystemInitialize for MATLAB Function: '<Root>/square_ref_player ' */
    Ball_and_Plate_MicroLabBox_student_DW.idx_ref = 1.0;
    for (r = 0; r < 16; r++) {
      /* SystemInitialize for MATLAB Function: '<Root>/MATLAB Function' */
      Ball_and_Plate_MicroLabBox_student_DW.P[r] = tmp[r];

      /* SystemInitialize for MATLAB Function: '<S9>/MATLAB Function1' */
      Ball_and_Plate_MicroLabBox_student_DW.xK[r] = 0.0;
    }

    /* SystemInitialize for MATLAB Function: '<S9>/MATLAB Function1' */
    Ball_and_Plate_MicroLabBox_student_DW.u_prev[0] = 0.0;
    Ball_and_Plate_MicroLabBox_student_DW.u_prev[1] = 0.0;

    /* SystemInitialize for MATLAB Function: '<S9>/MATLAB Function' */
    Ball_and_Plate_MicroLabBox_student_DW.initialized_not_empty = false;

    /* SystemInitialize for MATLAB Function: '<S19>/MATLAB Function1' */
    Ball_and_P_MATLABFunction1_Init
      (&Ball_and_Plate_MicroLabBox_student_DW.sf_MATLABFunction1);

    /* SystemInitialize for MATLAB Function: '<S24>/MATLAB Function' */
    Ball_and_Pl_MATLABFunction_Init
      (&Ball_and_Plate_MicroLabBox_student_DW.sf_MATLABFunction);

    /* SystemInitialize for MATLAB Function: '<S20>/MATLAB Function2' */
    Ball_and_P_MATLABFunction1_Init
      (&Ball_and_Plate_MicroLabBox_student_DW.sf_MATLABFunction2_d);

    /* SystemInitialize for MATLAB Function: '<S35>/MATLAB Function' */
    Ball_and_Plate_MicroLabBox_student_DW.index = 1.0;
    Ball_and_Plate_MicroLabBox_student_DW.previous_enable = 0.0;

    /* SystemInitialize for MATLAB Function: '<S37>/MATLAB Function' */
    Ball_and_Pl_MATLABFunction_Init
      (&Ball_and_Plate_MicroLabBox_student_DW.sf_MATLABFunction_ko);

    /* SystemInitialize for MATLAB Function: '<S21>/MATLAB Function1' */
    Ball_and_P_MATLABFunction1_Init
      (&Ball_and_Plate_MicroLabBox_student_DW.sf_MATLABFunction1_h);

    /* SystemInitialize for MATLAB Function: '<S51>/MATLAB Function' */
    Ball_and_Pl_MATLABFunction_Init
      (&Ball_and_Plate_MicroLabBox_student_DW.sf_MATLABFunction_g);

    /* SystemInitialize for MATLAB Function: '<S52>/MATLAB Function' */
    Ball_and_Pl_MATLABFunction_Init
      (&Ball_and_Plate_MicroLabBox_student_DW.sf_MATLABFunction_h);
  }
}

/* Model terminate function */
void Ball_and_Plate_MicroLabBox_student_terminate(void)
{
  /* Terminate for Atomic SubSystem: '<S19>/Position Measurement' */
  /* Terminate for Triggered SubSystem: '<S26>/EMC_ENCODER_POS_SET_BL1' */
  /* Terminate for S-Function (rti_commonblock): '<S31>/S-Function1' incorporates:
   *  Constant: '<S26>/Constant'
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

  /* End of Terminate for SubSystem: '<S26>/EMC_ENCODER_POS_SET_BL1' */

  /* Terminate for S-Function (rti_commonblock): '<S30>/S-Function1' */

  /* --- Ball_and_Plate_MicroLabBox_student/Innerloop_Actuator/Motor_A/Position Measurement/EMC_ENCODER_BL1 --- */
  /* --- [RTIEMC, Encoder] - DIO class: 2 - Unit: 1 - Port: 1 - Channel: 1 --- */
  {
    /* Deactivates encoder interface functionality */
    DioCl2EncoderIn_stop(pRTIEmcEncoder_Unit_1_DioCl_2_Port_1_Ch1);
  }

  /* End of Terminate for SubSystem: '<S19>/Position Measurement' */

  /* Terminate for Atomic SubSystem: '<S20>/Position Measurement' */
  /* Terminate for Triggered SubSystem: '<S39>/EMC_ENCODER_POS_SET_BL1' */
  /* Terminate for S-Function (rti_commonblock): '<S46>/S-Function1' incorporates:
   *  Constant: '<S39>/Constant'
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

  /* End of Terminate for SubSystem: '<S39>/EMC_ENCODER_POS_SET_BL1' */

  /* Terminate for S-Function (rti_commonblock): '<S45>/S-Function1' */

  /* --- Ball_and_Plate_MicroLabBox_student/Innerloop_Actuator/Motor_B/Position Measurement/EMC_ENCODER_BL1 --- */
  /* --- [RTIEMC, Encoder] - DIO class: 2 - Unit: 2 - Port: 1 - Channel: 3 --- */
  {
    /* Deactivates encoder interface functionality */
    DioCl2EncoderIn_stop(pRTIEmcEncoder_Unit_2_DioCl_2_Port_1_Ch3);
  }

  /* End of Terminate for SubSystem: '<S20>/Position Measurement' */

  /* Terminate for Atomic SubSystem: '<S21>/Position Measurement' */
  /* Terminate for Triggered SubSystem: '<S54>/EMC_ENCODER_POS_SET_BL1' */
  /* Terminate for S-Function (rti_commonblock): '<S60>/S-Function1' incorporates:
   *  Constant: '<S54>/Constant'
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

  /* End of Terminate for SubSystem: '<S54>/EMC_ENCODER_POS_SET_BL1' */

  /* Terminate for S-Function (rti_commonblock): '<S59>/S-Function1' */

  /* --- Ball_and_Plate_MicroLabBox_student/Innerloop_Actuator/Motor_C/Position Measurement/EMC_ENCODER_BL1 --- */
  /* --- [RTIEMC, Encoder] - DIO class: 2 - Unit: 3 - Port: 1 - Channel: 5 --- */
  {
    /* Deactivates encoder interface functionality */
    DioCl2EncoderIn_stop(pRTIEmcEncoder_Unit_3_DioCl_2_Port_1_Ch5);
  }

  /* End of Terminate for SubSystem: '<S21>/Position Measurement' */

  /* Terminate for S-Function (dlowpass1): '<S11>/Dct1lowpass2' */
  /* Level2 S-Function Block: '<S11>/Dct1lowpass2' (dlowpass1) */
  {
    SimStruct *rts = Ball_and_Plate_MicroLabBox_student_M->childSfunctions[0];
    sfcnTerminate(rts);
  }

  /* Terminate for S-Function (dlowpass1): '<S12>/Dct1lowpass2' */
  /* Level2 S-Function Block: '<S12>/Dct1lowpass2' (dlowpass1) */
  {
    SimStruct *rts = Ball_and_Plate_MicroLabBox_student_M->childSfunctions[1];
    sfcnTerminate(rts);
  }

  /* Terminate for S-Function (rti_commonblock): '<S17>/S-Function1' */
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

  /* Terminate for S-Function (dleadlag): '<S27>/Dctleadlag2' */
  /* Level2 S-Function Block: '<S27>/Dctleadlag2' (dleadlag) */
  {
    SimStruct *rts = Ball_and_Plate_MicroLabBox_student_M->childSfunctions[2];
    sfcnTerminate(rts);
  }

  /* Terminate for S-Function (dlowpass1): '<S27>/Dct1lowpass3' */
  /* Level2 S-Function Block: '<S27>/Dct1lowpass3' (dlowpass1) */
  {
    SimStruct *rts = Ball_and_Plate_MicroLabBox_student_M->childSfunctions[3];
    sfcnTerminate(rts);
  }

  /* Terminate for Atomic SubSystem: '<S19>/Outputs to Amplifier' */
  /* Terminate for S-Function (rti_commonblock): '<S29>/S-Function1' */

  /* --- Ball_and_Plate_MicroLabBox_student/Innerloop_Actuator/Motor_A/Outputs to Amplifier/DAC_CLASS1_BL1 --- */
  /* --- [RTI120X, DAC C1] - Channel: 1 --- */

  /* All channel outputs are set to high impedance state */
  DacCl1AnalogOut_setOutputHighZ(pRTIDacC1AnalogOut_Ch_1, DAC_CLASS1_HIGH_Z_ON);

  /* Deactivates AnalogOut functionality */
  DacCl1AnalogOut_stop(pRTIDacC1AnalogOut_Ch_1);

  /* End of Terminate for SubSystem: '<S19>/Outputs to Amplifier' */

  /* Terminate for S-Function (dleadlag): '<S40>/Dctleadlag2' */
  /* Level2 S-Function Block: '<S40>/Dctleadlag2' (dleadlag) */
  {
    SimStruct *rts = Ball_and_Plate_MicroLabBox_student_M->childSfunctions[4];
    sfcnTerminate(rts);
  }

  /* Terminate for S-Function (dlowpass1): '<S40>/Dct1lowpass3' */
  /* Level2 S-Function Block: '<S40>/Dct1lowpass3' (dlowpass1) */
  {
    SimStruct *rts = Ball_and_Plate_MicroLabBox_student_M->childSfunctions[5];
    sfcnTerminate(rts);
  }

  /* Terminate for Atomic SubSystem: '<S20>/Outputs to Amplifier' */
  /* Terminate for S-Function (rti_commonblock): '<S44>/S-Function1' */

  /* --- Ball_and_Plate_MicroLabBox_student/Innerloop_Actuator/Motor_B/Outputs to Amplifier/DAC_CLASS1_BL1 --- */
  /* --- [RTI120X, DAC C1] - Channel: 2 --- */

  /* All channel outputs are set to high impedance state */
  DacCl1AnalogOut_setOutputHighZ(pRTIDacC1AnalogOut_Ch_2, DAC_CLASS1_HIGH_Z_ON);

  /* Deactivates AnalogOut functionality */
  DacCl1AnalogOut_stop(pRTIDacC1AnalogOut_Ch_2);

  /* End of Terminate for SubSystem: '<S20>/Outputs to Amplifier' */

  /* Terminate for S-Function (dleadlag): '<S55>/Dctleadlag1' */
  /* Level2 S-Function Block: '<S55>/Dctleadlag1' (dleadlag) */
  {
    SimStruct *rts = Ball_and_Plate_MicroLabBox_student_M->childSfunctions[6];
    sfcnTerminate(rts);
  }

  /* Terminate for S-Function (dlowpass1): '<S55>/Dct1lowpass1' */
  /* Level2 S-Function Block: '<S55>/Dct1lowpass1' (dlowpass1) */
  {
    SimStruct *rts = Ball_and_Plate_MicroLabBox_student_M->childSfunctions[7];
    sfcnTerminate(rts);
  }

  /* Terminate for Atomic SubSystem: '<S21>/Outputs to Amplifier' */
  /* Terminate for S-Function (rti_commonblock): '<S58>/S-Function1' */

  /* --- Ball_and_Plate_MicroLabBox_student/Innerloop_Actuator/Motor_C/Outputs to Amplifier/DAC_CLASS1_BL1 --- */
  /* --- [RTI120X, DAC C1] - Channel: 3 --- */

  /* All channel outputs are set to high impedance state */
  DacCl1AnalogOut_setOutputHighZ(pRTIDacC1AnalogOut_Ch_3, DAC_CLASS1_HIGH_Z_ON);

  /* Deactivates AnalogOut functionality */
  DacCl1AnalogOut_stop(pRTIDacC1AnalogOut_Ch_3);

  /* End of Terminate for SubSystem: '<S21>/Outputs to Amplifier' */

  /* Terminate for S-Function (dlowpass1): '<S13>/Dct1lowpass2' */
  /* Level2 S-Function Block: '<S13>/Dct1lowpass2' (dlowpass1) */
  {
    SimStruct *rts = Ball_and_Plate_MicroLabBox_student_M->childSfunctions[8];
    sfcnTerminate(rts);
  }
}
