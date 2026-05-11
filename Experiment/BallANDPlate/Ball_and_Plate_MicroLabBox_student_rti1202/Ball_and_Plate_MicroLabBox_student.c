/*
 * Ball_and_Plate_MicroLabBox_student.c
 *
 * Academic License - for use in teaching, academic research, and meeting
 * course requirements at degree granting institutions only.  Not for
 * government, commercial, or other organizational use.
 *
 * Code generation for model "Ball_and_Plate_MicroLabBox_student".
 *
 * Model version              : 1.6
 * Simulink Coder version : 9.2 (R2019b) 18-Jul-2019
 * C source code generated on : Mon May 11 17:22:35 2026
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
 *    '<S13>/MATLAB Function'
 *    '<S27>/MATLAB Function'
 *    '<S41>/MATLAB Function'
 */
void Ball_and_Pl_MATLABFunction_Init(DW_MATLABFunction_Ball_and_Pl_T *localDW)
{
  localDW->index = 1.0;
  localDW->previous_enable = 0.0;
}

/*
 * Output and update for atomic system:
 *    '<S13>/MATLAB Function'
 *    '<S27>/MATLAB Function'
 *    '<S41>/MATLAB Function'
 */
void Ball_and_Plate_M_MATLABFunction(real_T rtu_enable, const real_T rtu_u[3001],
  B_MATLABFunction_Ball_and_Pla_T *localB, DW_MATLABFunction_Ball_and_Pl_T
  *localDW)
{
  /* MATLAB Function 'Innerloop_Actuator/Motor_A/Init_motion/MATLAB Function': '<S17>:1' */
  /* '<S17>:1:14' */
  if ((rtu_enable != 0.0) && (!(localDW->previous_enable != 0.0))) {
    /* '<S17>:1:17' */
    /* '<S17>:1:18' */
    localDW->index = 1.0;
  }

  if (rtu_enable != 0.0) {
    if (localDW->index <= 3001.0) {
      /* '<S17>:1:25' */
      /* '<S17>:1:26' */
      localB->y = rtu_u[(int32_T)localDW->index - 1];

      /* '<S17>:1:27' */
      localDW->index++;
    } else {
      /* '<S17>:1:31' */
      localB->y = 0.0;
    }
  } else {
    /* '<S17>:1:36' */
    localB->y = -0.0289;
  }

  /* '<S17>:1:39' */
  localDW->previous_enable = rtu_enable;
}

/*
 * Output and update for atomic system:
 *    '<S13>/MATLAB Function2'
 *    '<S27>/MATLAB Function2'
 *    '<S41>/MATLAB Function2'
 */
void Ball_and_Plate__MATLABFunction2(real_T rtu_y, real_T rtu_start, real_T
  rtu_init_value, B_MATLABFunction2_Ball_and_Pl_T *localB)
{
  /* MATLAB Function 'Innerloop_Actuator/Motor_A/Init_motion/MATLAB Function2': '<S18>:1' */
  if (rtu_start > 0.5) {
    /* '<S18>:1:3' */
    /* '<S18>:1:4' */
    localB->path = rtu_init_value;
  } else if (rtu_start < 0.5) {
    /* '<S18>:1:5' */
    /* '<S18>:1:6' */
    localB->path = rtu_y;
  } else {
    /* '<S18>:1:8' */
    localB->path = 0.0;
  }
}

/*
 * System initialize for atomic system:
 *    '<S14>/MATLAB Function'
 *    '<S28>/MATLAB Function'
 *    '<S42>/MATLAB Function'
 *    '<S43>/MATLAB Function'
 */
void Ball_and__MATLABFunction_o_Init(DW_MATLABFunction_Ball_and__j_T *localDW)
{
  localDW->index = 1.0;
  localDW->previous_enable = 0.0;
}

/*
 * Output and update for atomic system:
 *    '<S14>/MATLAB Function'
 *    '<S28>/MATLAB Function'
 *    '<S42>/MATLAB Function'
 *    '<S43>/MATLAB Function'
 */
void Ball_and_Plate_MATLABFunction_e(real_T rtu_enable, const real_T rtu_u[30000],
  B_MATLABFunction_Ball_and_P_m_T *localB, DW_MATLABFunction_Ball_and__j_T
  *localDW)
{
  /* MATLAB Function 'Innerloop_Actuator/Motor_A/Multisine/MATLAB Function': '<S19>:1' */
  /* '<S19>:1:14' */
  if ((rtu_enable != 0.0) && (!(localDW->previous_enable != 0.0))) {
    /* '<S19>:1:17' */
    /* '<S19>:1:18' */
    localDW->index = 1.0;
  }

  if (rtu_enable != 0.0) {
    if (localDW->index <= 30000.0) {
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
    localB->y = 0.0;
  }

  /* '<S19>:1:39' */
  localDW->previous_enable = rtu_enable;
}

/*
 * System initialize for enable system:
 *    '<S23>/Enabled Subsystem'
 *    '<S37>/Enabled Subsystem'
 *    '<S53>/Enabled Subsystem'
 */
void Ball_and__EnabledSubsystem_Init(B_EnabledSubsystem_Ball_and_P_T *localB,
  P_EnabledSubsystem_Ball_and_P_T *localP)
{
  /* SystemInitialize for Outport: '<S24>/Out1' */
  localB->OutportBufferForOut1 = localP->Out1_Y0;
}

/*
 * Disable for enable system:
 *    '<S23>/Enabled Subsystem'
 *    '<S37>/Enabled Subsystem'
 *    '<S53>/Enabled Subsystem'
 */
void Ball_a_EnabledSubsystem_Disable(B_EnabledSubsystem_Ball_and_P_T *localB,
  DW_EnabledSubsystem_Ball_and__T *localDW, P_EnabledSubsystem_Ball_and_P_T
  *localP)
{
  /* Disable for Outport: '<S24>/Out1' */
  localB->OutportBufferForOut1 = localP->Out1_Y0;
  localDW->EnabledSubsystem_MODE = false;
}

/*
 * Start for enable system:
 *    '<S23>/Enabled Subsystem'
 *    '<S37>/Enabled Subsystem'
 *    '<S53>/Enabled Subsystem'
 */
void Ball_and_EnabledSubsystem_Start(DW_EnabledSubsystem_Ball_and__T *localDW)
{
  localDW->EnabledSubsystem_MODE = false;
}

/*
 * Output and update for enable system:
 *    '<S23>/Enabled Subsystem'
 *    '<S37>/Enabled Subsystem'
 *    '<S53>/Enabled Subsystem'
 */
void Ball_and_Plate_EnabledSubsystem
  (RT_MODEL_Ball_and_Plate_MicroLabBox_student_T * const
   Ball_and_Plate_MicroLabBox_student_M, real_T rtu_Enable,
   B_EnabledSubsystem_Ball_and_P_T *localB, DW_EnabledSubsystem_Ball_and__T
   *localDW, P_EnabledSubsystem_Ball_and_P_T *localP)
{
  /* Outputs for Enabled SubSystem: '<S23>/Enabled Subsystem' incorporates:
   *  EnablePort: '<S24>/Enable'
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
    /* SignalConversion generated from: '<S24>/Out1' incorporates:
     *  Constant: '<S24>/Constant'
     */
    localB->OutportBufferForOut1 = localP->Constant_Value;
  }

  /* End of Outputs for SubSystem: '<S23>/Enabled Subsystem' */
}

/*
 * System initialize for enable system:
 *    '<S23>/Enabled Subsystem1'
 *    '<S37>/Enabled Subsystem1'
 *    '<S53>/Enabled Subsystem1'
 */
void Ball_and_EnabledSubsystem1_Init(B_EnabledSubsystem1_Ball_and__T *localB,
  P_EnabledSubsystem1_Ball_and__T *localP)
{
  /* SystemInitialize for Outport: '<S25>/Out1' */
  localB->In1 = localP->Out1_Y0;
}

/*
 * Disable for enable system:
 *    '<S23>/Enabled Subsystem1'
 *    '<S37>/Enabled Subsystem1'
 *    '<S53>/Enabled Subsystem1'
 */
void Ball__EnabledSubsystem1_Disable(DW_EnabledSubsystem1_Ball_and_T *localDW)
{
  localDW->EnabledSubsystem1_MODE = false;
}

/*
 * Start for enable system:
 *    '<S23>/Enabled Subsystem1'
 *    '<S37>/Enabled Subsystem1'
 *    '<S53>/Enabled Subsystem1'
 */
void Ball_an_EnabledSubsystem1_Start(DW_EnabledSubsystem1_Ball_and_T *localDW)
{
  localDW->EnabledSubsystem1_MODE = false;
}

/*
 * Output and update for enable system:
 *    '<S23>/Enabled Subsystem1'
 *    '<S37>/Enabled Subsystem1'
 *    '<S53>/Enabled Subsystem1'
 */
void Ball_and_Plat_EnabledSubsystem1
  (RT_MODEL_Ball_and_Plate_MicroLabBox_student_T * const
   Ball_and_Plate_MicroLabBox_student_M, boolean_T rtu_Enable, real_T rtu_In1,
   B_EnabledSubsystem1_Ball_and__T *localB, DW_EnabledSubsystem1_Ball_and_T
   *localDW)
{
  /* Outputs for Enabled SubSystem: '<S23>/Enabled Subsystem1' incorporates:
   *  EnablePort: '<S25>/Enable'
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
    /* Inport: '<S25>/In1' */
    localB->In1 = rtu_In1;
  }

  /* End of Outputs for SubSystem: '<S23>/Enabled Subsystem1' */
}

/* Model output function */
void Ball_and_Plate_MicroLabBox_student_output(void)
{
  ZCEventType zcEvent;
  real_T u0;
  real_T u1;
  real_T u2;
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
    u0 = 1.0 / Ball_and_Plate_MicroLabBox_student_P.Ts_Outer;
    Ball_and_Plate_MicroLabBox_student_B.Gain = u0 *
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

    /* Sum: '<S3>/Add1' incorporates:
     *  Constant: '<S3>/Constant'
     *  Constant: '<S3>/Constant3'
     */
    Ball_and_Plate_MicroLabBox_student_B.Add1 =
      Ball_and_Plate_MicroLabBox_student_P.Constant3_Value_c -
      Ball_and_Plate_MicroLabBox_student_P.Constant_Value_o;

    /* Sum: '<S3>/Add2' incorporates:
     *  Constant: '<S3>/Constant1'
     */
    Ball_and_Plate_MicroLabBox_student_B.Add2 = 0.0 -
      Ball_and_Plate_MicroLabBox_student_P.Constant1_Value_n;

    /* Sum: '<S3>/Add3' incorporates:
     *  Constant: '<S3>/Constant2'
     */
    Ball_and_Plate_MicroLabBox_student_B.Add3 = 0.0 -
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

  /* Outputs for Atomic SubSystem: '<S9>/Position Measurement' */
  if (rtmIsMajorTimeStep(Ball_and_Plate_MicroLabBox_student_M) &&
      Ball_and_Plate_MicroLabBox_student_M->Timing.TaskCounters.TID[1] == 0) {
    /* Outputs for Triggered SubSystem: '<S16>/EMC_ENCODER_POS_SET_BL1' incorporates:
     *  TriggerPort: '<S22>/Trigger'
     */
    if (rtmIsMajorTimeStep(Ball_and_Plate_MicroLabBox_student_M)) {
      /* Constant: '<S3>/Init  ' */
      zcEvent = rt_ZCFcn(RISING_ZERO_CROSSING,
                         &Ball_and_Plate_MicroLabBox_student_PrevZCX.EMC_ENCODER_POS_SET_BL1_Trig_fh,
                         (Ball_and_Plate_MicroLabBox_student_P.Init_Value));
      if (zcEvent != NO_ZCEVENT) {
        /* S-Function (rti_commonblock): '<S22>/S-Function1' incorporates:
         *  Constant: '<S16>/Constant'
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

    /* End of Outputs for SubSystem: '<S16>/EMC_ENCODER_POS_SET_BL1' */

    /* S-Function (rti_commonblock): '<S21>/S-Function1' */
    /* This comment workarounds a code generation problem */

    /* Gain: '<S16>/Inc2Pos' */
    Ball_and_Plate_MicroLabBox_student_B.Inc2Pos_p =
      Ball_and_Plate_MicroLabBox_student_P.Inc2Pos_Gain *
      Ball_and_Plate_MicroLabBox_student_B.SFunction1_o1_p;

    /* Sum: '<S16>/AbsPosition' incorporates:
     *  Constant: '<S16>/Pos_offset'
     *  Constant: '<S16>/ZP'
     */
    Ball_and_Plate_MicroLabBox_student_B.AbsPosition_m =
      (Ball_and_Plate_MicroLabBox_student_B.Inc2Pos_p -
       Ball_and_Plate_MicroLabBox_student_P.ZP_Value) -
      Ball_and_Plate_MicroLabBox_student_P.Pos_offset_Value;

    /* Abs: '<S23>/Abs' */
    Ball_and_Plate_MicroLabBox_student_B.Abs_h = fabs
      (Ball_and_Plate_MicroLabBox_student_B.SFunction1_o2_d);

    /* Outputs for Enabled SubSystem: '<S23>/Enabled Subsystem' */
    /* Constant: '<S3>/Init  ' */
    Ball_and_Plate_EnabledSubsystem(Ball_and_Plate_MicroLabBox_student_M,
      Ball_and_Plate_MicroLabBox_student_P.Init_Value,
      &Ball_and_Plate_MicroLabBox_student_B.EnabledSubsystem,
      &Ball_and_Plate_MicroLabBox_student_DW.EnabledSubsystem,
      &Ball_and_Plate_MicroLabBox_student_P.EnabledSubsystem);

    /* End of Outputs for SubSystem: '<S23>/Enabled Subsystem' */

    /* RelationalOperator: '<S23>/Relational Operator1' incorporates:
     *  Constant: '<S23>/Const'
     */
    Ball_and_Plate_MicroLabBox_student_B.RelationalOperator1_f =
      (Ball_and_Plate_MicroLabBox_student_B.Abs_h <=
       Ball_and_Plate_MicroLabBox_student_P.Const_Value);
  }

  /* TransferFcn: '<S23>/Transfer Fcn' */
  Ball_and_Plate_MicroLabBox_student_B.TransferFcn_f = 0.0;
  Ball_and_Plate_MicroLabBox_student_B.TransferFcn_f +=
    Ball_and_Plate_MicroLabBox_student_P.TransferFcn_C *
    Ball_and_Plate_MicroLabBox_student_X.TransferFcn_CSTATE_l;

  /* RelationalOperator: '<S23>/Relational Operator' incorporates:
   *  Constant: '<S23>/Const1'
   */
  Ball_and_Plate_MicroLabBox_student_B.RelationalOperator_a =
    (Ball_and_Plate_MicroLabBox_student_P.Const1_Value >=
     Ball_and_Plate_MicroLabBox_student_B.TransferFcn_f);

  /* Logic: '<S23>/Logical Operator2' incorporates:
   *  Constant: '<S3>/Init  '
   */
  Ball_and_Plate_MicroLabBox_student_B.LogicalOperator2_a =
    (Ball_and_Plate_MicroLabBox_student_B.RelationalOperator1_f &&
     Ball_and_Plate_MicroLabBox_student_B.RelationalOperator_a &&
     (Ball_and_Plate_MicroLabBox_student_P.Init_Value != 0.0));

  /* Outputs for Enabled SubSystem: '<S23>/Enabled Subsystem1' */
  Ball_and_Plat_EnabledSubsystem1(Ball_and_Plate_MicroLabBox_student_M,
    Ball_and_Plate_MicroLabBox_student_B.LogicalOperator2_a,
    Ball_and_Plate_MicroLabBox_student_B.Inc2Pos_p,
    &Ball_and_Plate_MicroLabBox_student_B.EnabledSubsystem1,
    &Ball_and_Plate_MicroLabBox_student_DW.EnabledSubsystem1);

  /* End of Outputs for SubSystem: '<S23>/Enabled Subsystem1' */
  if (rtmIsMajorTimeStep(Ball_and_Plate_MicroLabBox_student_M) &&
      Ball_and_Plate_MicroLabBox_student_M->Timing.TaskCounters.TID[1] == 0) {
    /* Gain: '<S16>/mm2m' */
    Ball_and_Plate_MicroLabBox_student_B.mm2m_po =
      Ball_and_Plate_MicroLabBox_student_P.mm2m_Gain *
      Ball_and_Plate_MicroLabBox_student_B.AbsPosition_m;
  }

  /* End of Outputs for SubSystem: '<S9>/Position Measurement' */
  if (rtmIsMajorTimeStep(Ball_and_Plate_MicroLabBox_student_M) &&
      Ball_and_Plate_MicroLabBox_student_M->Timing.TaskCounters.TID[1] == 0) {
    /* Product: '<S3>/Divide1' */
    Ball_and_Plate_MicroLabBox_student_B.Divide1 =
      Ball_and_Plate_MicroLabBox_student_B.mm2m_po /
      Ball_and_Plate_MicroLabBox_student_B.Add1;
  }

  /* Outputs for Atomic SubSystem: '<S10>/Position Measurement' */
  if (rtmIsMajorTimeStep(Ball_and_Plate_MicroLabBox_student_M) &&
      Ball_and_Plate_MicroLabBox_student_M->Timing.TaskCounters.TID[1] == 0) {
    /* Outputs for Triggered SubSystem: '<S30>/EMC_ENCODER_POS_SET_BL1' incorporates:
     *  TriggerPort: '<S36>/Trigger'
     */
    if (rtmIsMajorTimeStep(Ball_and_Plate_MicroLabBox_student_M)) {
      /* Constant: '<S3>/Init  ' */
      zcEvent = rt_ZCFcn(RISING_ZERO_CROSSING,
                         &Ball_and_Plate_MicroLabBox_student_PrevZCX.EMC_ENCODER_POS_SET_BL1_Trig__f,
                         (Ball_and_Plate_MicroLabBox_student_P.Init_Value));
      if (zcEvent != NO_ZCEVENT) {
        /* S-Function (rti_commonblock): '<S36>/S-Function1' incorporates:
         *  Constant: '<S30>/Constant'
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

    /* End of Outputs for SubSystem: '<S30>/EMC_ENCODER_POS_SET_BL1' */

    /* S-Function (rti_commonblock): '<S35>/S-Function1' */
    /* This comment workarounds a code generation problem */

    /* Gain: '<S30>/Inc2Pos' */
    Ball_and_Plate_MicroLabBox_student_B.Inc2Pos_e =
      Ball_and_Plate_MicroLabBox_student_P.Inc2Pos_Gain_k *
      Ball_and_Plate_MicroLabBox_student_B.SFunction1_o1_i;

    /* Sum: '<S30>/AbsPosition' incorporates:
     *  Constant: '<S30>/Pos_offset'
     *  Constant: '<S30>/ZP'
     */
    Ball_and_Plate_MicroLabBox_student_B.AbsPosition_i =
      (Ball_and_Plate_MicroLabBox_student_B.Inc2Pos_e -
       Ball_and_Plate_MicroLabBox_student_P.ZP_Value_m) -
      Ball_and_Plate_MicroLabBox_student_P.Pos_offset_Value_k;

    /* Abs: '<S37>/Abs' */
    Ball_and_Plate_MicroLabBox_student_B.Abs_j = fabs
      (Ball_and_Plate_MicroLabBox_student_B.SFunction1_o2_p);

    /* Outputs for Enabled SubSystem: '<S37>/Enabled Subsystem' */
    /* Constant: '<S3>/Init  ' */
    Ball_and_Plate_EnabledSubsystem(Ball_and_Plate_MicroLabBox_student_M,
      Ball_and_Plate_MicroLabBox_student_P.Init_Value,
      &Ball_and_Plate_MicroLabBox_student_B.EnabledSubsystem_a,
      &Ball_and_Plate_MicroLabBox_student_DW.EnabledSubsystem_a,
      &Ball_and_Plate_MicroLabBox_student_P.EnabledSubsystem_a);

    /* End of Outputs for SubSystem: '<S37>/Enabled Subsystem' */

    /* RelationalOperator: '<S37>/Relational Operator1' incorporates:
     *  Constant: '<S37>/Const'
     */
    Ball_and_Plate_MicroLabBox_student_B.RelationalOperator1_c =
      (Ball_and_Plate_MicroLabBox_student_B.Abs_j <=
       Ball_and_Plate_MicroLabBox_student_P.Const_Value_g);
  }

  /* TransferFcn: '<S37>/Transfer Fcn' */
  Ball_and_Plate_MicroLabBox_student_B.TransferFcn_o = 0.0;
  Ball_and_Plate_MicroLabBox_student_B.TransferFcn_o +=
    Ball_and_Plate_MicroLabBox_student_P.TransferFcn_C_e *
    Ball_and_Plate_MicroLabBox_student_X.TransferFcn_CSTATE_h;

  /* RelationalOperator: '<S37>/Relational Operator' incorporates:
   *  Constant: '<S37>/Const1'
   */
  Ball_and_Plate_MicroLabBox_student_B.RelationalOperator_n =
    (Ball_and_Plate_MicroLabBox_student_P.Const1_Value_a >=
     Ball_and_Plate_MicroLabBox_student_B.TransferFcn_o);

  /* Logic: '<S37>/Logical Operator2' incorporates:
   *  Constant: '<S3>/Init  '
   */
  Ball_and_Plate_MicroLabBox_student_B.LogicalOperator2_l =
    (Ball_and_Plate_MicroLabBox_student_B.RelationalOperator1_c &&
     Ball_and_Plate_MicroLabBox_student_B.RelationalOperator_n &&
     (Ball_and_Plate_MicroLabBox_student_P.Init_Value != 0.0));

  /* Outputs for Enabled SubSystem: '<S37>/Enabled Subsystem1' */
  Ball_and_Plat_EnabledSubsystem1(Ball_and_Plate_MicroLabBox_student_M,
    Ball_and_Plate_MicroLabBox_student_B.LogicalOperator2_l,
    Ball_and_Plate_MicroLabBox_student_B.Inc2Pos_e,
    &Ball_and_Plate_MicroLabBox_student_B.EnabledSubsystem1_f,
    &Ball_and_Plate_MicroLabBox_student_DW.EnabledSubsystem1_f);

  /* End of Outputs for SubSystem: '<S37>/Enabled Subsystem1' */
  if (rtmIsMajorTimeStep(Ball_and_Plate_MicroLabBox_student_M) &&
      Ball_and_Plate_MicroLabBox_student_M->Timing.TaskCounters.TID[1] == 0) {
    /* Gain: '<S30>/mm2m' */
    Ball_and_Plate_MicroLabBox_student_B.mm2m_p =
      Ball_and_Plate_MicroLabBox_student_P.mm2m_Gain_f *
      Ball_and_Plate_MicroLabBox_student_B.AbsPosition_i;
  }

  /* End of Outputs for SubSystem: '<S10>/Position Measurement' */
  if (rtmIsMajorTimeStep(Ball_and_Plate_MicroLabBox_student_M) &&
      Ball_and_Plate_MicroLabBox_student_M->Timing.TaskCounters.TID[1] == 0) {
    /* Product: '<S3>/Divide2' */
    Ball_and_Plate_MicroLabBox_student_B.Divide2 =
      Ball_and_Plate_MicroLabBox_student_B.mm2m_p /
      Ball_and_Plate_MicroLabBox_student_B.Add2;
  }

  /* Outputs for Atomic SubSystem: '<S11>/Position Measurement' */
  if (rtmIsMajorTimeStep(Ball_and_Plate_MicroLabBox_student_M) &&
      Ball_and_Plate_MicroLabBox_student_M->Timing.TaskCounters.TID[1] == 0) {
    /* Outputs for Triggered SubSystem: '<S45>/EMC_ENCODER_POS_SET_BL1' incorporates:
     *  TriggerPort: '<S52>/Trigger'
     */
    if (rtmIsMajorTimeStep(Ball_and_Plate_MicroLabBox_student_M)) {
      /* Constant: '<S3>/Init  ' */
      zcEvent = rt_ZCFcn(RISING_ZERO_CROSSING,
                         &Ball_and_Plate_MicroLabBox_student_PrevZCX.EMC_ENCODER_POS_SET_BL1_Trig_ZC,
                         (Ball_and_Plate_MicroLabBox_student_P.Init_Value));
      if (zcEvent != NO_ZCEVENT) {
        /* S-Function (rti_commonblock): '<S52>/S-Function1' incorporates:
         *  Constant: '<S45>/Constant'
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

    /* End of Outputs for SubSystem: '<S45>/EMC_ENCODER_POS_SET_BL1' */

    /* S-Function (rti_commonblock): '<S51>/S-Function1' */
    /* This comment workarounds a code generation problem */

    /* Gain: '<S45>/Inc2Pos' */
    Ball_and_Plate_MicroLabBox_student_B.Inc2Pos =
      Ball_and_Plate_MicroLabBox_student_P.Inc2Pos_Gain_h *
      Ball_and_Plate_MicroLabBox_student_B.SFunction1_o1;

    /* Sum: '<S45>/AbsPosition' incorporates:
     *  Constant: '<S45>/Pos_offset'
     *  Constant: '<S45>/ZP'
     */
    Ball_and_Plate_MicroLabBox_student_B.AbsPosition =
      (Ball_and_Plate_MicroLabBox_student_B.Inc2Pos -
       Ball_and_Plate_MicroLabBox_student_P.ZP_Value_p) -
      Ball_and_Plate_MicroLabBox_student_P.Pos_offset_Value_f;

    /* Abs: '<S53>/Abs' */
    Ball_and_Plate_MicroLabBox_student_B.Abs = fabs
      (Ball_and_Plate_MicroLabBox_student_B.SFunction1_o2);

    /* Outputs for Enabled SubSystem: '<S53>/Enabled Subsystem' */
    /* Constant: '<S3>/Init  ' */
    Ball_and_Plate_EnabledSubsystem(Ball_and_Plate_MicroLabBox_student_M,
      Ball_and_Plate_MicroLabBox_student_P.Init_Value,
      &Ball_and_Plate_MicroLabBox_student_B.EnabledSubsystem_n,
      &Ball_and_Plate_MicroLabBox_student_DW.EnabledSubsystem_n,
      &Ball_and_Plate_MicroLabBox_student_P.EnabledSubsystem_n);

    /* End of Outputs for SubSystem: '<S53>/Enabled Subsystem' */

    /* RelationalOperator: '<S53>/Relational Operator1' incorporates:
     *  Constant: '<S53>/Const'
     */
    Ball_and_Plate_MicroLabBox_student_B.RelationalOperator1 =
      (Ball_and_Plate_MicroLabBox_student_B.Abs <=
       Ball_and_Plate_MicroLabBox_student_P.Const_Value_m);
  }

  /* TransferFcn: '<S53>/Transfer Fcn' */
  Ball_and_Plate_MicroLabBox_student_B.TransferFcn = 0.0;
  Ball_and_Plate_MicroLabBox_student_B.TransferFcn +=
    Ball_and_Plate_MicroLabBox_student_P.TransferFcn_C_c *
    Ball_and_Plate_MicroLabBox_student_X.TransferFcn_CSTATE;

  /* RelationalOperator: '<S53>/Relational Operator' incorporates:
   *  Constant: '<S53>/Const1'
   */
  Ball_and_Plate_MicroLabBox_student_B.RelationalOperator =
    (Ball_and_Plate_MicroLabBox_student_P.Const1_Value_p >=
     Ball_and_Plate_MicroLabBox_student_B.TransferFcn);

  /* Logic: '<S53>/Logical Operator2' incorporates:
   *  Constant: '<S3>/Init  '
   */
  Ball_and_Plate_MicroLabBox_student_B.LogicalOperator2 =
    (Ball_and_Plate_MicroLabBox_student_B.RelationalOperator1 &&
     Ball_and_Plate_MicroLabBox_student_B.RelationalOperator &&
     (Ball_and_Plate_MicroLabBox_student_P.Init_Value != 0.0));

  /* Outputs for Enabled SubSystem: '<S53>/Enabled Subsystem1' */
  Ball_and_Plat_EnabledSubsystem1(Ball_and_Plate_MicroLabBox_student_M,
    Ball_and_Plate_MicroLabBox_student_B.LogicalOperator2,
    Ball_and_Plate_MicroLabBox_student_B.Inc2Pos,
    &Ball_and_Plate_MicroLabBox_student_B.EnabledSubsystem1_b,
    &Ball_and_Plate_MicroLabBox_student_DW.EnabledSubsystem1_b);

  /* End of Outputs for SubSystem: '<S53>/Enabled Subsystem1' */
  if (rtmIsMajorTimeStep(Ball_and_Plate_MicroLabBox_student_M) &&
      Ball_and_Plate_MicroLabBox_student_M->Timing.TaskCounters.TID[1] == 0) {
    /* Gain: '<S45>/mm2m' */
    Ball_and_Plate_MicroLabBox_student_B.mm2m =
      Ball_and_Plate_MicroLabBox_student_P.mm2m_Gain_o *
      Ball_and_Plate_MicroLabBox_student_B.AbsPosition;
  }

  /* End of Outputs for SubSystem: '<S11>/Position Measurement' */
  if (rtmIsMajorTimeStep(Ball_and_Plate_MicroLabBox_student_M) &&
      Ball_and_Plate_MicroLabBox_student_M->Timing.TaskCounters.TID[1] == 0) {
    /* Product: '<S3>/Divide3' */
    Ball_and_Plate_MicroLabBox_student_B.Divide3 =
      Ball_and_Plate_MicroLabBox_student_B.mm2m /
      Ball_and_Plate_MicroLabBox_student_B.Add3;

    /* MATLAB Function: '<S13>/MATLAB Function' incorporates:
     *  Constant: '<S13>/Constant4'
     *  Constant: '<S13>/start_m1 '
     */
    Ball_and_Plate_M_MATLABFunction
      (Ball_and_Plate_MicroLabBox_student_P.start_m1_Value,
       Ball_and_Plate_MicroLabBox_student_P.path,
       &Ball_and_Plate_MicroLabBox_student_B.sf_MATLABFunction,
       &Ball_and_Plate_MicroLabBox_student_DW.sf_MATLABFunction);

    /* Sum: '<S9>/Sum1' */
    Ball_and_Plate_MicroLabBox_student_B.Sum1 =
      Ball_and_Plate_MicroLabBox_student_B.sf_MATLABFunction.y -
      Ball_and_Plate_MicroLabBox_student_B.mm2m_po;

    /* Gain: '<S12>/Gain1' */
    Ball_and_Plate_MicroLabBox_student_B.Gain1 =
      Ball_and_Plate_MicroLabBox_student_P.Gain1_Gain *
      Ball_and_Plate_MicroLabBox_student_B.Sum1;

    /* S-Function (dleadlag): '<S12>/Dctleadlag2' */

    /* Level2 S-Function Block: '<S12>/Dctleadlag2' (dleadlag) */
    {
      SimStruct *rts = Ball_and_Plate_MicroLabBox_student_M->childSfunctions[0];
      sfcnOutputs(rts,1);
    }

    /* S-Function (dlowpass1): '<S12>/Dct1lowpass3' */

    /* Level2 S-Function Block: '<S12>/Dct1lowpass3' (dlowpass1) */
    {
      SimStruct *rts = Ball_and_Plate_MicroLabBox_student_M->childSfunctions[1];
      sfcnOutputs(rts,1);
    }

    /* Switch: '<S9>/Switch2' incorporates:
     *  Constant: '<S3>/controller_disable'
     *  Constant: '<S9>/Constant'
     */
    if (Ball_and_Plate_MicroLabBox_student_P.controller_disable_Value >=
        Ball_and_Plate_MicroLabBox_student_P.Switch2_Threshold) {
      Ball_and_Plate_MicroLabBox_student_B.Switch2 =
        Ball_and_Plate_MicroLabBox_student_P.Constant_Value;
    } else {
      Ball_and_Plate_MicroLabBox_student_B.Switch2 =
        Ball_and_Plate_MicroLabBox_student_B.Dct1lowpass3;
    }

    /* End of Switch: '<S9>/Switch2' */

    /* MATLAB Function: '<S14>/MATLAB Function' incorporates:
     *  Constant: '<S14>/Constant'
     *  Constant: '<S9>/1_to_enable_id'
     */
    Ball_and_Plate_MATLABFunction_e
      (Ball_and_Plate_MicroLabBox_student_P.u_to_enable_id_Value,
       Ball_and_Plate_MicroLabBox_student_P.uA,
       &Ball_and_Plate_MicroLabBox_student_B.sf_MATLABFunction_e,
       &Ball_and_Plate_MicroLabBox_student_DW.sf_MATLABFunction_e);

    /* Sum: '<S9>/Sum' */
    Ball_and_Plate_MicroLabBox_student_B.Sum =
      Ball_and_Plate_MicroLabBox_student_B.Switch2 +
      Ball_and_Plate_MicroLabBox_student_B.sf_MATLABFunction_e.y;
  }

  /* Switch: '<S9>/Switch3' incorporates:
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

  /* End of Switch: '<S9>/Switch3' */

  /* Outputs for Atomic SubSystem: '<S9>/Outputs to Amplifier' */

  /* Saturate: '<S15>/Saturation' */
  u0 = Ball_and_Plate_MicroLabBox_student_B.Switch3;
  u1 = Ball_and_Plate_MicroLabBox_student_P.Saturation_LowerSat;
  u2 = Ball_and_Plate_MicroLabBox_student_P.Saturation_UpperSat;
  if (u0 > u2) {
    Ball_and_Plate_MicroLabBox_student_B.Saturation_a = u2;
  } else if (u0 < u1) {
    Ball_and_Plate_MicroLabBox_student_B.Saturation_a = u1;
  } else {
    Ball_and_Plate_MicroLabBox_student_B.Saturation_a = u0;
  }

  /* End of Saturate: '<S15>/Saturation' */

  /* Gain: '<S15>/Current2V' */
  Ball_and_Plate_MicroLabBox_student_B.Current2V_h =
    Ball_and_Plate_MicroLabBox_student_P.Current2V_Gain *
    Ball_and_Plate_MicroLabBox_student_B.Saturation_a;

  /* Gain: '<S15>/DSPscale' */
  Ball_and_Plate_MicroLabBox_student_B.DSPscale_m =
    Ball_and_Plate_MicroLabBox_student_P.DSPscale_Gain *
    Ball_and_Plate_MicroLabBox_student_B.Current2V_h;
  if (rtmIsMajorTimeStep(Ball_and_Plate_MicroLabBox_student_M) &&
      Ball_and_Plate_MicroLabBox_student_M->Timing.TaskCounters.TID[1] == 0) {
    /* S-Function (rti_commonblock): '<S20>/S-Function1' */
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

  /* End of Outputs for SubSystem: '<S9>/Outputs to Amplifier' */
  if (rtmIsMajorTimeStep(Ball_and_Plate_MicroLabBox_student_M) &&
      Ball_and_Plate_MicroLabBox_student_M->Timing.TaskCounters.TID[1] == 0) {
    /* MATLAB Function: '<S13>/MATLAB Function2' incorporates:
     *  Constant: '<S9>/1_no_0_init_motion'
     *  Constant: '<S9>/Constant4'
     */
    Ball_and_Plate__MATLABFunction2(0.0,
      Ball_and_Plate_MicroLabBox_student_P.u_no_0_init_motion_Value,
      Ball_and_Plate_MicroLabBox_student_P.Constant4_Value,
      &Ball_and_Plate_MicroLabBox_student_B.sf_MATLABFunction2);

    /* Switch: '<S13>/Switch' incorporates:
     *  Constant: '<S13>/Constant3'
     *  Constant: '<S9>/1_no_0_init_motion'
     *  Constant: '<S9>/Constant4'
     */
    if (Ball_and_Plate_MicroLabBox_student_P.u_no_0_init_motion_Value >
        Ball_and_Plate_MicroLabBox_student_P.Switch_Threshold) {
      Ball_and_Plate_MicroLabBox_student_B.Switch =
        Ball_and_Plate_MicroLabBox_student_P.Constant4_Value;
    } else {
      Ball_and_Plate_MicroLabBox_student_B.Switch =
        Ball_and_Plate_MicroLabBox_student_P.Constant3_Value;
    }

    /* End of Switch: '<S13>/Switch' */

    /* Constant: '<S13>/Constant1' */
    Ball_and_Plate_MicroLabBox_student_B.Constant1 =
      Ball_and_Plate_MicroLabBox_student_P.Constant1_Value_i;

    /* Constant: '<S13>/Constant2' */
    Ball_and_Plate_MicroLabBox_student_B.Constant2 =
      Ball_and_Plate_MicroLabBox_student_P.Constant2_Value_e;

    /* MATLAB Function: '<S27>/MATLAB Function' incorporates:
     *  Constant: '<S27>/Constant4'
     *  Constant: '<S27>/start_m1 '
     */
    Ball_and_Plate_M_MATLABFunction
      (Ball_and_Plate_MicroLabBox_student_P.start_m1_Value_h,
       Ball_and_Plate_MicroLabBox_student_P.path,
       &Ball_and_Plate_MicroLabBox_student_B.sf_MATLABFunction_k,
       &Ball_and_Plate_MicroLabBox_student_DW.sf_MATLABFunction_k);

    /* Sum: '<S10>/Sum1' */
    Ball_and_Plate_MicroLabBox_student_B.Sum1_f =
      Ball_and_Plate_MicroLabBox_student_B.sf_MATLABFunction_k.y -
      Ball_and_Plate_MicroLabBox_student_B.mm2m_p;

    /* Gain: '<S26>/Gain1' */
    Ball_and_Plate_MicroLabBox_student_B.Gain1_o =
      Ball_and_Plate_MicroLabBox_student_P.Gain1_Gain_n *
      Ball_and_Plate_MicroLabBox_student_B.Sum1_f;

    /* S-Function (dleadlag): '<S26>/Dctleadlag2' */

    /* Level2 S-Function Block: '<S26>/Dctleadlag2' (dleadlag) */
    {
      SimStruct *rts = Ball_and_Plate_MicroLabBox_student_M->childSfunctions[2];
      sfcnOutputs(rts,1);
    }

    /* S-Function (dlowpass1): '<S26>/Dct1lowpass3' */

    /* Level2 S-Function Block: '<S26>/Dct1lowpass3' (dlowpass1) */
    {
      SimStruct *rts = Ball_and_Plate_MicroLabBox_student_M->childSfunctions[3];
      sfcnOutputs(rts,1);
    }

    /* Switch: '<S10>/Switch2' incorporates:
     *  Constant: '<S10>/Constant1'
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

    /* End of Switch: '<S10>/Switch2' */

    /* MATLAB Function: '<S28>/MATLAB Function' incorporates:
     *  Constant: '<S10>/1_to_enable_id'
     *  Constant: '<S28>/Constant'
     */
    Ball_and_Plate_MATLABFunction_e
      (Ball_and_Plate_MicroLabBox_student_P.u_to_enable_id_Value_b,
       Ball_and_Plate_MicroLabBox_student_P.uB,
       &Ball_and_Plate_MicroLabBox_student_B.sf_MATLABFunction_ko,
       &Ball_and_Plate_MicroLabBox_student_DW.sf_MATLABFunction_ko);

    /* Sum: '<S10>/Sum' */
    Ball_and_Plate_MicroLabBox_student_B.Sum_g =
      Ball_and_Plate_MicroLabBox_student_B.Switch2_p +
      Ball_and_Plate_MicroLabBox_student_B.sf_MATLABFunction_ko.y;
  }

  /* Switch: '<S10>/Switch3' incorporates:
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

  /* End of Switch: '<S10>/Switch3' */

  /* Outputs for Atomic SubSystem: '<S10>/Outputs to Amplifier' */

  /* Saturate: '<S29>/Saturation' */
  u0 = Ball_and_Plate_MicroLabBox_student_B.Switch3_d;
  u1 = Ball_and_Plate_MicroLabBox_student_P.Saturation_LowerSat_f;
  u2 = Ball_and_Plate_MicroLabBox_student_P.Saturation_UpperSat_e;
  if (u0 > u2) {
    Ball_and_Plate_MicroLabBox_student_B.Saturation_l = u2;
  } else if (u0 < u1) {
    Ball_and_Plate_MicroLabBox_student_B.Saturation_l = u1;
  } else {
    Ball_and_Plate_MicroLabBox_student_B.Saturation_l = u0;
  }

  /* End of Saturate: '<S29>/Saturation' */

  /* Gain: '<S29>/Current2V' */
  Ball_and_Plate_MicroLabBox_student_B.Current2V_f =
    Ball_and_Plate_MicroLabBox_student_P.Current2V_Gain_f *
    Ball_and_Plate_MicroLabBox_student_B.Saturation_l;

  /* Gain: '<S29>/DSPscale' */
  Ball_and_Plate_MicroLabBox_student_B.DSPscale_o =
    Ball_and_Plate_MicroLabBox_student_P.DSPscale_Gain_d *
    Ball_and_Plate_MicroLabBox_student_B.Current2V_f;
  if (rtmIsMajorTimeStep(Ball_and_Plate_MicroLabBox_student_M) &&
      Ball_and_Plate_MicroLabBox_student_M->Timing.TaskCounters.TID[1] == 0) {
    /* S-Function (rti_commonblock): '<S34>/S-Function1' */
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

  /* End of Outputs for SubSystem: '<S10>/Outputs to Amplifier' */
  if (rtmIsMajorTimeStep(Ball_and_Plate_MicroLabBox_student_M) &&
      Ball_and_Plate_MicroLabBox_student_M->Timing.TaskCounters.TID[1] == 0) {
    /* MATLAB Function: '<S27>/MATLAB Function2' incorporates:
     *  Constant: '<S10>/1_no_0_init_motion'
     *  Constant: '<S10>/Constant4'
     */
    Ball_and_Plate__MATLABFunction2(0.0,
      Ball_and_Plate_MicroLabBox_student_P.u_no_0_init_motion_Value_e,
      Ball_and_Plate_MicroLabBox_student_P.Constant4_Value_b,
      &Ball_and_Plate_MicroLabBox_student_B.sf_MATLABFunction2_m);

    /* Switch: '<S27>/Switch' incorporates:
     *  Constant: '<S10>/1_no_0_init_motion'
     *  Constant: '<S10>/Constant4'
     *  Constant: '<S27>/Constant3'
     */
    if (Ball_and_Plate_MicroLabBox_student_P.u_no_0_init_motion_Value_e >
        Ball_and_Plate_MicroLabBox_student_P.Switch_Threshold_k) {
      Ball_and_Plate_MicroLabBox_student_B.Switch_n =
        Ball_and_Plate_MicroLabBox_student_P.Constant4_Value_b;
    } else {
      Ball_and_Plate_MicroLabBox_student_B.Switch_n =
        Ball_and_Plate_MicroLabBox_student_P.Constant3_Value_i;
    }

    /* End of Switch: '<S27>/Switch' */

    /* Constant: '<S27>/Constant1' */
    Ball_and_Plate_MicroLabBox_student_B.Constant1_c =
      Ball_and_Plate_MicroLabBox_student_P.Constant1_Value_p;

    /* Constant: '<S27>/Constant2' */
    Ball_and_Plate_MicroLabBox_student_B.Constant2_b =
      Ball_and_Plate_MicroLabBox_student_P.Constant2_Value_g;

    /* MATLAB Function: '<S41>/MATLAB Function' incorporates:
     *  Constant: '<S41>/Constant4'
     *  Constant: '<S41>/start_m1 '
     */
    Ball_and_Plate_M_MATLABFunction
      (Ball_and_Plate_MicroLabBox_student_P.start_m1_Value_f,
       Ball_and_Plate_MicroLabBox_student_P.path,
       &Ball_and_Plate_MicroLabBox_student_B.sf_MATLABFunction_h,
       &Ball_and_Plate_MicroLabBox_student_DW.sf_MATLABFunction_h);

    /* Sum: '<S11>/Sum1' */
    Ball_and_Plate_MicroLabBox_student_B.Sum1_h =
      Ball_and_Plate_MicroLabBox_student_B.sf_MATLABFunction_h.y -
      Ball_and_Plate_MicroLabBox_student_B.mm2m;

    /* Gain: '<S40>/Gain1' */
    Ball_and_Plate_MicroLabBox_student_B.Gain1_e =
      Ball_and_Plate_MicroLabBox_student_P.Gain1_Gain_m *
      Ball_and_Plate_MicroLabBox_student_B.Sum1_h;

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

    /* Switch: '<S11>/Switch2' incorporates:
     *  Constant: '<S11>/Constant'
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

    /* End of Switch: '<S11>/Switch2' */

    /* MATLAB Function: '<S43>/MATLAB Function' incorporates:
     *  Constant: '<S11>/1_to_enable_id1'
     *  Constant: '<S43>/Constant'
     */
    Ball_and_Plate_MATLABFunction_e
      (Ball_and_Plate_MicroLabBox_student_P.u_to_enable_id1_Value,
       Ball_and_Plate_MicroLabBox_student_P.uC,
       &Ball_and_Plate_MicroLabBox_student_B.sf_MATLABFunction_hg,
       &Ball_and_Plate_MicroLabBox_student_DW.sf_MATLABFunction_hg);

    /* Sum: '<S11>/Sum' */
    Ball_and_Plate_MicroLabBox_student_B.Sum_h =
      Ball_and_Plate_MicroLabBox_student_B.Switch2_h +
      Ball_and_Plate_MicroLabBox_student_B.sf_MATLABFunction_hg.y;
  }

  /* Switch: '<S11>/Switch3' incorporates:
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

  /* End of Switch: '<S11>/Switch3' */

  /* Outputs for Atomic SubSystem: '<S11>/Outputs to Amplifier' */

  /* Saturate: '<S44>/Saturation' */
  u0 = Ball_and_Plate_MicroLabBox_student_B.Switch3_l;
  u1 = Ball_and_Plate_MicroLabBox_student_P.Saturation_LowerSat_p;
  u2 = Ball_and_Plate_MicroLabBox_student_P.Saturation_UpperSat_h;
  if (u0 > u2) {
    Ball_and_Plate_MicroLabBox_student_B.Saturation = u2;
  } else if (u0 < u1) {
    Ball_and_Plate_MicroLabBox_student_B.Saturation = u1;
  } else {
    Ball_and_Plate_MicroLabBox_student_B.Saturation = u0;
  }

  /* End of Saturate: '<S44>/Saturation' */

  /* Gain: '<S44>/Current2V' */
  Ball_and_Plate_MicroLabBox_student_B.Current2V =
    Ball_and_Plate_MicroLabBox_student_P.Current2V_Gain_p *
    Ball_and_Plate_MicroLabBox_student_B.Saturation;

  /* Gain: '<S44>/DSPscale' */
  Ball_and_Plate_MicroLabBox_student_B.DSPscale =
    Ball_and_Plate_MicroLabBox_student_P.DSPscale_Gain_h *
    Ball_and_Plate_MicroLabBox_student_B.Current2V;
  if (rtmIsMajorTimeStep(Ball_and_Plate_MicroLabBox_student_M) &&
      Ball_and_Plate_MicroLabBox_student_M->Timing.TaskCounters.TID[1] == 0) {
    /* S-Function (rti_commonblock): '<S50>/S-Function1' */
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

  /* End of Outputs for SubSystem: '<S11>/Outputs to Amplifier' */
  if (rtmIsMajorTimeStep(Ball_and_Plate_MicroLabBox_student_M) &&
      Ball_and_Plate_MicroLabBox_student_M->Timing.TaskCounters.TID[1] == 0) {
    /* MATLAB Function: '<S41>/MATLAB Function2' incorporates:
     *  Constant: '<S11>/1_no_0_init_motion'
     *  Constant: '<S11>/Constant4'
     */
    Ball_and_Plate__MATLABFunction2(0.0,
      Ball_and_Plate_MicroLabBox_student_P.u_no_0_init_motion_Value_n,
      Ball_and_Plate_MicroLabBox_student_P.Constant4_Value_c,
      &Ball_and_Plate_MicroLabBox_student_B.sf_MATLABFunction2_e);

    /* Switch: '<S41>/Switch' incorporates:
     *  Constant: '<S11>/1_no_0_init_motion'
     *  Constant: '<S11>/Constant4'
     *  Constant: '<S41>/Constant3'
     */
    if (Ball_and_Plate_MicroLabBox_student_P.u_no_0_init_motion_Value_n >
        Ball_and_Plate_MicroLabBox_student_P.Switch_Threshold_l) {
      Ball_and_Plate_MicroLabBox_student_B.Switch_e =
        Ball_and_Plate_MicroLabBox_student_P.Constant4_Value_c;
    } else {
      Ball_and_Plate_MicroLabBox_student_B.Switch_e =
        Ball_and_Plate_MicroLabBox_student_P.Constant3_Value_p;
    }

    /* End of Switch: '<S41>/Switch' */

    /* Constant: '<S41>/Constant1' */
    Ball_and_Plate_MicroLabBox_student_B.Constant1_h =
      Ball_and_Plate_MicroLabBox_student_P.Constant1_Value_f;

    /* Constant: '<S41>/Constant2' */
    Ball_and_Plate_MicroLabBox_student_B.Constant2_h =
      Ball_and_Plate_MicroLabBox_student_P.Constant2_Value_i;

    /* MATLAB Function: '<S42>/MATLAB Function' incorporates:
     *  Constant: '<S11>/1_to_enable_id'
     *  Constant: '<S42>/Constant'
     */
    Ball_and_Plate_MATLABFunction_e
      (Ball_and_Plate_MicroLabBox_student_P.u_to_enable_id_Value_m,
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

  /* Derivatives for Atomic SubSystem: '<S9>/Position Measurement' */
  /* Derivatives for TransferFcn: '<S23>/Transfer Fcn' */
  _rtXdot->TransferFcn_CSTATE_l = 0.0;
  _rtXdot->TransferFcn_CSTATE_l +=
    Ball_and_Plate_MicroLabBox_student_P.TransferFcn_A *
    Ball_and_Plate_MicroLabBox_student_X.TransferFcn_CSTATE_l;
  _rtXdot->TransferFcn_CSTATE_l +=
    Ball_and_Plate_MicroLabBox_student_B.EnabledSubsystem.OutportBufferForOut1;

  /* End of Derivatives for SubSystem: '<S9>/Position Measurement' */

  /* Derivatives for Atomic SubSystem: '<S10>/Position Measurement' */
  /* Derivatives for TransferFcn: '<S37>/Transfer Fcn' */
  _rtXdot->TransferFcn_CSTATE_h = 0.0;
  _rtXdot->TransferFcn_CSTATE_h +=
    Ball_and_Plate_MicroLabBox_student_P.TransferFcn_A_g *
    Ball_and_Plate_MicroLabBox_student_X.TransferFcn_CSTATE_h;
  _rtXdot->TransferFcn_CSTATE_h +=
    Ball_and_Plate_MicroLabBox_student_B.EnabledSubsystem_a.OutportBufferForOut1;

  /* End of Derivatives for SubSystem: '<S10>/Position Measurement' */

  /* Derivatives for Atomic SubSystem: '<S11>/Position Measurement' */
  /* Derivatives for TransferFcn: '<S53>/Transfer Fcn' */
  _rtXdot->TransferFcn_CSTATE = 0.0;
  _rtXdot->TransferFcn_CSTATE +=
    Ball_and_Plate_MicroLabBox_student_P.TransferFcn_A_m *
    Ball_and_Plate_MicroLabBox_student_X.TransferFcn_CSTATE;
  _rtXdot->TransferFcn_CSTATE +=
    Ball_and_Plate_MicroLabBox_student_B.EnabledSubsystem_n.OutportBufferForOut1;

  /* End of Derivatives for SubSystem: '<S11>/Position Measurement' */
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

    /* Level2 S-Function Block: Ball_and_Plate_MicroLabBox_student/<S12>/Dctleadlag2 (dleadlag) */
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

    /* Level2 S-Function Block: Ball_and_Plate_MicroLabBox_student/<S12>/Dct1lowpass3 (dlowpass1) */
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

    /* Level2 S-Function Block: Ball_and_Plate_MicroLabBox_student/<S26>/Dctleadlag2 (dleadlag) */
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

    /* Level2 S-Function Block: Ball_and_Plate_MicroLabBox_student/<S26>/Dct1lowpass3 (dlowpass1) */
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

  /* Start for Enabled SubSystem: '<S23>/Enabled Subsystem' */
  Ball_and_EnabledSubsystem_Start
    (&Ball_and_Plate_MicroLabBox_student_DW.EnabledSubsystem);

  /* End of Start for SubSystem: '<S23>/Enabled Subsystem' */

  /* Start for Enabled SubSystem: '<S23>/Enabled Subsystem1' */
  Ball_an_EnabledSubsystem1_Start
    (&Ball_and_Plate_MicroLabBox_student_DW.EnabledSubsystem1);

  /* End of Start for SubSystem: '<S23>/Enabled Subsystem1' */
  /* End of Start for SubSystem: '<S9>/Position Measurement' */

  /* Start for Enabled SubSystem: '<S37>/Enabled Subsystem' */
  Ball_and_EnabledSubsystem_Start
    (&Ball_and_Plate_MicroLabBox_student_DW.EnabledSubsystem_a);

  /* End of Start for SubSystem: '<S37>/Enabled Subsystem' */

  /* Start for Enabled SubSystem: '<S37>/Enabled Subsystem1' */
  Ball_an_EnabledSubsystem1_Start
    (&Ball_and_Plate_MicroLabBox_student_DW.EnabledSubsystem1_f);

  /* End of Start for SubSystem: '<S37>/Enabled Subsystem1' */
  /* End of Start for SubSystem: '<S10>/Position Measurement' */

  /* Start for Enabled SubSystem: '<S53>/Enabled Subsystem' */
  Ball_and_EnabledSubsystem_Start
    (&Ball_and_Plate_MicroLabBox_student_DW.EnabledSubsystem_n);

  /* End of Start for SubSystem: '<S53>/Enabled Subsystem' */

  /* Start for Enabled SubSystem: '<S53>/Enabled Subsystem1' */
  Ball_an_EnabledSubsystem1_Start
    (&Ball_and_Plate_MicroLabBox_student_DW.EnabledSubsystem1_b);

  /* End of Start for SubSystem: '<S53>/Enabled Subsystem1' */
  /* End of Start for SubSystem: '<S11>/Position Measurement' */

  /* Start for S-Function (dleadlag): '<S12>/Dctleadlag2' */
  /* Level2 S-Function Block: '<S12>/Dctleadlag2' (dleadlag) */
  {
    SimStruct *rts = Ball_and_Plate_MicroLabBox_student_M->childSfunctions[0];
    sfcnStart(rts);
    if (ssGetErrorStatus(rts) != (NULL))
      return;
  }

  /* Start for S-Function (dlowpass1): '<S12>/Dct1lowpass3' */
  /* Level2 S-Function Block: '<S12>/Dct1lowpass3' (dlowpass1) */
  {
    SimStruct *rts = Ball_and_Plate_MicroLabBox_student_M->childSfunctions[1];
    sfcnStart(rts);
    if (ssGetErrorStatus(rts) != (NULL))
      return;
  }

  /* Start for S-Function (dleadlag): '<S26>/Dctleadlag2' */
  /* Level2 S-Function Block: '<S26>/Dctleadlag2' (dleadlag) */
  {
    SimStruct *rts = Ball_and_Plate_MicroLabBox_student_M->childSfunctions[2];
    sfcnStart(rts);
    if (ssGetErrorStatus(rts) != (NULL))
      return;
  }

  /* Start for S-Function (dlowpass1): '<S26>/Dct1lowpass3' */
  /* Level2 S-Function Block: '<S26>/Dct1lowpass3' (dlowpass1) */
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

  Ball_and_Plate_MicroLabBox_student_PrevZCX.EMC_ENCODER_POS_SET_BL1_Trig_fh =
    UNINITIALIZED_ZCSIG;
  Ball_and_Plate_MicroLabBox_student_PrevZCX.EMC_ENCODER_POS_SET_BL1_Trig__f =
    UNINITIALIZED_ZCSIG;
  Ball_and_Plate_MicroLabBox_student_PrevZCX.EMC_ENCODER_POS_SET_BL1_Trig_ZC =
    UNINITIALIZED_ZCSIG;

  /* InitializeConditions for UnitDelay: '<S1>/UD' */
  Ball_and_Plate_MicroLabBox_student_DW.UD_DSTATE =
    Ball_and_Plate_MicroLabBox_student_P.Difference_ICPrevInput;

  /* SystemInitialize for Atomic SubSystem: '<S9>/Position Measurement' */
  /* InitializeConditions for TransferFcn: '<S23>/Transfer Fcn' */
  Ball_and_Plate_MicroLabBox_student_X.TransferFcn_CSTATE_l = 0.0;

  /* SystemInitialize for Enabled SubSystem: '<S23>/Enabled Subsystem' */
  Ball_and__EnabledSubsystem_Init
    (&Ball_and_Plate_MicroLabBox_student_B.EnabledSubsystem,
     &Ball_and_Plate_MicroLabBox_student_P.EnabledSubsystem);

  /* End of SystemInitialize for SubSystem: '<S23>/Enabled Subsystem' */

  /* SystemInitialize for Enabled SubSystem: '<S23>/Enabled Subsystem1' */
  Ball_and_EnabledSubsystem1_Init
    (&Ball_and_Plate_MicroLabBox_student_B.EnabledSubsystem1,
     &Ball_and_Plate_MicroLabBox_student_P.EnabledSubsystem1);

  /* End of SystemInitialize for SubSystem: '<S23>/Enabled Subsystem1' */
  /* End of SystemInitialize for SubSystem: '<S9>/Position Measurement' */

  /* SystemInitialize for Atomic SubSystem: '<S10>/Position Measurement' */
  /* InitializeConditions for TransferFcn: '<S37>/Transfer Fcn' */
  Ball_and_Plate_MicroLabBox_student_X.TransferFcn_CSTATE_h = 0.0;

  /* SystemInitialize for Enabled SubSystem: '<S37>/Enabled Subsystem' */
  Ball_and__EnabledSubsystem_Init
    (&Ball_and_Plate_MicroLabBox_student_B.EnabledSubsystem_a,
     &Ball_and_Plate_MicroLabBox_student_P.EnabledSubsystem_a);

  /* End of SystemInitialize for SubSystem: '<S37>/Enabled Subsystem' */

  /* SystemInitialize for Enabled SubSystem: '<S37>/Enabled Subsystem1' */
  Ball_and_EnabledSubsystem1_Init
    (&Ball_and_Plate_MicroLabBox_student_B.EnabledSubsystem1_f,
     &Ball_and_Plate_MicroLabBox_student_P.EnabledSubsystem1_f);

  /* End of SystemInitialize for SubSystem: '<S37>/Enabled Subsystem1' */
  /* End of SystemInitialize for SubSystem: '<S10>/Position Measurement' */

  /* SystemInitialize for Atomic SubSystem: '<S11>/Position Measurement' */
  /* InitializeConditions for TransferFcn: '<S53>/Transfer Fcn' */
  Ball_and_Plate_MicroLabBox_student_X.TransferFcn_CSTATE = 0.0;

  /* SystemInitialize for Enabled SubSystem: '<S53>/Enabled Subsystem' */
  Ball_and__EnabledSubsystem_Init
    (&Ball_and_Plate_MicroLabBox_student_B.EnabledSubsystem_n,
     &Ball_and_Plate_MicroLabBox_student_P.EnabledSubsystem_n);

  /* End of SystemInitialize for SubSystem: '<S53>/Enabled Subsystem' */

  /* SystemInitialize for Enabled SubSystem: '<S53>/Enabled Subsystem1' */
  Ball_and_EnabledSubsystem1_Init
    (&Ball_and_Plate_MicroLabBox_student_B.EnabledSubsystem1_b,
     &Ball_and_Plate_MicroLabBox_student_P.EnabledSubsystem1_b);

  /* End of SystemInitialize for SubSystem: '<S53>/Enabled Subsystem1' */
  /* End of SystemInitialize for SubSystem: '<S11>/Position Measurement' */

  /* SystemInitialize for MATLAB Function: '<S13>/MATLAB Function' */
  Ball_and_Pl_MATLABFunction_Init
    (&Ball_and_Plate_MicroLabBox_student_DW.sf_MATLABFunction);

  /* SystemInitialize for MATLAB Function: '<S14>/MATLAB Function' */
  Ball_and__MATLABFunction_o_Init
    (&Ball_and_Plate_MicroLabBox_student_DW.sf_MATLABFunction_e);

  /* SystemInitialize for MATLAB Function: '<S27>/MATLAB Function' */
  Ball_and_Pl_MATLABFunction_Init
    (&Ball_and_Plate_MicroLabBox_student_DW.sf_MATLABFunction_k);

  /* SystemInitialize for MATLAB Function: '<S28>/MATLAB Function' */
  Ball_and__MATLABFunction_o_Init
    (&Ball_and_Plate_MicroLabBox_student_DW.sf_MATLABFunction_ko);

  /* SystemInitialize for MATLAB Function: '<S41>/MATLAB Function' */
  Ball_and_Pl_MATLABFunction_Init
    (&Ball_and_Plate_MicroLabBox_student_DW.sf_MATLABFunction_h);

  /* SystemInitialize for MATLAB Function: '<S43>/MATLAB Function' */
  Ball_and__MATLABFunction_o_Init
    (&Ball_and_Plate_MicroLabBox_student_DW.sf_MATLABFunction_hg);

  /* SystemInitialize for MATLAB Function: '<S42>/MATLAB Function' */
  Ball_and__MATLABFunction_o_Init
    (&Ball_and_Plate_MicroLabBox_student_DW.sf_MATLABFunction_g);
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

  /* Terminate for Atomic SubSystem: '<S9>/Position Measurement' */
  /* Terminate for Triggered SubSystem: '<S16>/EMC_ENCODER_POS_SET_BL1' */
  /* Terminate for S-Function (rti_commonblock): '<S22>/S-Function1' incorporates:
   *  Constant: '<S16>/Constant'
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

  /* End of Terminate for SubSystem: '<S16>/EMC_ENCODER_POS_SET_BL1' */

  /* Terminate for S-Function (rti_commonblock): '<S21>/S-Function1' */

  /* --- Ball_and_Plate_MicroLabBox_student/Innerloop_Actuator/Motor_A/Position Measurement/EMC_ENCODER_BL1 --- */
  /* --- [RTIEMC, Encoder] - DIO class: 2 - Unit: 1 - Port: 1 - Channel: 1 --- */
  {
    /* Deactivates encoder interface functionality */
    DioCl2EncoderIn_stop(pRTIEmcEncoder_Unit_1_DioCl_2_Port_1_Ch1);
  }

  /* End of Terminate for SubSystem: '<S9>/Position Measurement' */

  /* Terminate for Atomic SubSystem: '<S10>/Position Measurement' */
  /* Terminate for Triggered SubSystem: '<S30>/EMC_ENCODER_POS_SET_BL1' */
  /* Terminate for S-Function (rti_commonblock): '<S36>/S-Function1' incorporates:
   *  Constant: '<S30>/Constant'
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

  /* End of Terminate for SubSystem: '<S30>/EMC_ENCODER_POS_SET_BL1' */

  /* Terminate for S-Function (rti_commonblock): '<S35>/S-Function1' */

  /* --- Ball_and_Plate_MicroLabBox_student/Innerloop_Actuator/Motor_B/Position Measurement/EMC_ENCODER_BL1 --- */
  /* --- [RTIEMC, Encoder] - DIO class: 2 - Unit: 2 - Port: 1 - Channel: 3 --- */
  {
    /* Deactivates encoder interface functionality */
    DioCl2EncoderIn_stop(pRTIEmcEncoder_Unit_2_DioCl_2_Port_1_Ch3);
  }

  /* End of Terminate for SubSystem: '<S10>/Position Measurement' */

  /* Terminate for Atomic SubSystem: '<S11>/Position Measurement' */
  /* Terminate for Triggered SubSystem: '<S45>/EMC_ENCODER_POS_SET_BL1' */
  /* Terminate for S-Function (rti_commonblock): '<S52>/S-Function1' incorporates:
   *  Constant: '<S45>/Constant'
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

  /* End of Terminate for SubSystem: '<S45>/EMC_ENCODER_POS_SET_BL1' */

  /* Terminate for S-Function (rti_commonblock): '<S51>/S-Function1' */

  /* --- Ball_and_Plate_MicroLabBox_student/Innerloop_Actuator/Motor_C/Position Measurement/EMC_ENCODER_BL1 --- */
  /* --- [RTIEMC, Encoder] - DIO class: 2 - Unit: 3 - Port: 1 - Channel: 5 --- */
  {
    /* Deactivates encoder interface functionality */
    DioCl2EncoderIn_stop(pRTIEmcEncoder_Unit_3_DioCl_2_Port_1_Ch5);
  }

  /* End of Terminate for SubSystem: '<S11>/Position Measurement' */

  /* Terminate for S-Function (dleadlag): '<S12>/Dctleadlag2' */
  /* Level2 S-Function Block: '<S12>/Dctleadlag2' (dleadlag) */
  {
    SimStruct *rts = Ball_and_Plate_MicroLabBox_student_M->childSfunctions[0];
    sfcnTerminate(rts);
  }

  /* Terminate for S-Function (dlowpass1): '<S12>/Dct1lowpass3' */
  /* Level2 S-Function Block: '<S12>/Dct1lowpass3' (dlowpass1) */
  {
    SimStruct *rts = Ball_and_Plate_MicroLabBox_student_M->childSfunctions[1];
    sfcnTerminate(rts);
  }

  /* Terminate for Atomic SubSystem: '<S9>/Outputs to Amplifier' */
  /* Terminate for S-Function (rti_commonblock): '<S20>/S-Function1' */

  /* --- Ball_and_Plate_MicroLabBox_student/Innerloop_Actuator/Motor_A/Outputs to Amplifier/DAC_CLASS1_BL1 --- */
  /* --- [RTI120X, DAC C1] - Channel: 1 --- */

  /* All channel outputs are set to high impedance state */
  DacCl1AnalogOut_setOutputHighZ(pRTIDacC1AnalogOut_Ch_1, DAC_CLASS1_HIGH_Z_ON);

  /* Deactivates AnalogOut functionality */
  DacCl1AnalogOut_stop(pRTIDacC1AnalogOut_Ch_1);

  /* End of Terminate for SubSystem: '<S9>/Outputs to Amplifier' */

  /* Terminate for S-Function (dleadlag): '<S26>/Dctleadlag2' */
  /* Level2 S-Function Block: '<S26>/Dctleadlag2' (dleadlag) */
  {
    SimStruct *rts = Ball_and_Plate_MicroLabBox_student_M->childSfunctions[2];
    sfcnTerminate(rts);
  }

  /* Terminate for S-Function (dlowpass1): '<S26>/Dct1lowpass3' */
  /* Level2 S-Function Block: '<S26>/Dct1lowpass3' (dlowpass1) */
  {
    SimStruct *rts = Ball_and_Plate_MicroLabBox_student_M->childSfunctions[3];
    sfcnTerminate(rts);
  }

  /* Terminate for Atomic SubSystem: '<S10>/Outputs to Amplifier' */
  /* Terminate for S-Function (rti_commonblock): '<S34>/S-Function1' */

  /* --- Ball_and_Plate_MicroLabBox_student/Innerloop_Actuator/Motor_B/Outputs to Amplifier/DAC_CLASS1_BL1 --- */
  /* --- [RTI120X, DAC C1] - Channel: 2 --- */

  /* All channel outputs are set to high impedance state */
  DacCl1AnalogOut_setOutputHighZ(pRTIDacC1AnalogOut_Ch_2, DAC_CLASS1_HIGH_Z_ON);

  /* Deactivates AnalogOut functionality */
  DacCl1AnalogOut_stop(pRTIDacC1AnalogOut_Ch_2);

  /* End of Terminate for SubSystem: '<S10>/Outputs to Amplifier' */

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

  /* Terminate for Atomic SubSystem: '<S11>/Outputs to Amplifier' */
  /* Terminate for S-Function (rti_commonblock): '<S50>/S-Function1' */

  /* --- Ball_and_Plate_MicroLabBox_student/Innerloop_Actuator/Motor_C/Outputs to Amplifier/DAC_CLASS1_BL1 --- */
  /* --- [RTI120X, DAC C1] - Channel: 3 --- */

  /* All channel outputs are set to high impedance state */
  DacCl1AnalogOut_setOutputHighZ(pRTIDacC1AnalogOut_Ch_3, DAC_CLASS1_HIGH_Z_ON);

  /* Deactivates AnalogOut functionality */
  DacCl1AnalogOut_stop(pRTIDacC1AnalogOut_Ch_3);

  /* End of Terminate for SubSystem: '<S11>/Outputs to Amplifier' */
}
