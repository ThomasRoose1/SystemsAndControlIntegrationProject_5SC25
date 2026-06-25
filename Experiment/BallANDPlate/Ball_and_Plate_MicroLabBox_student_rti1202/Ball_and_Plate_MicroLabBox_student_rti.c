/*********************** dSPACE target specific file *************************

   Include file Ball_and_Plate_MicroLabBox_student_rti.c:

   Definition of functions and variables for the system I/O and for
   the hardware and software interrupts used.

   RTI1202 7.15 (02-Nov-2020)
   Thu Jun 25 16:40:41 2026

   Copyright 2026, dSPACE GmbH. All rights reserved.

 *****************************************************************************/

#if !(defined(__RTI_SIMENGINE__) || defined(RTIMP_FRAME))
# error This file may be included only by the RTI(-MP) simulation engine.
#endif

/* Include the model header file. */
#include "Ball_and_Plate_MicroLabBox_student.h"
#include "Ball_and_Plate_MicroLabBox_student_private.h"

/* Defines for block output and parameter structure existence */
#define RTI_rtB_STRUCTURE_EXISTS       1
#define RTI_rtP_STRUCTURE_EXISTS       1
#define RTB_STRUCTURE_NAME             Ball_and_Plate_MicroLabBox_student_B
#define RTP_STRUCTURE_NAME             Ball_and_Plate_MicroLabBox_student_P

/* dSPACE generated includes for header files */
#include <brtenv.h>
#include <rtkernel.h>
#include <rti_assert.h>
#include <rtidefineddatatypes.h>
#include <dsIoEth.h>
#ifndef dsRtmGetNumSampleTimes
# define dsRtmGetNumSampleTimes(rtm)   3
#endif

#ifndef dsRtmGetTPtr
# define dsRtmGetTPtr(rtm)             ((rtm)->Timing.t)
#endif

#ifndef dsRtmSetTaskTime
# define dsRtmSetTaskTime(rtm, sti, val) (dsRtmGetTPtr((rtm))[sti] = (val))
#endif

/****** Definitions: task functions for timer tasks *********************/

/* Timer Task 1. (Base rate). */
static void rti_TIMERA(rtk_p_task_control_block task)
{
  /* Task entry code BEGIN */
  /* -- None. -- */
  /* Task entry code END */

  /* Task code. */
  baseRateService(task);

  /* Task exit code BEGIN */
  /* -- None. -- */
  /* Task exit code END */
}

/* ===== Declarations of RTI blocks ======================================== */
DacCl1AnalogOutSDrvObject *pRTIDacC1AnalogOut_Ch_1;
DioCl2EncoderInSDrvObject *pRTIEmcEncoder_Unit_1_DioCl_2_Port_1_Ch1;
DacCl1AnalogOutSDrvObject *pRTIDacC1AnalogOut_Ch_2;
DioCl2EncoderInSDrvObject *pRTIEmcEncoder_Unit_2_DioCl_2_Port_1_Ch3;
DacCl1AnalogOutSDrvObject *pRTIDacC1AnalogOut_Ch_3;
DioCl2EncoderInSDrvObject *pRTIEmcEncoder_Unit_3_DioCl_2_Port_1_Ch5;

/* ===== Definition of interface functions for simulation engine =========== */
#if GRTINTERFACE == 1
#ifdef MULTITASKING
# define dsIsSampleHit(RTM,sti)        rtmGetSampleHitPtr(RTM)[sti]
#else
# define dsIsSampleHit(RTM,sti)        rtmIsSampleHit(RTM,sti,0)
#endif

#else
#ifndef rtmStepTask
# define rtmStepTask(rtm, idx)         ((rtm)->Timing.TaskCounters.TID[(idx)] == 0)
#endif

# define dsIsSampleHit(RTM,sti)        rtmStepTask(RTM, sti)
#endif

#undef __INLINE
#if defined(_INLINE)
# define __INLINE                      static inline
#else
# define __INLINE                      static
#endif

/*Define additional variables*/
static time_T dsTFinal = -1.0;

#define dsGetTFinal(rtm)               (dsTFinal)

static time_T dsStepSize = 0.001;

# define dsGetStepSize(rtm)            (dsStepSize)

static void rti_mdl_initialize_host_services(void)
{
  DsDaq_Init(0, 32, 1);
}

static void rti_mdl_initialize_io_boards(void)
{
  /* Registering of RTI products and modules at VCM */
  {
    vcm_module_register(VCM_MID_RTI1202, (void *) 0,
                        VCM_TXT_RTI1202, 7, 15, 0,
                        VCM_VERSION_RELEASE, 0, 0, 0, VCM_CTRL_NO_ST);

    {
      vcm_module_descriptor_type* msg_mod_ptr;
      msg_mod_ptr = vcm_module_register(VCM_MID_MATLAB, (void *) 0,
        VCM_TXT_MATLAB, 9, 7, 0,
        VCM_VERSION_RELEASE, 0, 0, 0, VCM_CTRL_NO_ST);
      vcm_module_register(VCM_MID_SIMULINK, msg_mod_ptr,
                          VCM_TXT_SIMULINK, 10, 0, 0,
                          VCM_VERSION_RELEASE, 0, 0, 0, VCM_CTRL_NO_ST);
      vcm_module_register(VCM_MID_RTW, msg_mod_ptr,
                          VCM_TXT_RTW, 9, 2, 0,
                          VCM_VERSION_RELEASE, 0, 0, 0, VCM_CTRL_NO_ST);
      vcm_module_register(VCM_MID_STATEFLOW, msg_mod_ptr,
                          VCM_TXT_STATEFLOW, 10, 1, 0,
                          VCM_VERSION_RELEASE, 0, 0, 0, VCM_CTRL_NO_ST);
      vcm_module_register(VCM_MID_STATEFLOW_CODER, msg_mod_ptr,
                          VCM_TXT_STATEFLOW_CODER, 9, 2, 0,
                          VCM_VERSION_RELEASE, 0, 0, 0, VCM_CTRL_NO_ST);
    }
  }

  /* --- Ball_and_Plate_MicroLabBox_student/Innerloop_Actuator/Motor_A/Outputs to Amplifier/DAC_CLASS1_BL1 --- */
  /* --- [RTI120X, DAC C1] - Channel: 1 --- */
  {
    /* define a variable for IO error handling */
    UInt32 ioErrorCode = IOLIB_NO_ERROR;

    /* Init DAC CL1 AnalogOut driver object pRTIDacC1AnalogOut_Ch_1 */
    ioErrorCode = DacCl1AnalogOut_create(&pRTIDacC1AnalogOut_Ch_1,
      DAC_CLASS1_MASK_CH_1);
    if (ioErrorCode > IOLIB_NO_ERROR) {
      RTLIB_EXIT(1);
    }

    /* Apply all parameter settings. Finalize the construction of the AnalogOut driver object */
    ioErrorCode = DacCl1AnalogOut_apply(pRTIDacC1AnalogOut_Ch_1);
    if (ioErrorCode > IOLIB_NO_ERROR) {
      RTLIB_EXIT(1);
    }
  }

  /* --- Ball_and_Plate_MicroLabBox_student/Innerloop_Actuator/Motor_A/Position Measurement/EMC_ENCODER_BL1 --- */
  /* --- [RTIEMC, Encoder] - DIO class: 2 - Unit: 1 - Port: 1 - Channel: 1 --- */
  {
    /* define a variable for IO error handling */
    UInt32 ioErrorRetValue = IOLIB_NO_ERROR;

    /* Create EMC Encoder driver object pRTIEmcEncoder_Unit_1_DioCl_2_Port_1_Ch1 */
    ioErrorRetValue = DioCl2EncoderIn_create
      (&pRTIEmcEncoder_Unit_1_DioCl_2_Port_1_Ch1, DIO_CLASS2_CHANNEL_1,
       DIO_ENC_INSTANCE_1,DIO_ENC_IUSAGE_DISABLED);
    if (ioErrorRetValue > IOLIB_NO_ERROR) {
      RTLIB_EXIT(1);
    }

    /* -- Position measurement enabled -- */

    /* Sets number of encoder lines for the selected incremental Encoder */
    ioErrorRetValue = DioCl2EncoderIn_setEncoderLines
      (pRTIEmcEncoder_Unit_1_DioCl_2_Port_1_Ch1, (UInt32) 10000.0);
    if (ioErrorRetValue > IOLIB_NO_ERROR) {
      RTLIB_EXIT(1);
    }

    /* Sets minimum position for the selected incremental Encoder (in lines) */
    ioErrorRetValue = DioCl2EncoderIn_setMinPosition
      (pRTIEmcEncoder_Unit_1_DioCl_2_Port_1_Ch1, (Float64) 0);
    if (ioErrorRetValue > IOLIB_NO_ERROR) {
      RTLIB_EXIT(1);
    }

    /* Sets maximum position for the selected incremental Encoder (in lines) */
    ioErrorRetValue = DioCl2EncoderIn_setMaxPosition
      (pRTIEmcEncoder_Unit_1_DioCl_2_Port_1_Ch1, (Float64) 10000);
    if (ioErrorRetValue > IOLIB_NO_ERROR) {
      RTLIB_EXIT(1);
    }

    /* Sets index signal usage mode for the selected incremental Encoder */
    ioErrorRetValue = DioCl2EncoderIn_setIndexMode
      (pRTIEmcEncoder_Unit_1_DioCl_2_Port_1_Ch1, (Float64)
       DIO_ENC_IMODE_DISABLED);
    if (ioErrorRetValue > IOLIB_NO_ERROR) {
      RTLIB_EXIT(1);
    }

    /* no user specific minimum encoder velocity used */

    /* Sets gated mode for the selected incremental Encoder */
    ioErrorRetValue = DioCl2EncoderIn_setGatedMode
      (pRTIEmcEncoder_Unit_1_DioCl_2_Port_1_Ch1, DIO_ENC_GMODE_DISABLED);
    if (ioErrorRetValue > IOLIB_NO_ERROR) {
      RTLIB_EXIT(1);
    }

    ioErrorRetValue = DioCl2EncoderIn_setMeasurementInterval
      (pRTIEmcEncoder_Unit_1_DioCl_2_Port_1_Ch1, 1);
    if (ioErrorRetValue > IOLIB_NO_ERROR) {
      RTLIB_EXIT(1);
    }

    /* Apply all parameter settings. Finalize the construction of the Encoder Obj driver object */
    ioErrorRetValue = DioCl2EncoderIn_apply
      (pRTIEmcEncoder_Unit_1_DioCl_2_Port_1_Ch1);
    if (ioErrorRetValue > IOLIB_NO_ERROR) {
      RTLIB_EXIT(1);
    }
  }

  /* --- Ball_and_Plate_MicroLabBox_student/Innerloop_Actuator/Motor_B/Outputs to Amplifier/DAC_CLASS1_BL1 --- */
  /* --- [RTI120X, DAC C1] - Channel: 2 --- */
  {
    /* define a variable for IO error handling */
    UInt32 ioErrorCode = IOLIB_NO_ERROR;

    /* Init DAC CL1 AnalogOut driver object pRTIDacC1AnalogOut_Ch_2 */
    ioErrorCode = DacCl1AnalogOut_create(&pRTIDacC1AnalogOut_Ch_2,
      DAC_CLASS1_MASK_CH_2);
    if (ioErrorCode > IOLIB_NO_ERROR) {
      RTLIB_EXIT(1);
    }

    /* Apply all parameter settings. Finalize the construction of the AnalogOut driver object */
    ioErrorCode = DacCl1AnalogOut_apply(pRTIDacC1AnalogOut_Ch_2);
    if (ioErrorCode > IOLIB_NO_ERROR) {
      RTLIB_EXIT(1);
    }
  }

  /* --- Ball_and_Plate_MicroLabBox_student/Innerloop_Actuator/Motor_B/Position Measurement/EMC_ENCODER_BL1 --- */
  /* --- [RTIEMC, Encoder] - DIO class: 2 - Unit: 2 - Port: 1 - Channel: 3 --- */
  {
    /* define a variable for IO error handling */
    UInt32 ioErrorRetValue = IOLIB_NO_ERROR;

    /* Create EMC Encoder driver object pRTIEmcEncoder_Unit_2_DioCl_2_Port_1_Ch3 */
    ioErrorRetValue = DioCl2EncoderIn_create
      (&pRTIEmcEncoder_Unit_2_DioCl_2_Port_1_Ch3, DIO_CLASS2_CHANNEL_3,
       DIO_ENC_INSTANCE_2,DIO_ENC_IUSAGE_DISABLED);
    if (ioErrorRetValue > IOLIB_NO_ERROR) {
      RTLIB_EXIT(1);
    }

    /* -- Position measurement enabled -- */

    /* Sets number of encoder lines for the selected incremental Encoder */
    ioErrorRetValue = DioCl2EncoderIn_setEncoderLines
      (pRTIEmcEncoder_Unit_2_DioCl_2_Port_1_Ch3, (UInt32) 10000.0);
    if (ioErrorRetValue > IOLIB_NO_ERROR) {
      RTLIB_EXIT(1);
    }

    /* Sets minimum position for the selected incremental Encoder (in lines) */
    ioErrorRetValue = DioCl2EncoderIn_setMinPosition
      (pRTIEmcEncoder_Unit_2_DioCl_2_Port_1_Ch3, (Float64) 0);
    if (ioErrorRetValue > IOLIB_NO_ERROR) {
      RTLIB_EXIT(1);
    }

    /* Sets maximum position for the selected incremental Encoder (in lines) */
    ioErrorRetValue = DioCl2EncoderIn_setMaxPosition
      (pRTIEmcEncoder_Unit_2_DioCl_2_Port_1_Ch3, (Float64) 10000);
    if (ioErrorRetValue > IOLIB_NO_ERROR) {
      RTLIB_EXIT(1);
    }

    /* Sets index signal usage mode for the selected incremental Encoder */
    ioErrorRetValue = DioCl2EncoderIn_setIndexMode
      (pRTIEmcEncoder_Unit_2_DioCl_2_Port_1_Ch3, (Float64)
       DIO_ENC_IMODE_DISABLED);
    if (ioErrorRetValue > IOLIB_NO_ERROR) {
      RTLIB_EXIT(1);
    }

    /* no user specific minimum encoder velocity used */

    /* Sets gated mode for the selected incremental Encoder */
    ioErrorRetValue = DioCl2EncoderIn_setGatedMode
      (pRTIEmcEncoder_Unit_2_DioCl_2_Port_1_Ch3, DIO_ENC_GMODE_DISABLED);
    if (ioErrorRetValue > IOLIB_NO_ERROR) {
      RTLIB_EXIT(1);
    }

    ioErrorRetValue = DioCl2EncoderIn_setMeasurementInterval
      (pRTIEmcEncoder_Unit_2_DioCl_2_Port_1_Ch3, 1);
    if (ioErrorRetValue > IOLIB_NO_ERROR) {
      RTLIB_EXIT(1);
    }

    /* Apply all parameter settings. Finalize the construction of the Encoder Obj driver object */
    ioErrorRetValue = DioCl2EncoderIn_apply
      (pRTIEmcEncoder_Unit_2_DioCl_2_Port_1_Ch3);
    if (ioErrorRetValue > IOLIB_NO_ERROR) {
      RTLIB_EXIT(1);
    }
  }

  /* --- Ball_and_Plate_MicroLabBox_student/Innerloop_Actuator/Motor_C/Outputs to Amplifier/DAC_CLASS1_BL1 --- */
  /* --- [RTI120X, DAC C1] - Channel: 3 --- */
  {
    /* define a variable for IO error handling */
    UInt32 ioErrorCode = IOLIB_NO_ERROR;

    /* Init DAC CL1 AnalogOut driver object pRTIDacC1AnalogOut_Ch_3 */
    ioErrorCode = DacCl1AnalogOut_create(&pRTIDacC1AnalogOut_Ch_3,
      DAC_CLASS1_MASK_CH_3);
    if (ioErrorCode > IOLIB_NO_ERROR) {
      RTLIB_EXIT(1);
    }

    /* Apply all parameter settings. Finalize the construction of the AnalogOut driver object */
    ioErrorCode = DacCl1AnalogOut_apply(pRTIDacC1AnalogOut_Ch_3);
    if (ioErrorCode > IOLIB_NO_ERROR) {
      RTLIB_EXIT(1);
    }
  }

  /* --- Ball_and_Plate_MicroLabBox_student/Innerloop_Actuator/Motor_C/Position Measurement/EMC_ENCODER_BL1 --- */
  /* --- [RTIEMC, Encoder] - DIO class: 2 - Unit: 3 - Port: 1 - Channel: 5 --- */
  {
    /* define a variable for IO error handling */
    UInt32 ioErrorRetValue = IOLIB_NO_ERROR;

    /* Create EMC Encoder driver object pRTIEmcEncoder_Unit_3_DioCl_2_Port_1_Ch5 */
    ioErrorRetValue = DioCl2EncoderIn_create
      (&pRTIEmcEncoder_Unit_3_DioCl_2_Port_1_Ch5, DIO_CLASS2_CHANNEL_5,
       DIO_ENC_INSTANCE_3,DIO_ENC_IUSAGE_DISABLED);
    if (ioErrorRetValue > IOLIB_NO_ERROR) {
      RTLIB_EXIT(1);
    }

    /* -- Position measurement enabled -- */

    /* Sets number of encoder lines for the selected incremental Encoder */
    ioErrorRetValue = DioCl2EncoderIn_setEncoderLines
      (pRTIEmcEncoder_Unit_3_DioCl_2_Port_1_Ch5, (UInt32) 10000.0);
    if (ioErrorRetValue > IOLIB_NO_ERROR) {
      RTLIB_EXIT(1);
    }

    /* Sets minimum position for the selected incremental Encoder (in lines) */
    ioErrorRetValue = DioCl2EncoderIn_setMinPosition
      (pRTIEmcEncoder_Unit_3_DioCl_2_Port_1_Ch5, (Float64) 0);
    if (ioErrorRetValue > IOLIB_NO_ERROR) {
      RTLIB_EXIT(1);
    }

    /* Sets maximum position for the selected incremental Encoder (in lines) */
    ioErrorRetValue = DioCl2EncoderIn_setMaxPosition
      (pRTIEmcEncoder_Unit_3_DioCl_2_Port_1_Ch5, (Float64) 10000);
    if (ioErrorRetValue > IOLIB_NO_ERROR) {
      RTLIB_EXIT(1);
    }

    /* Sets index signal usage mode for the selected incremental Encoder */
    ioErrorRetValue = DioCl2EncoderIn_setIndexMode
      (pRTIEmcEncoder_Unit_3_DioCl_2_Port_1_Ch5, (Float64)
       DIO_ENC_IMODE_DISABLED);
    if (ioErrorRetValue > IOLIB_NO_ERROR) {
      RTLIB_EXIT(1);
    }

    /* no user specific minimum encoder velocity used */

    /* Sets gated mode for the selected incremental Encoder */
    ioErrorRetValue = DioCl2EncoderIn_setGatedMode
      (pRTIEmcEncoder_Unit_3_DioCl_2_Port_1_Ch5, DIO_ENC_GMODE_DISABLED);
    if (ioErrorRetValue > IOLIB_NO_ERROR) {
      RTLIB_EXIT(1);
    }

    ioErrorRetValue = DioCl2EncoderIn_setMeasurementInterval
      (pRTIEmcEncoder_Unit_3_DioCl_2_Port_1_Ch5, 1);
    if (ioErrorRetValue > IOLIB_NO_ERROR) {
      RTLIB_EXIT(1);
    }

    /* Apply all parameter settings. Finalize the construction of the Encoder Obj driver object */
    ioErrorRetValue = DioCl2EncoderIn_apply
      (pRTIEmcEncoder_Unit_3_DioCl_2_Port_1_Ch5);
    if (ioErrorRetValue > IOLIB_NO_ERROR) {
      RTLIB_EXIT(1);
    }
  }

  /* dSPACE I/O Board DSETHERNET #1 */
  {
    /* dSPACE I/O Board DSETHERNET #1 Unit:SETUP Group:SETUP */

    /* set IP address, netmask and gateway of the ethernet interface */
    DsIoEth_setIpAddress("192.168.140.8");
    DsIoEth_setNetMask("255.255.255.0");
    DsIoEth_setGatewayAddress("0.0.0.0");
  }

  /* dSPACE I/O Board DSETHERNET #1 Unit:SETUPUDP */
  /* --- Ball_and_Plate_MicroLabBox_student/Ethernet communication/ETHERNET_UDP_SETUP_BL1: ==> Socket ID = (1) --- */
  {
    /* dSPACE I/O Board DSETHERNET #1 Unit:SETUPUDP Group:SETUPUDP */
    DsSSockAddrIn localAddr;
    UInt32 iRet, addrLen= sizeof(localAddr);

    /**/
    /* set first application socket to any IP address */
    localAddr.sin_addr.s_addr = (UInt32) DSIOETH_INADDR_ANY;

    /* socket is bound to IP Address */

    /* set application to local port */
    localAddr.sin_port = DsIoEth_htons((UInt16) 49001U);
    localAddr.sin_family = DSIOETH_AF_INET;

    /* creates a non-blocking UDP connection with specified local address *
     * (containing the local IP and port number) and protocol to use.     */
    iRet =
      DsIoEth_create( DSIOETH_CONNECTION_ID_1,
                     (DsSSockAddr*) &localAddr,
                     addrLen,
                     DSIOETH_PROTO_UDP_SERVER,
                     DSIOETH_FLAG_NONE|DSIOETH_FLAG_BIND_TO_INTERFACE_ADDR
                     ) ;
    if (iRet < 0) {
      msg_error_printf(0,0,
                       "Internal Error: Cannot create UDP socket. (dSPACE Support information: DsIoEth_create(SocketID %d) failed!)",
                       1);
      RTLIB_EXIT(1);
    }
  }
}

/* Function rti_mdl_slave_load() is empty */
#define rti_mdl_slave_load()

/* Function rti_mdl_rtk_initialize() is empty */
#define rti_mdl_rtk_initialize()

static void rti_mdl_initialize_io_units(void)
{
  /* --- Ball_and_Plate_MicroLabBox_student/Innerloop_Actuator/Motor_A/Outputs to Amplifier/DAC_CLASS1_BL1 --- */
  /* --- [RTI120X, DAC C1] - Channel: 1 --- */
  {
    /* define a variable for IO error handling */
    UInt32 ioErrorCode = IOLIB_NO_ERROR;

    /* All channel outputs are released from high impedance state */
    DacCl1AnalogOut_setOutputHighZ(pRTIDacC1AnalogOut_Ch_1,
      DAC_CLASS1_HIGH_Z_OFF);

    /* write initial value of CL1 DAC for output channel 1 */
    ioErrorCode = DacCl1AnalogOut_setOutputValue(pRTIDacC1AnalogOut_Ch_1,
      DAC_CLASS1_CHANNEL_1, (Float64) 0.0);
    if (ioErrorCode > IOLIB_NO_ERROR) {
      RTLIB_EXIT(1);
    }

    /* Activates AnalogOut functionality */
    ioErrorCode = DacCl1AnalogOut_start(pRTIDacC1AnalogOut_Ch_1);
    if (ioErrorCode > IOLIB_NO_ERROR) {
      RTLIB_EXIT(1);
    }
  }

  /* --- Ball_and_Plate_MicroLabBox_student/Innerloop_Actuator/Motor_A/Position Measurement/EMC_ENCODER_BL1 --- */
  /* --- [RTIEMC, Encoder] - DIO class: 2 - Unit: 1 - Port: 1 - Channel: 1 --- */
  {
    /* define a variable for IO error handling */
    UInt32 ioErrorRetValue = IOLIB_NO_ERROR;

    /* Resets start (initial) position for the selected incremental Encoder (in lines) */
    ioErrorRetValue = DioCl2EncoderIn_setEncPosition
      (pRTIEmcEncoder_Unit_1_DioCl_2_Port_1_Ch1, (Float64) 0);
    if (ioErrorRetValue > IOLIB_NO_ERROR) {
      RTLIB_EXIT(1);
    }

    /* Activate Encoder signals read */
    ioErrorRetValue = DioCl2EncoderIn_start
      (pRTIEmcEncoder_Unit_1_DioCl_2_Port_1_Ch1);
    if (ioErrorRetValue > IOLIB_NO_ERROR) {
      RTLIB_EXIT(1);
    }
  }

  /* --- Ball_and_Plate_MicroLabBox_student/Innerloop_Actuator/Motor_B/Outputs to Amplifier/DAC_CLASS1_BL1 --- */
  /* --- [RTI120X, DAC C1] - Channel: 2 --- */
  {
    /* define a variable for IO error handling */
    UInt32 ioErrorCode = IOLIB_NO_ERROR;

    /* All channel outputs are released from high impedance state */
    DacCl1AnalogOut_setOutputHighZ(pRTIDacC1AnalogOut_Ch_2,
      DAC_CLASS1_HIGH_Z_OFF);

    /* write initial value of CL1 DAC for output channel 2 */
    ioErrorCode = DacCl1AnalogOut_setOutputValue(pRTIDacC1AnalogOut_Ch_2,
      DAC_CLASS1_CHANNEL_2, (Float64) 0.0);
    if (ioErrorCode > IOLIB_NO_ERROR) {
      RTLIB_EXIT(1);
    }

    /* Activates AnalogOut functionality */
    ioErrorCode = DacCl1AnalogOut_start(pRTIDacC1AnalogOut_Ch_2);
    if (ioErrorCode > IOLIB_NO_ERROR) {
      RTLIB_EXIT(1);
    }
  }

  /* --- Ball_and_Plate_MicroLabBox_student/Innerloop_Actuator/Motor_B/Position Measurement/EMC_ENCODER_BL1 --- */
  /* --- [RTIEMC, Encoder] - DIO class: 2 - Unit: 2 - Port: 1 - Channel: 3 --- */
  {
    /* define a variable for IO error handling */
    UInt32 ioErrorRetValue = IOLIB_NO_ERROR;

    /* Resets start (initial) position for the selected incremental Encoder (in lines) */
    ioErrorRetValue = DioCl2EncoderIn_setEncPosition
      (pRTIEmcEncoder_Unit_2_DioCl_2_Port_1_Ch3, (Float64) 0);
    if (ioErrorRetValue > IOLIB_NO_ERROR) {
      RTLIB_EXIT(1);
    }

    /* Activate Encoder signals read */
    ioErrorRetValue = DioCl2EncoderIn_start
      (pRTIEmcEncoder_Unit_2_DioCl_2_Port_1_Ch3);
    if (ioErrorRetValue > IOLIB_NO_ERROR) {
      RTLIB_EXIT(1);
    }
  }

  /* --- Ball_and_Plate_MicroLabBox_student/Innerloop_Actuator/Motor_C/Outputs to Amplifier/DAC_CLASS1_BL1 --- */
  /* --- [RTI120X, DAC C1] - Channel: 3 --- */
  {
    /* define a variable for IO error handling */
    UInt32 ioErrorCode = IOLIB_NO_ERROR;

    /* All channel outputs are released from high impedance state */
    DacCl1AnalogOut_setOutputHighZ(pRTIDacC1AnalogOut_Ch_3,
      DAC_CLASS1_HIGH_Z_OFF);

    /* write initial value of CL1 DAC for output channel 3 */
    ioErrorCode = DacCl1AnalogOut_setOutputValue(pRTIDacC1AnalogOut_Ch_3,
      DAC_CLASS1_CHANNEL_3, (Float64) 0.0);
    if (ioErrorCode > IOLIB_NO_ERROR) {
      RTLIB_EXIT(1);
    }

    /* Activates AnalogOut functionality */
    ioErrorCode = DacCl1AnalogOut_start(pRTIDacC1AnalogOut_Ch_3);
    if (ioErrorCode > IOLIB_NO_ERROR) {
      RTLIB_EXIT(1);
    }
  }

  /* --- Ball_and_Plate_MicroLabBox_student/Innerloop_Actuator/Motor_C/Position Measurement/EMC_ENCODER_BL1 --- */
  /* --- [RTIEMC, Encoder] - DIO class: 2 - Unit: 3 - Port: 1 - Channel: 5 --- */
  {
    /* define a variable for IO error handling */
    UInt32 ioErrorRetValue = IOLIB_NO_ERROR;

    /* Resets start (initial) position for the selected incremental Encoder (in lines) */
    ioErrorRetValue = DioCl2EncoderIn_setEncPosition
      (pRTIEmcEncoder_Unit_3_DioCl_2_Port_1_Ch5, (Float64) 0);
    if (ioErrorRetValue > IOLIB_NO_ERROR) {
      RTLIB_EXIT(1);
    }

    /* Activate Encoder signals read */
    ioErrorRetValue = DioCl2EncoderIn_start
      (pRTIEmcEncoder_Unit_3_DioCl_2_Port_1_Ch5);
    if (ioErrorRetValue > IOLIB_NO_ERROR) {
      RTLIB_EXIT(1);
    }
  }

  /* dSPACE I/O Board DSETHERNET #1 Unit:RXUDP */
  {
    /* dSPACE I/O Board DSETHERNET #1 Unit:RXUDP Group:RXUDP */

    /* set output Status initially to state 2 indicating empty RX buffer */
    Ball_and_Plate_MicroLabBox_student_Y.Status = (uint32_T) 4;

    /* set output Received Frames initially to 0 */
    Ball_and_Plate_MicroLabBox_student_Y.NumRXFrames = (real_T) 0;

    /* variable frame size mode is adjusted */

    /* configures the frame size used by the application (frame size setting). *
     * if more data has been received, nothing will be stored and no interrupt *
     * will be triggered then                                                  */

    /* set output Remote Bytes initially to 0 */
    Ball_and_Plate_MicroLabBox_student_B.SFunction1_o4 = (uint32_T) 0;

    {
      Int32 iRet =
        DsIoEth_setRecvFrameSize( DSIOETH_CONNECTION_ID_1,
        32U,
        DSIOETH_RECV_MODE_MAXIMUM_SIZE
        );

      /* error handling */
      if (iRet < 0) {
        msg_error_printf(0,0,
                         "Internal Error: Cannot configure UDP frame size filter. (dSPACE Support information: DsIoEth_setRecvFrameSize(SocketID %d) failed!)",
                         1);
        RTLIB_EXIT(1);
      }
    }

    /* set output Remote IP initially to 0.0.0.0 */
    *((UInt32*) Ball_and_Plate_MicroLabBox_student_B.SFunction1_o5) = (UInt32) 0;

    /* set output Remote Port initially to 0 */
    Ball_and_Plate_MicroLabBox_student_B.SFunction1_o6 = (uint16_T) 0;
  }

  /* dSPACE I/O Board DSETHERNET #1 Unit:SETUPUDP */
  /* --- Ball_and_Plate_MicroLabBox_student/Ethernet communication/ETHERNET_UDP_SETUP_BL1: ==> Socket ID = (1) --- */
  {
    /* dSPACE I/O Board DSETHERNET #1 Unit:SETUPUDP Group:SETUPUDP */
    DsSSockAddrIn remoteAddr ;
    UInt32 iRet, addrLen= sizeof(remoteAddr);
    UInt8 remoteIP[4];
    UInt16 remotePort ;

    /* set remote IP to 0.0.0.0 into the socket structure (to accept any remote IP) */
    remoteIP[0] = 0;
    remoteIP[1] = 0;
    remoteIP[2] = 0;
    remoteIP[3] = 0;

    /* set remote port to 0 into the socket structure (to accept any port) */
    remotePort = 0;

    /**/
    /* set remote IP and port to the socket structure */
    remoteAddr.sin_addr.s_addr = *((UInt32*) remoteIP);
    remoteAddr.sin_port = DsIoEth_htons(remotePort);
    remoteAddr.sin_family = DSIOETH_AF_INET;

    /* configures the socket to the remote station to accept data from.             *
     * If the port number of the socket address. If remote ip address of the socket *
     * address is 0 (INADDR_ANY), then any remote ip address will be accepted.      */
    iRet =
      DsIoEth_setAddrFilter( DSIOETH_CONNECTION_ID_1,
      (DsSSockAddr*) &remoteAddr,
      addrLen
      ) ;

    /* error handling */
    if (iRet < 0) {
      msg_error_printf(0,0,
                       "Internal Error: Cannot configure UDP socket address filter. (dSPACE Support information: DsIoEth_setAddrFilter(SocketID %d) failed!)",
                       1);
      RTLIB_EXIT(1);
    }
  }
}

/* Function rti_mdl_acknowledge_interrupts() is empty */
#define rti_mdl_acknowledge_interrupts()

/* Function rti_mdl_timetables_register() is empty */
#define rti_mdl_timetables_register()

static void rti_mdl_background(void)
{
  /* DsDaq background call */
  DsDaq_Background(0);
}

__INLINE void rti_mdl_sample_input(void)
{
  /* Calls for base sample time: [0.001, 0] */

  /* --- Ball_and_Plate_MicroLabBox_student/Innerloop_Actuator/Motor_A/Position Measurement/EMC_ENCODER_BL1 --- */
  /* --- [RTIEMC, Encoder] - DIO class: 2 - Unit: 1 - Port: 1 - Channel: 1 --- */
  {
    /* define variables required for encoder sensor realtime functions */
    Float64 positionOrAngle= 0.0, speedOrAngVelocity= 0.0;
    UInt32 isIndexRaised= 0;

    /* Reads complete input data from selected encoder input channels (update data from hardware) */
    DioCl2EncoderIn_read(pRTIEmcEncoder_Unit_1_DioCl_2_Port_1_Ch1);

    /* Gets encoder position (line) */
    DioCl2EncoderIn_getEncPosition(pRTIEmcEncoder_Unit_1_DioCl_2_Port_1_Ch1,
      &positionOrAngle);

    /* Gets encoder speed (lines/second) */
    DioCl2EncoderIn_getEncVelocity(pRTIEmcEncoder_Unit_1_DioCl_2_Port_1_Ch1,
      &speedOrAngVelocity);
    Ball_and_Plate_MicroLabBox_student_B.SFunction1_o1_p = (real_T)
      positionOrAngle;
    Ball_and_Plate_MicroLabBox_student_B.SFunction1_o2_d = (real_T)
      speedOrAngVelocity;
  }

  /* --- Ball_and_Plate_MicroLabBox_student/Innerloop_Actuator/Motor_B/Position Measurement/EMC_ENCODER_BL1 --- */
  /* --- [RTIEMC, Encoder] - DIO class: 2 - Unit: 2 - Port: 1 - Channel: 3 --- */
  {
    /* define variables required for encoder sensor realtime functions */
    Float64 positionOrAngle= 0.0, speedOrAngVelocity= 0.0;
    UInt32 isIndexRaised= 0;

    /* Reads complete input data from selected encoder input channels (update data from hardware) */
    DioCl2EncoderIn_read(pRTIEmcEncoder_Unit_2_DioCl_2_Port_1_Ch3);

    /* Gets encoder position (line) */
    DioCl2EncoderIn_getEncPosition(pRTIEmcEncoder_Unit_2_DioCl_2_Port_1_Ch3,
      &positionOrAngle);

    /* Gets encoder speed (lines/second) */
    DioCl2EncoderIn_getEncVelocity(pRTIEmcEncoder_Unit_2_DioCl_2_Port_1_Ch3,
      &speedOrAngVelocity);
    Ball_and_Plate_MicroLabBox_student_B.SFunction1_o1_i = (real_T)
      positionOrAngle;
    Ball_and_Plate_MicroLabBox_student_B.SFunction1_o2_p = (real_T)
      speedOrAngVelocity;
  }

  /* --- Ball_and_Plate_MicroLabBox_student/Innerloop_Actuator/Motor_C/Position Measurement/EMC_ENCODER_BL1 --- */
  /* --- [RTIEMC, Encoder] - DIO class: 2 - Unit: 3 - Port: 1 - Channel: 5 --- */
  {
    /* define variables required for encoder sensor realtime functions */
    Float64 positionOrAngle= 0.0, speedOrAngVelocity= 0.0;
    UInt32 isIndexRaised= 0;

    /* Reads complete input data from selected encoder input channels (update data from hardware) */
    DioCl2EncoderIn_read(pRTIEmcEncoder_Unit_3_DioCl_2_Port_1_Ch5);

    /* Gets encoder position (line) */
    DioCl2EncoderIn_getEncPosition(pRTIEmcEncoder_Unit_3_DioCl_2_Port_1_Ch5,
      &positionOrAngle);

    /* Gets encoder speed (lines/second) */
    DioCl2EncoderIn_getEncVelocity(pRTIEmcEncoder_Unit_3_DioCl_2_Port_1_Ch5,
      &speedOrAngVelocity);
    Ball_and_Plate_MicroLabBox_student_B.SFunction1_o1 = (real_T)
      positionOrAngle;
    Ball_and_Plate_MicroLabBox_student_B.SFunction1_o2 = (real_T)
      speedOrAngVelocity;
  }
}

static void rti_mdl_daq_service()
{
  /* dSPACE Host Service */
  DsDaq_Service(0, 0, 1, (DsDaqSTimestampStruct *)
                rtk_current_task_absolute_time_ptr_get());
}

#undef __INLINE

/****** [EOF] ****************************************************************/
