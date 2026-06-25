/***************************************************************************

   Source file Ball_and_Plate_MicroLabBox_student_trc_ptr.c:

   Definition of function that initializes the global TRC pointers

   RTI1202 7.15 (02-Nov-2020)
   Thu Jun 25 16:30:03 2026

   Copyright 2026, dSPACE GmbH. All rights reserved.

 *****************************************************************************/

/* Include header file. */
#include "Ball_and_Plate_MicroLabBox_student_trc_ptr.h"
#include "Ball_and_Plate_MicroLabBox_student.h"
#include "Ball_and_Plate_MicroLabBox_student_private.h"

/* Compiler options to turn off optimization. */
#if !defined(DS_OPTIMIZE_INIT_TRC_POINTERS)
#ifdef _MCCPPC

#pragma options -nOt -nOr -nOi -nOx

#endif

#ifdef __GNUC__

#pragma GCC optimize ("O0")

#endif

#ifdef _MSC_VER

#pragma optimize ("", off)

#endif
#endif

/* Definition of Global pointers to data type transitions (for TRC-file access) */
volatile real_T *p_0_Ball_and_Plate_MicroLabBox_student_real_T_0 = NULL;
volatile uint32_T *p_0_Ball_and_Plate_MicroLabBox_student_uint32_T_1 = NULL;
volatile real32_T *p_0_Ball_and_Plate_MicroLabBox_student_real32_T_2 = NULL;
volatile uint16_T *p_0_Ball_and_Plate_MicroLabBox_student_uint16_T_3 = NULL;
volatile uint8_T *p_0_Ball_and_Plate_MicroLabBox_student_uint8_T_4 = NULL;
volatile int8_T *p_0_Ball_and_Plate_MicroLabBox_student_int8_T_5 = NULL;
volatile boolean_T *p_0_Ball_and_Plate_MicroLabBox_student_boolean_T_6 = NULL;
volatile real_T *p_0_Ball_and_Plate_MicroLabBox_student_real_T_7 = NULL;
volatile real_T *p_0_Ball_and_Plate_MicroLabBox_student_real_T_8 = NULL;
volatile real_T *p_0_Ball_and_Plate_MicroLabBox_student_real_T_9 = NULL;
volatile real_T *p_0_Ball_and_Plate_MicroLabBox_student_real_T_10 = NULL;
volatile real_T *p_0_Ball_and_Plate_MicroLabBox_student_real_T_11 = NULL;
volatile real_T *p_0_Ball_and_Plate_MicroLabBox_student_real_T_12 = NULL;
volatile real_T *p_0_Ball_and_Plate_MicroLabBox_student_real_T_13 = NULL;
volatile real_T *p_0_Ball_and_Plate_MicroLabBox_student_real_T_14 = NULL;
volatile real_T *p_0_Ball_and_Plate_MicroLabBox_student_real_T_15 = NULL;
volatile real_T *p_0_Ball_and_Plate_MicroLabBox_student_real_T_16 = NULL;
volatile real_T *p_0_Ball_and_Plate_MicroLabBox_student_real_T_17 = NULL;
volatile real_T *p_0_Ball_and_Plate_MicroLabBox_student_real_T_18 = NULL;
volatile real_T *p_0_Ball_and_Plate_MicroLabBox_student_real_T_19 = NULL;
volatile real_T *p_1_Ball_and_Plate_MicroLabBox_student_real_T_0 = NULL;
volatile boolean_T *p_1_Ball_and_Plate_MicroLabBox_student_boolean_T_1 = NULL;
volatile real_T *p_1_Ball_and_Plate_MicroLabBox_student_real_T_2 = NULL;
volatile real32_T *p_1_Ball_and_Plate_MicroLabBox_student_real32_T_3 = NULL;
volatile uint16_T *p_1_Ball_and_Plate_MicroLabBox_student_uint16_T_4 = NULL;
volatile boolean_T *p_1_Ball_and_Plate_MicroLabBox_student_boolean_T_5 = NULL;
volatile real_T *p_1_Ball_and_Plate_MicroLabBox_student_real_T_6 = NULL;
volatile real_T *p_1_Ball_and_Plate_MicroLabBox_student_real_T_7 = NULL;
volatile real_T *p_1_Ball_and_Plate_MicroLabBox_student_real_T_8 = NULL;
volatile real_T *p_1_Ball_and_Plate_MicroLabBox_student_real_T_9 = NULL;
volatile real_T *p_1_Ball_and_Plate_MicroLabBox_student_real_T_10 = NULL;
volatile real_T *p_1_Ball_and_Plate_MicroLabBox_student_real_T_11 = NULL;
volatile real_T *p_2_Ball_and_Plate_MicroLabBox_student_real_T_0 = NULL;
volatile uint32_T *p_2_Ball_and_Plate_MicroLabBox_student_uint32_T_1 = NULL;
volatile int_T *p_2_Ball_and_Plate_MicroLabBox_student_int_T_2 = NULL;
volatile boolean_T *p_2_Ball_and_Plate_MicroLabBox_student_boolean_T_3 = NULL;
volatile int8_T *p_2_Ball_and_Plate_MicroLabBox_student_int8_T_4 = NULL;
volatile boolean_T *p_2_Ball_and_Plate_MicroLabBox_student_boolean_T_5 = NULL;
volatile boolean_T *p_2_Ball_and_Plate_MicroLabBox_student_boolean_T_6 = NULL;
volatile boolean_T *p_2_Ball_and_Plate_MicroLabBox_student_boolean_T_7 = NULL;
volatile real_T *p_2_Ball_and_Plate_MicroLabBox_student_real_T_8 = NULL;
volatile real_T *p_2_Ball_and_Plate_MicroLabBox_student_real_T_9 = NULL;
volatile real_T *p_2_Ball_and_Plate_MicroLabBox_student_real_T_10 = NULL;
volatile boolean_T *p_2_Ball_and_Plate_MicroLabBox_student_boolean_T_11 = NULL;
volatile boolean_T *p_2_Ball_and_Plate_MicroLabBox_student_boolean_T_12 = NULL;
volatile boolean_T *p_2_Ball_and_Plate_MicroLabBox_student_boolean_T_13 = NULL;
volatile real_T *p_2_Ball_and_Plate_MicroLabBox_student_real_T_14 = NULL;
volatile real_T *p_2_Ball_and_Plate_MicroLabBox_student_real_T_15 = NULL;
volatile boolean_T *p_2_Ball_and_Plate_MicroLabBox_student_boolean_T_16 = NULL;
volatile boolean_T *p_2_Ball_and_Plate_MicroLabBox_student_boolean_T_17 = NULL;
volatile boolean_T *p_2_Ball_and_Plate_MicroLabBox_student_boolean_T_18 = NULL;
volatile real_T *p_2_Ball_and_Plate_MicroLabBox_student_real_T_19 = NULL;
volatile real_T *p_2_Ball_and_Plate_MicroLabBox_student_real_T_20 = NULL;
volatile boolean_T *p_2_Ball_and_Plate_MicroLabBox_student_boolean_T_21 = NULL;
volatile real_T *p_3_Ball_and_Plate_MicroLabBox_student_real_T_0 = NULL;
volatile real_T *p_5_Ball_and_Plate_MicroLabBox_student_real_T_0 = NULL;
volatile uint32_T *p_5_Ball_and_Plate_MicroLabBox_student_uint32_T_1 = NULL;
volatile real_T *p_5_Ball_and_Plate_MicroLabBox_student_real_T_2 = NULL;

/*
 *  Declare the functions, that initially assign TRC pointers
 */
static void rti_init_trc_pointers_0(void);

/* Global pointers to data type transitions are separated in different functions to avoid overloading */
static void rti_init_trc_pointers_0(void)
{
  p_0_Ball_and_Plate_MicroLabBox_student_real_T_0 =
    &Ball_and_Plate_MicroLabBox_student_B.RateTransition3;
  p_0_Ball_and_Plate_MicroLabBox_student_uint32_T_1 =
    &Ball_and_Plate_MicroLabBox_student_B.SFunction1_o4;
  p_0_Ball_and_Plate_MicroLabBox_student_real32_T_2 =
    &Ball_and_Plate_MicroLabBox_student_B.ByteUnpacking_o1;
  p_0_Ball_and_Plate_MicroLabBox_student_uint16_T_3 =
    &Ball_and_Plate_MicroLabBox_student_B.SFunction1_o6;
  p_0_Ball_and_Plate_MicroLabBox_student_uint8_T_4 =
    &Ball_and_Plate_MicroLabBox_student_B.SFunction1_o1_c[0];
  p_0_Ball_and_Plate_MicroLabBox_student_int8_T_5 =
    &Ball_and_Plate_MicroLabBox_student_B.ByteUnpacking_o4;
  p_0_Ball_and_Plate_MicroLabBox_student_boolean_T_6 =
    &Ball_and_Plate_MicroLabBox_student_B.DataTypeConversion;
  p_0_Ball_and_Plate_MicroLabBox_student_real_T_7 =
    &Ball_and_Plate_MicroLabBox_student_B.EnabledSubsystem1_b.In1;
  p_0_Ball_and_Plate_MicroLabBox_student_real_T_8 =
    &Ball_and_Plate_MicroLabBox_student_B.EnabledSubsystem_n.OutportBufferForOut1;
  p_0_Ball_and_Plate_MicroLabBox_student_real_T_9 =
    &Ball_and_Plate_MicroLabBox_student_B.sf_MATLABFunction_h.y;
  p_0_Ball_and_Plate_MicroLabBox_student_real_T_10 =
    &Ball_and_Plate_MicroLabBox_student_B.sf_MATLABFunction_g.y;
  p_0_Ball_and_Plate_MicroLabBox_student_real_T_11 =
    &Ball_and_Plate_MicroLabBox_student_B.sf_MATLABFunction1_h.path;
  p_0_Ball_and_Plate_MicroLabBox_student_real_T_12 =
    &Ball_and_Plate_MicroLabBox_student_B.EnabledSubsystem1_f.In1;
  p_0_Ball_and_Plate_MicroLabBox_student_real_T_13 =
    &Ball_and_Plate_MicroLabBox_student_B.EnabledSubsystem_a.OutportBufferForOut1;
  p_0_Ball_and_Plate_MicroLabBox_student_real_T_14 =
    &Ball_and_Plate_MicroLabBox_student_B.sf_MATLABFunction_ko.y;
  p_0_Ball_and_Plate_MicroLabBox_student_real_T_15 =
    &Ball_and_Plate_MicroLabBox_student_B.sf_MATLABFunction2_d.path;
  p_0_Ball_and_Plate_MicroLabBox_student_real_T_16 =
    &Ball_and_Plate_MicroLabBox_student_B.EnabledSubsystem1.In1;
  p_0_Ball_and_Plate_MicroLabBox_student_real_T_17 =
    &Ball_and_Plate_MicroLabBox_student_B.EnabledSubsystem.OutportBufferForOut1;
  p_0_Ball_and_Plate_MicroLabBox_student_real_T_18 =
    &Ball_and_Plate_MicroLabBox_student_B.sf_MATLABFunction.y;
  p_0_Ball_and_Plate_MicroLabBox_student_real_T_19 =
    &Ball_and_Plate_MicroLabBox_student_B.sf_MATLABFunction1.path;
  p_1_Ball_and_Plate_MicroLabBox_student_real_T_0 =
    &Ball_and_Plate_MicroLabBox_student_P.A_robust[0];
  p_1_Ball_and_Plate_MicroLabBox_student_boolean_T_1 =
    &Ball_and_Plate_MicroLabBox_student_P.DetectRisePositive_vinit;
  p_1_Ball_and_Plate_MicroLabBox_student_real_T_2 =
    &Ball_and_Plate_MicroLabBox_student_P.Constant8_Value;
  p_1_Ball_and_Plate_MicroLabBox_student_real32_T_3 =
    &Ball_and_Plate_MicroLabBox_student_P.Gain2_Gain;
  p_1_Ball_and_Plate_MicroLabBox_student_uint16_T_4 =
    &Ball_and_Plate_MicroLabBox_student_P.MatrixDimensionCheck_P1[0];
  p_1_Ball_and_Plate_MicroLabBox_student_boolean_T_5 =
    &Ball_and_Plate_MicroLabBox_student_P.Memory_InitialCondition[0];
  p_1_Ball_and_Plate_MicroLabBox_student_real_T_6 =
    &Ball_and_Plate_MicroLabBox_student_P.EnabledSubsystem1_b.Out1_Y0;
  p_1_Ball_and_Plate_MicroLabBox_student_real_T_7 =
    &Ball_and_Plate_MicroLabBox_student_P.EnabledSubsystem_n.Out1_Y0;
  p_1_Ball_and_Plate_MicroLabBox_student_real_T_8 =
    &Ball_and_Plate_MicroLabBox_student_P.EnabledSubsystem1_f.Out1_Y0;
  p_1_Ball_and_Plate_MicroLabBox_student_real_T_9 =
    &Ball_and_Plate_MicroLabBox_student_P.EnabledSubsystem_a.Out1_Y0;
  p_1_Ball_and_Plate_MicroLabBox_student_real_T_10 =
    &Ball_and_Plate_MicroLabBox_student_P.EnabledSubsystem1.Out1_Y0;
  p_1_Ball_and_Plate_MicroLabBox_student_real_T_11 =
    &Ball_and_Plate_MicroLabBox_student_P.EnabledSubsystem.Out1_Y0;
  p_2_Ball_and_Plate_MicroLabBox_student_real_T_0 =
    &Ball_and_Plate_MicroLabBox_student_DW.UD_DSTATE;
  p_2_Ball_and_Plate_MicroLabBox_student_uint32_T_1 =
    &Ball_and_Plate_MicroLabBox_student_DW.RandSeed;
  p_2_Ball_and_Plate_MicroLabBox_student_int_T_2 =
    &Ball_and_Plate_MicroLabBox_student_DW.ByteUnpacking_IWORK[0];
  p_2_Ball_and_Plate_MicroLabBox_student_boolean_T_3 =
    &Ball_and_Plate_MicroLabBox_student_DW.DelayInput1_DSTATE;
  p_2_Ball_and_Plate_MicroLabBox_student_int8_T_4 =
    &Ball_and_Plate_MicroLabBox_student_DW.RateTransition3_semaphoreTaken;
  p_2_Ball_and_Plate_MicroLabBox_student_boolean_T_5 =
    &Ball_and_Plate_MicroLabBox_student_DW.Memory_PreviousInput[0];
  p_2_Ball_and_Plate_MicroLabBox_student_boolean_T_6 =
    &Ball_and_Plate_MicroLabBox_student_DW.EnabledSubsystem1_b.EnabledSubsystem1_MODE;
  p_2_Ball_and_Plate_MicroLabBox_student_boolean_T_7 =
    &Ball_and_Plate_MicroLabBox_student_DW.EnabledSubsystem_n.EnabledSubsystem_MODE;
  p_2_Ball_and_Plate_MicroLabBox_student_real_T_8 =
    &Ball_and_Plate_MicroLabBox_student_DW.sf_MATLABFunction_h.index;
  p_2_Ball_and_Plate_MicroLabBox_student_real_T_9 =
    &Ball_and_Plate_MicroLabBox_student_DW.sf_MATLABFunction_g.index;
  p_2_Ball_and_Plate_MicroLabBox_student_real_T_10 =
    &Ball_and_Plate_MicroLabBox_student_DW.sf_MATLABFunction1_h.coeffs[0];
  p_2_Ball_and_Plate_MicroLabBox_student_boolean_T_11 =
    &Ball_and_Plate_MicroLabBox_student_DW.sf_MATLABFunction1_h.prev_ref_end_not_empty;
  p_2_Ball_and_Plate_MicroLabBox_student_boolean_T_12 =
    &Ball_and_Plate_MicroLabBox_student_DW.EnabledSubsystem1_f.EnabledSubsystem1_MODE;
  p_2_Ball_and_Plate_MicroLabBox_student_boolean_T_13 =
    &Ball_and_Plate_MicroLabBox_student_DW.EnabledSubsystem_a.EnabledSubsystem_MODE;
  p_2_Ball_and_Plate_MicroLabBox_student_real_T_14 =
    &Ball_and_Plate_MicroLabBox_student_DW.sf_MATLABFunction_ko.index;
  p_2_Ball_and_Plate_MicroLabBox_student_real_T_15 =
    &Ball_and_Plate_MicroLabBox_student_DW.sf_MATLABFunction2_d.coeffs[0];
  p_2_Ball_and_Plate_MicroLabBox_student_boolean_T_16 =
    &Ball_and_Plate_MicroLabBox_student_DW.sf_MATLABFunction2_d.prev_ref_end_not_empty;
  p_2_Ball_and_Plate_MicroLabBox_student_boolean_T_17 =
    &Ball_and_Plate_MicroLabBox_student_DW.EnabledSubsystem1.EnabledSubsystem1_MODE;
  p_2_Ball_and_Plate_MicroLabBox_student_boolean_T_18 =
    &Ball_and_Plate_MicroLabBox_student_DW.EnabledSubsystem.EnabledSubsystem_MODE;
  p_2_Ball_and_Plate_MicroLabBox_student_real_T_19 =
    &Ball_and_Plate_MicroLabBox_student_DW.sf_MATLABFunction.index;
  p_2_Ball_and_Plate_MicroLabBox_student_real_T_20 =
    &Ball_and_Plate_MicroLabBox_student_DW.sf_MATLABFunction1.coeffs[0];
  p_2_Ball_and_Plate_MicroLabBox_student_boolean_T_21 =
    &Ball_and_Plate_MicroLabBox_student_DW.sf_MATLABFunction1.prev_ref_end_not_empty;
  p_3_Ball_and_Plate_MicroLabBox_student_real_T_0 =
    &Ball_and_Plate_MicroLabBox_student_X.TransferFcn_CSTATE;
  p_5_Ball_and_Plate_MicroLabBox_student_real_T_0 =
    &Ball_and_Plate_MicroLabBox_student_Y.NumRXFrames;
  p_5_Ball_and_Plate_MicroLabBox_student_uint32_T_1 =
    &Ball_and_Plate_MicroLabBox_student_Y.Status;
  p_5_Ball_and_Plate_MicroLabBox_student_real_T_2 =
    &Ball_and_Plate_MicroLabBox_student_Y.FrameRate;
}

void Ball_and_Plate_MicroLabBox_student_rti_init_trc_pointers(void)
{
  rti_init_trc_pointers_0();
}
