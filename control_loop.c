/*
 * File: control_loop.c
 *
 * Code generated for Simulink model 'control_loop'.
 *
 * Model version                  : 1.38
 * Simulink Coder version         : 8.4 (R2013a) 13-Feb-2013
 * TLC version                    : 8.4 (Jan 18 2013)
 * C/C++ source code generated on : Wed Sep 17 16:40:03 2014
 *
 * Target selection: ert.tlc
 * Embedded hardware selection: ARM Compatible->ARM 7
 * Code generation objectives: Unspecified
 * Validation result: Not run
 */

#include "control_loop.h"
#include "control_loop_private.h"

/* Block states (auto storage) */
DW_control_loop_T control_loop_DW;

/* External inputs (root inport signals with auto storage) */
ExtU_control_loop_T control_loop_U;

/* External outputs (root outports fed by signals with auto storage) */
ExtY_control_loop_T control_loop_Y;

/* Real-time model */
RT_MODEL_control_loop_T control_loop_M_;
RT_MODEL_control_loop_T *const control_loop_M = &control_loop_M_;

/* Model step function */
void control_loop_step(void)
{
  /* local block i/o variables */
  real_T rtb_IntegralGain;
  real_T rtb_IntegralGain_b;
  real_T rtb_Saturation;
  real_T rtb_Product1;
  real_T rtb_Sum3;
  real_T rtb_Sum3_o;
  real_T Integrator;
  real_T Integrator_b;

  /* Sum: '<S11>/Sum3' incorporates:
   *  Constant: '<S2>/m//step'
   *  Delay: '<S1>/Delay'
   *  Gain: '<S11>/1-a'
   *  Gain: '<S11>/a'
   *  Gain: '<S2>/to_sec'
   *  Inport: '<Root>/Moving_l'
   *  Inport: '<Root>/T_edge_l'
   *  Memory: '<S11>/V_k'
   *  Product: '<S1>/Vl_new'
   *  Product: '<S2>/V_abs'
   */
  rtb_Sum3 = 1.0 / (control_loop_P.to_sec_Gain * control_loop_U.T_edge_l) *
    control_loop_P.mstep_Value * control_loop_U.Moving_l *
    control_loop_DW.Delay_DSTATE * control_loop_P.a_Gain +
    control_loop_P.a_Gain_p * control_loop_DW.V_k_PreviousInput;

  /* Product: '<S1>/Divide' incorporates:
   *  Inport: '<Root>/Rd'
   *  Inport: '<Root>/Vd'
   */
  rtb_Saturation = control_loop_U.Vd / control_loop_U.Rd;

  /* Product: '<S4>/Product1' incorporates:
   *  Constant: '<S4>/Ld//2'
   */
  rtb_Product1 = control_loop_P.Ld2_Value * rtb_Saturation;

  /* Sum: '<S1>/Sum1' incorporates:
   *  Constant: '<S4>/Ld//2'
   *  Inport: '<Root>/Vd'
   *  Product: '<S4>/Product'
   *  Sum: '<S4>/Sum2'
   */
  rtb_Saturation = (control_loop_U.Vd - rtb_Saturation *
                    control_loop_P.Ld2_Value) - rtb_Sum3;

  /* Gain: '<S9>/Integral Gain' */
  rtb_IntegralGain = control_loop_P.IntegralGain_Gain * rtb_Saturation;

  /* DiscreteIntegrator: '<S9>/Integrator' */
  Integrator = control_loop_P.Integrator_gainval * rtb_IntegralGain +
    control_loop_DW.Integrator_DSTATE;

  /* Sum: '<S9>/Sum' incorporates:
   *  Gain: '<S9>/Proportional Gain'
   */
  rtb_Saturation = control_loop_P.ProportionalGain_Gain * rtb_Saturation +
    Integrator;

  /* Saturate: '<S9>/Saturation' */
  if (rtb_Saturation >= control_loop_P.Saturation_UpperSat) {
    rtb_Saturation = control_loop_P.Saturation_UpperSat;
  } else {
    if (rtb_Saturation <= control_loop_P.Saturation_LowerSat) {
      rtb_Saturation = control_loop_P.Saturation_LowerSat;
    }
  }

  /* End of Saturate: '<S9>/Saturation' */

  /* Sum: '<S12>/Sum3' incorporates:
   *  Constant: '<S3>/m//step'
   *  Delay: '<S1>/Delay1'
   *  Gain: '<S12>/1-a'
   *  Gain: '<S12>/a'
   *  Gain: '<S3>/to_sec'
   *  Inport: '<Root>/Moving_r'
   *  Inport: '<Root>/T_edge_r'
   *  Memory: '<S12>/V_k'
   *  Product: '<S1>/Vr_new'
   *  Product: '<S3>/V_abs'
   */
  rtb_Sum3_o = 1.0 / (control_loop_P.to_sec_Gain_e * control_loop_U.T_edge_r) *
    control_loop_P.mstep_Value_h * control_loop_DW.Delay1_DSTATE *
    control_loop_U.Moving_r * control_loop_P.a_Gain_h + control_loop_P.a_Gain_j *
    control_loop_DW.V_k_PreviousInput_p;

  /* Sum: '<S1>/Sum2' incorporates:
   *  Inport: '<Root>/Vd'
   *  Sum: '<S4>/Sum3'
   */
  rtb_Product1 = (rtb_Product1 + control_loop_U.Vd) - rtb_Sum3_o;

  /* Outport: '<Root>/PWM_l' incorporates:
   *  Abs: '<S7>/Abs'
   *  Gain: '<S7>/Gain'
   */
  control_loop_Y.PWM_l = control_loop_P.Gain_Gain * fabs(rtb_Saturation);

  /* Gain: '<S10>/Integral Gain' */
  rtb_IntegralGain_b = control_loop_P.IntegralGain_Gain_j * rtb_Product1;

  /* DiscreteIntegrator: '<S10>/Integrator' */
  Integrator_b = control_loop_P.Integrator_gainval_j * rtb_IntegralGain_b +
    control_loop_DW.Integrator_DSTATE_d;

  /* Sum: '<S10>/Sum' incorporates:
   *  Gain: '<S10>/Proportional Gain'
   */
  rtb_Product1 = control_loop_P.ProportionalGain_Gain_i * rtb_Product1 +
    Integrator_b;

  /* Saturate: '<S10>/Saturation' */
  if (rtb_Product1 >= control_loop_P.Saturation_UpperSat_k) {
    rtb_Product1 = control_loop_P.Saturation_UpperSat_k;
  } else {
    if (rtb_Product1 <= control_loop_P.Saturation_LowerSat_o) {
      rtb_Product1 = control_loop_P.Saturation_LowerSat_o;
    }
  }

  /* End of Saturate: '<S10>/Saturation' */

  /* Outport: '<Root>/PWM_r' incorporates:
   *  Abs: '<S8>/Abs'
   *  Gain: '<S8>/Gain'
   */
  control_loop_Y.PWM_r = control_loop_P.Gain_Gain_g * fabs(rtb_Product1);

  /* Signum: '<S5>/Sign' */
  if (rtb_Saturation < 0.0) {
    rtb_Saturation = -1.0;
  } else {
    if (rtb_Saturation > 0.0) {
      rtb_Saturation = 1.0;
    }
  }

  /* End of Signum: '<S5>/Sign' */

  /* Outport: '<Root>/Dir_l' */
  control_loop_Y.Dir_l = rtb_Saturation;

  /* Signum: '<S6>/Sign' */
  if (rtb_Product1 < 0.0) {
    rtb_Product1 = -1.0;
  } else {
    if (rtb_Product1 > 0.0) {
      rtb_Product1 = 1.0;
    }
  }

  /* End of Signum: '<S6>/Sign' */

  /* Outport: '<Root>/Dir_r' */
  control_loop_Y.Dir_r = rtb_Product1;

  /* Outport: '<Root>/V_l' */
  control_loop_Y.V_l = rtb_Sum3;

  /* Outport: '<Root>/V_r' */
  control_loop_Y.V_r = rtb_Sum3_o;

  /* Update for Delay: '<S1>/Delay' */
  control_loop_DW.Delay_DSTATE = rtb_Saturation;

  /* Update for Memory: '<S11>/V_k' */
  control_loop_DW.V_k_PreviousInput = rtb_Sum3;

  /* Update for DiscreteIntegrator: '<S9>/Integrator' */
  control_loop_DW.Integrator_DSTATE = control_loop_P.Integrator_gainval *
    rtb_IntegralGain + Integrator;

  /* Update for Delay: '<S1>/Delay1' */
  control_loop_DW.Delay1_DSTATE = rtb_Product1;

  /* Update for Memory: '<S12>/V_k' */
  control_loop_DW.V_k_PreviousInput_p = rtb_Sum3_o;

  /* Update for DiscreteIntegrator: '<S10>/Integrator' */
  control_loop_DW.Integrator_DSTATE_d = control_loop_P.Integrator_gainval_j *
    rtb_IntegralGain_b + Integrator_b;
}

/* Model initialize function */
void control_loop_initialize(void)
{
  /* Registration code */

  /* initialize error status */
  rtmSetErrorStatus(control_loop_M, (NULL));

  /* states (dwork) */
  (void) memset((void *)&control_loop_DW, 0,
                sizeof(DW_control_loop_T));

  /* external inputs */
  (void) memset((void *)&control_loop_U, 0,
                sizeof(ExtU_control_loop_T));

  /* external outputs */
  (void) memset((void *)&control_loop_Y, 0,
                sizeof(ExtY_control_loop_T));

  /* InitializeConditions for Delay: '<S1>/Delay' */
  control_loop_DW.Delay_DSTATE = control_loop_P.Delay_InitialCondition;

  /* InitializeConditions for Memory: '<S11>/V_k' */
  control_loop_DW.V_k_PreviousInput = control_loop_P.V_k_X0;

  /* InitializeConditions for DiscreteIntegrator: '<S9>/Integrator' */
  control_loop_DW.Integrator_DSTATE = control_loop_P.Integrator_IC;

  /* InitializeConditions for Delay: '<S1>/Delay1' */
  control_loop_DW.Delay1_DSTATE = control_loop_P.Delay1_InitialCondition;

  /* InitializeConditions for Memory: '<S12>/V_k' */
  control_loop_DW.V_k_PreviousInput_p = control_loop_P.V_k_X0_p;

  /* InitializeConditions for DiscreteIntegrator: '<S10>/Integrator' */
  control_loop_DW.Integrator_DSTATE_d = control_loop_P.Integrator_IC_m;
}

/* Model terminate function */
void control_loop_terminate(void)
{
  /* (no terminate code required) */
}

/*
 * File trailer for generated code.
 *
 * [EOF]
 */
