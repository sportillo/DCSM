/*
 * File: control_loop_data.c
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

/* Block parameters (auto storage) */
P_control_loop_T control_loop_P = {
  1.0E-6,                              /* Expression: 10^-6
                                        * Referenced by: '<S2>/to_sec'
                                        */
  0.0017,                              /* Expression: 0.0017
                                        * Referenced by: '<S2>/m//step'
                                        */
  0.0,                                 /* Expression: 0
                                        * Referenced by: '<S1>/Delay'
                                        */
  0.65,                                /* Expression: 0.65
                                        * Referenced by: '<S11>/1-a'
                                        */
  0.0,                                 /* Expression: 0
                                        * Referenced by: '<S11>/V_k'
                                        */
  0.35,                                /* Expression: 0.35
                                        * Referenced by: '<S11>/a'
                                        */
  0.115,                               /* Expression: 0.115
                                        * Referenced by: '<S4>/Ld//2'
                                        */
  0.441212917967112,                   /* Expression: P
                                        * Referenced by: '<S9>/Proportional Gain'
                                        */
  8.33976301057019,                    /* Expression: I
                                        * Referenced by: '<S9>/Integral Gain'
                                        */
  0.005,                               /* Computed Parameter: Integrator_gainval
                                        * Referenced by: '<S9>/Integrator'
                                        */
  0.0,                                 /* Expression: InitialConditionForIntegrator
                                        * Referenced by: '<S9>/Integrator'
                                        */
  1.0,                                 /* Expression: UpperSaturationLimit
                                        * Referenced by: '<S9>/Saturation'
                                        */
  -1.0,                                /* Expression: LowerSaturationLimit
                                        * Referenced by: '<S9>/Saturation'
                                        */
  100.0,                               /* Expression: 100
                                        * Referenced by: '<S7>/Gain'
                                        */
  0.0,                                 /* Expression: 0
                                        * Referenced by: '<S1>/Delay1'
                                        */
  1.0E-6,                              /* Expression: 10^-6
                                        * Referenced by: '<S3>/to_sec'
                                        */
  0.0017,                              /* Expression: 0.0017
                                        * Referenced by: '<S3>/m//step'
                                        */
  0.65,                                /* Expression: 0.65
                                        * Referenced by: '<S12>/1-a'
                                        */
  0.0,                                 /* Expression: 0
                                        * Referenced by: '<S12>/V_k'
                                        */
  0.35,                                /* Expression: 0.35
                                        * Referenced by: '<S12>/a'
                                        */
  0.441212917967112,                   /* Expression: P
                                        * Referenced by: '<S10>/Proportional Gain'
                                        */
  8.33976301057019,                    /* Expression: I
                                        * Referenced by: '<S10>/Integral Gain'
                                        */
  0.005,                               /* Computed Parameter: Integrator_gainval_j
                                        * Referenced by: '<S10>/Integrator'
                                        */
  0.0,                                 /* Expression: InitialConditionForIntegrator
                                        * Referenced by: '<S10>/Integrator'
                                        */
  1.0,                                 /* Expression: UpperSaturationLimit
                                        * Referenced by: '<S10>/Saturation'
                                        */
  -1.0,                                /* Expression: LowerSaturationLimit
                                        * Referenced by: '<S10>/Saturation'
                                        */
  100.0,                               /* Expression: 100
                                        * Referenced by: '<S8>/Gain'
                                        */
  1U,                                  /* Computed Parameter: Delay_DelayLength
                                        * Referenced by: '<S1>/Delay'
                                        */
  1U                                   /* Computed Parameter: Delay1_DelayLength
                                        * Referenced by: '<S1>/Delay1'
                                        */
};

/*
 * File trailer for generated code.
 *
 * [EOF]
 */
