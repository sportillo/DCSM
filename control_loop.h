/*
 * File: control_loop.h
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

#ifndef RTW_HEADER_control_loop_h_
#define RTW_HEADER_control_loop_h_
#ifndef control_loop_COMMON_INCLUDES_
# define control_loop_COMMON_INCLUDES_
#include <stddef.h>
#include <math.h>
#include <string.h>
#include "rtwtypes.h"
#endif                                 /* control_loop_COMMON_INCLUDES_ */

#include "control_loop_types.h"

/* Macros for accessing real-time model data structure */
#ifndef rtmGetErrorStatus
# define rtmGetErrorStatus(rtm)        ((rtm)->errorStatus)
#endif

#ifndef rtmSetErrorStatus
# define rtmSetErrorStatus(rtm, val)   ((rtm)->errorStatus = (val))
#endif

/* Block states (auto storage) for system '<Root>' */
typedef struct {
  real_T Delay_DSTATE;                 /* '<S1>/Delay' */
  real_T Integrator_DSTATE;            /* '<S9>/Integrator' */
  real_T Delay1_DSTATE;                /* '<S1>/Delay1' */
  real_T Integrator_DSTATE_d;          /* '<S10>/Integrator' */
  real_T V_k_PreviousInput;            /* '<S11>/V_k' */
  real_T V_k_PreviousInput_p;          /* '<S12>/V_k' */
} DW_control_loop_T;

/* External inputs (root inport signals with auto storage) */
typedef struct {
  real_T Vd;                           /* '<Root>/Vd' */
  real_T Rd;                           /* '<Root>/Rd' */
  real_T T_edge_l;                     /* '<Root>/T_edge_l' */
  real_T T_edge_r;                     /* '<Root>/T_edge_r' */
  real_T Moving_l;                     /* '<Root>/Moving_l' */
  real_T Moving_r;                     /* '<Root>/Moving_r' */
} ExtU_control_loop_T;

/* External outputs (root outports fed by signals with auto storage) */
typedef struct {
  real_T PWM_l;                        /* '<Root>/PWM_l' */
  real_T PWM_r;                        /* '<Root>/PWM_r' */
  real_T Dir_l;                        /* '<Root>/Dir_l' */
  real_T Dir_r;                        /* '<Root>/Dir_r' */
  real_T V_l;                          /* '<Root>/V_l' */
  real_T V_r;                          /* '<Root>/V_r' */
} ExtY_control_loop_T;

/* Parameters (auto storage) */
struct P_control_loop_T_ {
  real_T to_sec_Gain;                  /* Expression: 10^-6
                                        * Referenced by: '<S2>/to_sec'
                                        */
  real_T mstep_Value;                  /* Expression: 0.0017
                                        * Referenced by: '<S2>/m//step'
                                        */
  real_T Delay_InitialCondition;       /* Expression: 0
                                        * Referenced by: '<S1>/Delay'
                                        */
  real_T a_Gain;                       /* Expression: 0.65
                                        * Referenced by: '<S11>/1-a'
                                        */
  real_T V_k_X0;                       /* Expression: 0
                                        * Referenced by: '<S11>/V_k'
                                        */
  real_T a_Gain_p;                     /* Expression: 0.35
                                        * Referenced by: '<S11>/a'
                                        */
  real_T Ld2_Value;                    /* Expression: 0.115
                                        * Referenced by: '<S4>/Ld//2'
                                        */
  real_T ProportionalGain_Gain;        /* Expression: P
                                        * Referenced by: '<S9>/Proportional Gain'
                                        */
  real_T IntegralGain_Gain;            /* Expression: I
                                        * Referenced by: '<S9>/Integral Gain'
                                        */
  real_T Integrator_gainval;           /* Computed Parameter: Integrator_gainval
                                        * Referenced by: '<S9>/Integrator'
                                        */
  real_T Integrator_IC;                /* Expression: InitialConditionForIntegrator
                                        * Referenced by: '<S9>/Integrator'
                                        */
  real_T Saturation_UpperSat;          /* Expression: UpperSaturationLimit
                                        * Referenced by: '<S9>/Saturation'
                                        */
  real_T Saturation_LowerSat;          /* Expression: LowerSaturationLimit
                                        * Referenced by: '<S9>/Saturation'
                                        */
  real_T Gain_Gain;                    /* Expression: 100
                                        * Referenced by: '<S7>/Gain'
                                        */
  real_T Delay1_InitialCondition;      /* Expression: 0
                                        * Referenced by: '<S1>/Delay1'
                                        */
  real_T to_sec_Gain_e;                /* Expression: 10^-6
                                        * Referenced by: '<S3>/to_sec'
                                        */
  real_T mstep_Value_h;                /* Expression: 0.0017
                                        * Referenced by: '<S3>/m//step'
                                        */
  real_T a_Gain_h;                     /* Expression: 0.65
                                        * Referenced by: '<S12>/1-a'
                                        */
  real_T V_k_X0_p;                     /* Expression: 0
                                        * Referenced by: '<S12>/V_k'
                                        */
  real_T a_Gain_j;                     /* Expression: 0.35
                                        * Referenced by: '<S12>/a'
                                        */
  real_T ProportionalGain_Gain_i;      /* Expression: P
                                        * Referenced by: '<S10>/Proportional Gain'
                                        */
  real_T IntegralGain_Gain_j;          /* Expression: I
                                        * Referenced by: '<S10>/Integral Gain'
                                        */
  real_T Integrator_gainval_j;         /* Computed Parameter: Integrator_gainval_j
                                        * Referenced by: '<S10>/Integrator'
                                        */
  real_T Integrator_IC_m;              /* Expression: InitialConditionForIntegrator
                                        * Referenced by: '<S10>/Integrator'
                                        */
  real_T Saturation_UpperSat_k;        /* Expression: UpperSaturationLimit
                                        * Referenced by: '<S10>/Saturation'
                                        */
  real_T Saturation_LowerSat_o;        /* Expression: LowerSaturationLimit
                                        * Referenced by: '<S10>/Saturation'
                                        */
  real_T Gain_Gain_g;                  /* Expression: 100
                                        * Referenced by: '<S8>/Gain'
                                        */
  uint32_T Delay_DelayLength;          /* Computed Parameter: Delay_DelayLength
                                        * Referenced by: '<S1>/Delay'
                                        */
  uint32_T Delay1_DelayLength;         /* Computed Parameter: Delay1_DelayLength
                                        * Referenced by: '<S1>/Delay1'
                                        */
};

/* Real-time Model Data Structure */
struct tag_RTM_control_loop_T {
  const char_T * volatile errorStatus;
};

/* Block parameters (auto storage) */
extern P_control_loop_T control_loop_P;

/* Block states (auto storage) */
extern DW_control_loop_T control_loop_DW;

/* External inputs (root inport signals with auto storage) */
extern ExtU_control_loop_T control_loop_U;

/* External outputs (root outports fed by signals with auto storage) */
extern ExtY_control_loop_T control_loop_Y;

/* Model entry point functions */
extern void control_loop_initialize(void);
extern void control_loop_step(void);
extern void control_loop_terminate(void);

/* Real-time Model object */
extern RT_MODEL_control_loop_T *const control_loop_M;

/*-
 * The generated code includes comments that allow you to trace directly
 * back to the appropriate location in the model.  The basic format
 * is <system>/block_name, where system is the system number (uniquely
 * assigned by Simulink) and block_name is the name of the block.
 *
 * Note that this particular code originates from a subsystem build,
 * and has its own system numbers different from the parent model.
 * Refer to the system hierarchy for this subsystem below, and use the
 * MATLAB hilite_system command to trace the generated code back
 * to the parent model.  For example,
 *
 * hilite_system('control_loop_100Hz_mdl/control_loop ')    - opens subsystem control_loop_100Hz_mdl/control_loop
 * hilite_system('control_loop_100Hz_mdl/control_loop /Kp') - opens and selects block Kp
 *
 * Here is the system hierarchy for this model
 *
 * '<Root>' : 'control_loop_100Hz_mdl'
 * '<S1>'   : 'control_loop_100Hz_mdl/control_loop '
 * '<S2>'   : 'control_loop_100Hz_mdl/control_loop /Compute velocity Left'
 * '<S3>'   : 'control_loop_100Hz_mdl/control_loop /Compute velocity Right'
 * '<S4>'   : 'control_loop_100Hz_mdl/control_loop /Dinamica uniciclo'
 * '<S5>'   : 'control_loop_100Hz_mdl/control_loop /Direction'
 * '<S6>'   : 'control_loop_100Hz_mdl/control_loop /Direction1'
 * '<S7>'   : 'control_loop_100Hz_mdl/control_loop /Normalize Left'
 * '<S8>'   : 'control_loop_100Hz_mdl/control_loop /Normalize Right'
 * '<S9>'   : 'control_loop_100Hz_mdl/control_loop /PID Left'
 * '<S10>'  : 'control_loop_100Hz_mdl/control_loop /PID Right'
 * '<S11>'  : 'control_loop_100Hz_mdl/control_loop /Velocity Estimator Left'
 * '<S12>'  : 'control_loop_100Hz_mdl/control_loop /Velocity Estimator Right'
 */
#endif                                 /* RTW_HEADER_control_loop_h_ */

/*
 * File trailer for generated code.
 *
 * [EOF]
 */
