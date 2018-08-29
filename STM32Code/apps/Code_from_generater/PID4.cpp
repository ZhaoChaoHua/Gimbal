//
// Academic License - for use in teaching, academic research, and meeting
// course requirements at degree granting institutions only.  Not for
// government, commercial, or other organizational use.
//
// File: PID4.cpp
//
// Code generated for Simulink model 'PID4'.
//
// Model version                  : 1.10
// Simulink Coder version         : 8.9 (R2015b) 13-Aug-2015
// C/C++ source code generated on : Mon Jun 26 11:30:55 2017
//
// Target selection: ert.tlc
// Embedded hardware selection: ARM Compatible->ARM Cortex
// Code generation objectives:
//    1. Execution efficiency
//    2. RAM efficiency
// Validation result: Not run
//
#include "PID4.h"

// Private macros used by the generated code to access rtModel
#ifndef rtmIsMajorTimeStep
# define rtmIsMajorTimeStep(rtm)       (((rtm)->Timing.simTimeStep) == MAJOR_TIME_STEP)
#endif

#ifndef rtmIsMinorTimeStep
# define rtmIsMinorTimeStep(rtm)       (((rtm)->Timing.simTimeStep) == MINOR_TIME_STEP)
#endif

#ifndef rtmGetTPtr
# define rtmGetTPtr(rtm)               ((rtm)->Timing.t)
#endif

#ifndef rtmSetTPtr
# define rtmSetTPtr(rtm, val)          ((rtm)->Timing.t = (val))
#endif

// private model entry point functions
extern void PID4_derivatives();

//
// This function updates continuous states using the ODE2 fixed-step
// solver algorithm
//
void PID4ModelClass::rt_ertODEUpdateContinuousStates(RTWSolverInfo *si )
{
  time_T tnew = rtsiGetSolverStopTime(si);
  time_T h = rtsiGetStepSize(si);
  real_T *x = rtsiGetContStates(si);
  ODE2_IntgData *id = (ODE2_IntgData *)rtsiGetSolverData(si);
  real_T *y = id->y;
  real_T *f0 = id->f[0];
  real_T *f1 = id->f[1];
  real_T temp;
  int_T i;
  int_T nXc = 12;
  rtsiSetSimTimeStep(si,MINOR_TIME_STEP);

  // Save the state values at time t in y, we'll use x as ynew.
  (void) memcpy(y, x,
                (uint_T)nXc*sizeof(real_T));

  // Assumes that rtsiSetT and ModelOutputs are up-to-date
  // f0 = f(t,y)
  rtsiSetdX(si, f0);
  PID4_derivatives();

  // f1 = f(t + h, y + h*f0)
  for (i = 0; i < nXc; i++) {
    x[i] = y[i] + (h*f0[i]);
  }

  rtsiSetT(si, tnew);
  rtsiSetdX(si, f1);
  this->step();
  PID4_derivatives();

  // tnew = t + h
  // ynew = y + (h/2)*(f0 + f1)
  temp = 0.5*h;
  for (i = 0; i < nXc; i++) {
    x[i] = y[i] + temp*(f0[i] + f1[i]);
  }

  rtsiSetSimTimeStep(si,MAJOR_TIME_STEP);
}

// Model step function
void PID4ModelClass::step()
{
  real_T u0;
  if (rtmIsMajorTimeStep((&rtM))) {
    // set solver stop time
    rtsiSetSolverStopTime(&(&rtM)->solverInfo,(((&rtM)->Timing.clockTick0+1)*
      (&rtM)->Timing.stepSize0));
  }                                    // end MajorTimeStep

  // Update absolute time of base rate at minor time step
  if (rtmIsMinorTimeStep((&rtM))) {
    (&rtM)->Timing.t[0] = rtsiGetT(&(&rtM)->solverInfo);
  }

  // Outputs for Atomic SubSystem: '<Root>/PID Controller'
  // Gain: '<S7>/Filter Coefficient' incorporates:
  //   Gain: '<S7>/Derivative Gain'
  //   Integrator: '<S7>/Filter'
  //   Sum: '<S7>/SumD'

  rtDW.FilterCoefficient = (0.0 - rtX.Filter_CSTATE) * 20.0;

  // Gain: '<S2>/Filter Coefficient' incorporates:
  //   Gain: '<S2>/Derivative Gain'
  //   Integrator: '<S2>/Filter'
  //   Sum: '<S2>/SumD'

  rtDW.FilterCoefficient_a = (0.0 - rtX.Filter_CSTATE_o) * 20.0;

  // Gain: '<S2>/Integral Gain'
  rtDW.IntegralGain = 0.0;

  // Gain: '<S7>/Integral Gain'
  rtDW.IntegralGain_e = 0.0;

  // Gain: '<S6>/Filter Coefficient' incorporates:
  //   Gain: '<S6>/Derivative Gain'
  //   Integrator: '<S6>/Filter'
  //   Sum: '<S6>/SumD'

  rtDW.FilterCoefficient_m = (0.0 - rtX.Filter_CSTATE_p) * 20.0;

  // Gain: '<S3>/Filter Coefficient' incorporates:
  //   Gain: '<S3>/Derivative Gain'
  //   Integrator: '<S3>/Filter'
  //   Sum: '<S3>/SumD'

  rtDW.FilterCoefficient_k = (0.0 - rtX.Filter_CSTATE_k) * 100.0;

  // Gain: '<S3>/Integral Gain'
  rtDW.IntegralGain_h = 0.0;

  // Gain: '<S6>/Integral Gain'
  rtDW.IntegralGain_b = 0.0;

  // Gain: '<S5>/Filter Coefficient' incorporates:
  //   Gain: '<S5>/Derivative Gain'
  //   Integrator: '<S5>/Filter'
  //   Sum: '<S5>/SumD'

  rtDW.FilterCoefficient_e = (0.0 - rtX.Filter_CSTATE_kn) * 20.0;

  // Gain: '<S4>/Filter Coefficient' incorporates:
  //   Gain: '<S4>/Derivative Gain'
  //   Integrator: '<S4>/Filter'
  //   Sum: '<S4>/SumD'

  rtDW.FilterCoefficient_o = (0.0 - rtX.Filter_CSTATE_a) * 100.0;

  // Gain: '<S4>/Integral Gain'
  rtDW.IntegralGain_p = 0.0;

  // Gain: '<S5>/Integral Gain'
  rtDW.IntegralGain_c = 0.0;

  // Sum: '<S2>/Sum' incorporates:
  //   Gain: '<S2>/Proportional Gain'
  //   Inport: '<Root>/yaw_vel_fb'
  //   Integrator: '<S2>/Integrator'
  //   Integrator: '<S7>/Integrator'
  //   Sum: '<S1>/Sum6'
  //   Sum: '<S7>/Sum'

  u0 = (((rtX.Integrator_CSTATE + rtDW.FilterCoefficient) - rtU.yaw_vel_fb) *
        1500.0 + rtX.Integrator_CSTATE_l) + rtDW.FilterCoefficient_a;

  // Saturate: '<S2>/Saturate'
  if (u0 > 1000.0) {
    // Outport: '<Root>/yaw_out'
    rtY.yaw_out = 1000.0;
  } else if (u0 < -1000.0) {
    // Outport: '<Root>/yaw_out'
    rtY.yaw_out = -1000.0;
  } else {
    // Outport: '<Root>/yaw_out'
    rtY.yaw_out = u0;
  }

  // End of Saturate: '<S2>/Saturate'

  // Sum: '<S3>/Sum' incorporates:
  //   Gain: '<S3>/Proportional Gain'
  //   Inport: '<Root>/roll_vel_fb'
  //   Integrator: '<S3>/Integrator'
  //   Integrator: '<S6>/Integrator'
  //   Sum: '<S1>/Sum7'
  //   Sum: '<S6>/Sum'

  u0 = (((rtX.Integrator_CSTATE_l1 + rtDW.FilterCoefficient_m) - rtU.roll_vel_fb)
        * 1000.0 + rtX.Integrator_CSTATE_i) + rtDW.FilterCoefficient_k;

  // Saturate: '<S3>/Saturate'
  if (u0 > 1000.0) {
    // Outport: '<Root>/roll_out'
    rtY.roll_out = 1000.0;
  } else if (u0 < -1000.0) {
    // Outport: '<Root>/roll_out'
    rtY.roll_out = -1000.0;
  } else {
    // Outport: '<Root>/roll_out'
    rtY.roll_out = u0;
  }

  // End of Saturate: '<S3>/Saturate'

  // Sum: '<S4>/Sum' incorporates:
  //   Gain: '<S4>/Proportional Gain'
  //   Inport: '<Root>/pitch_vel_fb'
  //   Integrator: '<S4>/Integrator'
  //   Integrator: '<S5>/Integrator'
  //   Sum: '<S1>/Sum8'
  //   Sum: '<S5>/Sum'

  u0 = (((rtX.Integrator_CSTATE_a + rtDW.FilterCoefficient_e) - rtU.pitch_vel_fb)
        * 1000.0 + rtX.Integrator_CSTATE_id) + rtDW.FilterCoefficient_o;

  // Saturate: '<S4>/Saturate'
  if (u0 > 1000.0) {
    // Outport: '<Root>/pitch_out'
    rtY.pitch_out = 1000.0;
  } else if (u0 < -1000.0) {
    // Outport: '<Root>/pitch_out'
    rtY.pitch_out = -1000.0;
  } else {
    // Outport: '<Root>/pitch_out'
    rtY.pitch_out = u0;
  }

  // End of Saturate: '<S4>/Saturate'
  // End of Outputs for SubSystem: '<Root>/PID Controller'
  if (rtmIsMajorTimeStep((&rtM))) {
    rt_ertODEUpdateContinuousStates(&(&rtM)->solverInfo);

    // Update absolute time for base rate
    // The "clockTick0" counts the number of times the code of this task has
    //  been executed. The absolute time is the multiplication of "clockTick0"
    //  and "Timing.stepSize0". Size of "clockTick0" ensures timer will not
    //  overflow during the application lifespan selected.

    ++(&rtM)->Timing.clockTick0;
    (&rtM)->Timing.t[0] = rtsiGetSolverStopTime(&(&rtM)->solverInfo);

    {
      // Update absolute timer for sample time: [0.001s, 0.0s]
      // The "clockTick1" counts the number of times the code of this task has
      //  been executed. The resolution of this integer timer is 0.001, which is the step size
      //  of the task. Size of "clockTick1" ensures timer will not overflow during the
      //  application lifespan selected.

      (&rtM)->Timing.clockTick1++;
    }
  }                                    // end MajorTimeStep
}

// Derivatives for root system: '<Root>'
void PID4ModelClass::PID4_derivatives()
{
  XDot *_rtXdot;
  _rtXdot = ((XDot *) (&rtM)->ModelData.derivs);

  // Derivatives for Atomic SubSystem: '<Root>/PID Controller'
  // Derivatives for Integrator: '<S7>/Integrator'
  _rtXdot->Integrator_CSTATE = rtDW.IntegralGain_e;

  // Derivatives for Integrator: '<S7>/Filter'
  _rtXdot->Filter_CSTATE = rtDW.FilterCoefficient;

  // Derivatives for Integrator: '<S2>/Filter'
  _rtXdot->Filter_CSTATE_o = rtDW.FilterCoefficient_a;

  // Derivatives for Integrator: '<S2>/Integrator'
  _rtXdot->Integrator_CSTATE_l = rtDW.IntegralGain;

  // Derivatives for Integrator: '<S6>/Integrator'
  _rtXdot->Integrator_CSTATE_l1 = rtDW.IntegralGain_b;

  // Derivatives for Integrator: '<S6>/Filter'
  _rtXdot->Filter_CSTATE_p = rtDW.FilterCoefficient_m;

  // Derivatives for Integrator: '<S3>/Filter'
  _rtXdot->Filter_CSTATE_k = rtDW.FilterCoefficient_k;

  // Derivatives for Integrator: '<S3>/Integrator'
  _rtXdot->Integrator_CSTATE_i = rtDW.IntegralGain_h;

  // Derivatives for Integrator: '<S5>/Integrator'
  _rtXdot->Integrator_CSTATE_a = rtDW.IntegralGain_c;

  // Derivatives for Integrator: '<S5>/Filter'
  _rtXdot->Filter_CSTATE_kn = rtDW.FilterCoefficient_e;

  // Derivatives for Integrator: '<S4>/Filter'
  _rtXdot->Filter_CSTATE_a = rtDW.FilterCoefficient_o;

  // Derivatives for Integrator: '<S4>/Integrator'
  _rtXdot->Integrator_CSTATE_id = rtDW.IntegralGain_p;

  // End of Derivatives for SubSystem: '<Root>/PID Controller'
}

// Model initialize function
void PID4ModelClass::initialize()
{
  // Registration code
  {
    // Setup solver object
    rtsiSetSimTimeStepPtr(&(&rtM)->solverInfo, &(&rtM)->Timing.simTimeStep);
    rtsiSetTPtr(&(&rtM)->solverInfo, &rtmGetTPtr((&rtM)));
    rtsiSetStepSizePtr(&(&rtM)->solverInfo, &(&rtM)->Timing.stepSize0);
    rtsiSetdXPtr(&(&rtM)->solverInfo, &(&rtM)->ModelData.derivs);
    rtsiSetContStatesPtr(&(&rtM)->solverInfo, (real_T **) &(&rtM)
                         ->ModelData.contStates);
    rtsiSetNumContStatesPtr(&(&rtM)->solverInfo, &(&rtM)->Sizes.numContStates);
    rtsiSetNumPeriodicContStatesPtr(&(&rtM)->solverInfo, &(&rtM)
      ->Sizes.numPeriodicContStates);
    rtsiSetPeriodicContStateIndicesPtr(&(&rtM)->solverInfo, &(&rtM)
      ->ModelData.periodicContStateIndices);
    rtsiSetPeriodicContStateRangesPtr(&(&rtM)->solverInfo, &(&rtM)
      ->ModelData.periodicContStateRanges);
    rtsiSetErrorStatusPtr(&(&rtM)->solverInfo, (&rtmGetErrorStatus((&rtM))));
    rtsiSetRTModelPtr(&(&rtM)->solverInfo, (&rtM));
  }

  rtsiSetSimTimeStep(&(&rtM)->solverInfo, MAJOR_TIME_STEP);
  (&rtM)->ModelData.intgData.y = (&rtM)->ModelData.odeY;
  (&rtM)->ModelData.intgData.f[0] = (&rtM)->ModelData.odeF[0];
  (&rtM)->ModelData.intgData.f[1] = (&rtM)->ModelData.odeF[1];
  (&rtM)->ModelData.contStates = ((X *) &rtX);
  rtsiSetSolverData(&(&rtM)->solverInfo, (void *)&(&rtM)->ModelData.intgData);
  rtsiSetSolverName(&(&rtM)->solverInfo,"ode2");
  rtmSetTPtr((&rtM), &(&rtM)->Timing.tArray[0]);
  (&rtM)->Timing.stepSize0 = 0.001;

  // InitializeConditions for Atomic SubSystem: '<Root>/PID Controller'
  // InitializeConditions for Integrator: '<S7>/Integrator'
  rtX.Integrator_CSTATE = 0.0;

  // InitializeConditions for Integrator: '<S7>/Filter'
  rtX.Filter_CSTATE = 0.0;

  // InitializeConditions for Integrator: '<S2>/Filter'
  rtX.Filter_CSTATE_o = 0.0;

  // InitializeConditions for Integrator: '<S2>/Integrator'
  rtX.Integrator_CSTATE_l = 0.0;

  // InitializeConditions for Integrator: '<S6>/Integrator'
  rtX.Integrator_CSTATE_l1 = 0.0;

  // InitializeConditions for Integrator: '<S6>/Filter'
  rtX.Filter_CSTATE_p = 0.0;

  // InitializeConditions for Integrator: '<S3>/Filter'
  rtX.Filter_CSTATE_k = 0.0;

  // InitializeConditions for Integrator: '<S3>/Integrator'
  rtX.Integrator_CSTATE_i = 0.0;

  // InitializeConditions for Integrator: '<S5>/Integrator'
  rtX.Integrator_CSTATE_a = 0.0;

  // InitializeConditions for Integrator: '<S5>/Filter'
  rtX.Filter_CSTATE_kn = 0.0;

  // InitializeConditions for Integrator: '<S4>/Filter'
  rtX.Filter_CSTATE_a = 0.0;

  // InitializeConditions for Integrator: '<S4>/Integrator'
  rtX.Integrator_CSTATE_id = 0.0;

  // End of InitializeConditions for SubSystem: '<Root>/PID Controller'
}

// Constructor
PID4ModelClass::PID4ModelClass()
{
}

// Destructor
PID4ModelClass::~PID4ModelClass()
{
  // Currently there is no destructor body generated.
}

// Real-Time Model get method
RT_MODEL * PID4ModelClass::getRTM()
{
  return (&rtM);
}

//
// File trailer for generated code.
//
// [EOF]
//
