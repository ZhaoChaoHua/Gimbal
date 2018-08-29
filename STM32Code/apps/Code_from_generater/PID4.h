//
// Academic License - for use in teaching, academic research, and meeting
// course requirements at degree granting institutions only.  Not for
// government, commercial, or other organizational use.
//
// File: PID4.h
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
#ifndef RTW_HEADER_PID4_h_
#define RTW_HEADER_PID4_h_
#include <string.h>
#ifndef PID4_COMMON_INCLUDES_
# define PID4_COMMON_INCLUDES_
#include "rtwtypes.h"
#include "rtw_continuous.h"
#include "rtw_solver.h"
#endif                                 // PID4_COMMON_INCLUDES_

// Macros for accessing real-time model data structure
#ifndef rtmGetErrorStatus
# define rtmGetErrorStatus(rtm)        ((rtm)->errorStatus)
#endif

#ifndef rtmSetErrorStatus
# define rtmSetErrorStatus(rtm, val)   ((rtm)->errorStatus = (val))
#endif

#ifndef rtmGetStopRequested
# define rtmGetStopRequested(rtm)      ((rtm)->Timing.stopRequestedFlag)
#endif

#ifndef rtmSetStopRequested
# define rtmSetStopRequested(rtm, val) ((rtm)->Timing.stopRequestedFlag = (val))
#endif

#ifndef rtmGetStopRequestedPtr
# define rtmGetStopRequestedPtr(rtm)   (&((rtm)->Timing.stopRequestedFlag))
#endif

#ifndef rtmGetT
# define rtmGetT(rtm)                  (rtmGetTPtr((rtm))[0])
#endif

// Forward declaration for rtModel
typedef struct tag_RTM RT_MODEL;

// Block signals and states (auto storage) for system '<Root>'
typedef struct {
  real_T FilterCoefficient;            // '<S7>/Filter Coefficient'
  real_T FilterCoefficient_a;          // '<S2>/Filter Coefficient'
  real_T IntegralGain;                 // '<S2>/Integral Gain'
  real_T IntegralGain_e;               // '<S7>/Integral Gain'
  real_T FilterCoefficient_m;          // '<S6>/Filter Coefficient'
  real_T FilterCoefficient_k;          // '<S3>/Filter Coefficient'
  real_T IntegralGain_h;               // '<S3>/Integral Gain'
  real_T IntegralGain_b;               // '<S6>/Integral Gain'
  real_T FilterCoefficient_e;          // '<S5>/Filter Coefficient'
  real_T FilterCoefficient_o;          // '<S4>/Filter Coefficient'
  real_T IntegralGain_p;               // '<S4>/Integral Gain'
  real_T IntegralGain_c;               // '<S5>/Integral Gain'
} DW;

// Continuous states (auto storage)
typedef struct {
  real_T Integrator_CSTATE;            // '<S7>/Integrator'
  real_T Filter_CSTATE;                // '<S7>/Filter'
  real_T Filter_CSTATE_o;              // '<S2>/Filter'
  real_T Integrator_CSTATE_l;          // '<S2>/Integrator'
  real_T Integrator_CSTATE_l1;         // '<S6>/Integrator'
  real_T Filter_CSTATE_p;              // '<S6>/Filter'
  real_T Filter_CSTATE_k;              // '<S3>/Filter'
  real_T Integrator_CSTATE_i;          // '<S3>/Integrator'
  real_T Integrator_CSTATE_a;          // '<S5>/Integrator'
  real_T Filter_CSTATE_kn;             // '<S5>/Filter'
  real_T Filter_CSTATE_a;              // '<S4>/Filter'
  real_T Integrator_CSTATE_id;         // '<S4>/Integrator'
} X;

// State derivatives (auto storage)
typedef struct {
  real_T Integrator_CSTATE;            // '<S7>/Integrator'
  real_T Filter_CSTATE;                // '<S7>/Filter'
  real_T Filter_CSTATE_o;              // '<S2>/Filter'
  real_T Integrator_CSTATE_l;          // '<S2>/Integrator'
  real_T Integrator_CSTATE_l1;         // '<S6>/Integrator'
  real_T Filter_CSTATE_p;              // '<S6>/Filter'
  real_T Filter_CSTATE_k;              // '<S3>/Filter'
  real_T Integrator_CSTATE_i;          // '<S3>/Integrator'
  real_T Integrator_CSTATE_a;          // '<S5>/Integrator'
  real_T Filter_CSTATE_kn;             // '<S5>/Filter'
  real_T Filter_CSTATE_a;              // '<S4>/Filter'
  real_T Integrator_CSTATE_id;         // '<S4>/Integrator'
} XDot;

// State disabled
typedef struct {
  boolean_T Integrator_CSTATE;         // '<S7>/Integrator'
  boolean_T Filter_CSTATE;             // '<S7>/Filter'
  boolean_T Filter_CSTATE_o;           // '<S2>/Filter'
  boolean_T Integrator_CSTATE_l;       // '<S2>/Integrator'
  boolean_T Integrator_CSTATE_l1;      // '<S6>/Integrator'
  boolean_T Filter_CSTATE_p;           // '<S6>/Filter'
  boolean_T Filter_CSTATE_k;           // '<S3>/Filter'
  boolean_T Integrator_CSTATE_i;       // '<S3>/Integrator'
  boolean_T Integrator_CSTATE_a;       // '<S5>/Integrator'
  boolean_T Filter_CSTATE_kn;          // '<S5>/Filter'
  boolean_T Filter_CSTATE_a;           // '<S4>/Filter'
  boolean_T Integrator_CSTATE_id;      // '<S4>/Integrator'
} XDis;

#ifndef ODE2_INTG
#define ODE2_INTG

// ODE2 Integration Data
typedef struct {
  real_T *y;                           // output
  real_T *f[2];                        // derivatives
} ODE2_IntgData;

#endif

// External inputs (root inport signals with auto storage)
typedef struct {
  real_T yaw_in;                       // '<Root>/yaw_in'
  real_T roll_in;                      // '<Root>/roll_in'
  real_T pitch_in;                     // '<Root>/pitch_in'
  real_T yaw_vel_fb;                   // '<Root>/yaw_vel_fb'
  real_T roll_vel_fb;                  // '<Root>/roll_vel_fb'
  real_T pitch_vel_fb;                 // '<Root>/pitch_vel_fb'
  real_T yaw_fb;                       // '<Root>/yaw_fb'
  real_T roll_fb;                      // '<Root>/roll_fb'
  real_T pitch_fb;                     // '<Root>/pitch_fb'
} ExtU;

// External outputs (root outports fed by signals with auto storage)
typedef struct {
  real_T yaw_out;                      // '<Root>/yaw_out'
  real_T roll_out;                     // '<Root>/roll_out'
  real_T pitch_out;                    // '<Root>/pitch_out'
} ExtY;

// Real-time Model Data Structure
struct tag_RTM {
  const char_T *errorStatus;
  RTWSolverInfo solverInfo;

  //
  //  ModelData:
  //  The following substructure contains information regarding
  //  the data used in the model.

  struct {
    X *contStates;
    int_T *periodicContStateIndices;
    real_T *periodicContStateRanges;
    real_T *derivs;
    boolean_T *contStateDisabled;
    boolean_T zCCacheNeedsReset;
    boolean_T derivCacheNeedsReset;
    boolean_T blkStateChange;
    real_T odeY[12];
    real_T odeF[2][12];
    ODE2_IntgData intgData;
  } ModelData;

  //
  //  Sizes:
  //  The following substructure contains sizes information
  //  for many of the model attributes such as inputs, outputs,
  //  dwork, sample times, etc.

  struct {
    int_T numContStates;
    int_T numPeriodicContStates;
    int_T numSampTimes;
  } Sizes;

  //
  //  Timing:
  //  The following substructure contains information regarding
  //  the timing information for the model.

  struct {
    uint32_T clockTick0;
    time_T stepSize0;
    uint32_T clockTick1;
    SimTimeStep simTimeStep;
    boolean_T stopRequestedFlag;
    time_T *t;
    time_T tArray[2];
  } Timing;
};

#ifdef __cplusplus

extern "C" {

#endif

#ifdef __cplusplus

}
#endif

// Class declaration for model PID4
class PID4ModelClass {
  // public data and function members
 public:
  // External inputs
  ExtU rtU;

  // External outputs
  ExtY rtY;

  // Model entry point functions

  // model initialize function
  void initialize();

  // model step function
  void step();

  // Constructor
  PID4ModelClass();

  // Destructor
  ~PID4ModelClass();

  // Real-Time Model get method
  RT_MODEL * getRTM();

  // private data and function members
 private:
  // Block signals and states
  DW rtDW;
  X rtX;                               // Block continuous states

  // Real-Time Model
  RT_MODEL rtM;

  // Continuous states update member function
  void rt_ertODEUpdateContinuousStates(RTWSolverInfo *si );

  // Derivatives member function
  void PID4_derivatives();
};

//-
//  The generated code includes comments that allow you to trace directly
//  back to the appropriate location in the model.  The basic format
//  is <system>/block_name, where system is the system number (uniquely
//  assigned by Simulink) and block_name is the name of the block.
//
//  Note that this particular code originates from a subsystem build,
//  and has its own system numbers different from the parent model.
//  Refer to the system hierarchy for this subsystem below, and use the
//  MATLAB hilite_system command to trace the generated code back
//  to the parent model.  For example,
//
//  hilite_system('test_gimbal_T1/PID Controller')    - opens subsystem test_gimbal_T1/PID Controller
//  hilite_system('test_gimbal_T1/PID Controller/Kp') - opens and selects block Kp
//
//  Here is the system hierarchy for this model
//
//  '<Root>' : 'test_gimbal_T1'
//  '<S1>'   : 'test_gimbal_T1/PID Controller'
//  '<S2>'   : 'test_gimbal_T1/PID Controller/PID Controller'
//  '<S3>'   : 'test_gimbal_T1/PID Controller/PID Controller1'
//  '<S4>'   : 'test_gimbal_T1/PID Controller/PID Controller2'
//  '<S5>'   : 'test_gimbal_T1/PID Controller/PID Controller3'
//  '<S6>'   : 'test_gimbal_T1/PID Controller/PID Controller4'
//  '<S7>'   : 'test_gimbal_T1/PID Controller/PID Controller5'

#endif                                 // RTW_HEADER_PID4_h_

//
// File trailer for generated code.
//
// [EOF]
//
