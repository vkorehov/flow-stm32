#ifndef __PID_H__
#define __PID_H__

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "limit.h"
#include "step.h"
  
// PID constants
#define MAX_FLOW         200      // edges/sec
#define PID_MAX_OUTPUT   1500
#define PID_MIN_OUTPUT  -1500
#define PID_NO_FLOW_POS  1500
#define PID_I_CLAMP      5

#define GAIN_SCALE 100

// Stability / timeout
#define PID_STABLE_TICKS   10
#define PID_TIMEOUT_TICKS  100
  
typedef struct {
  int16_t setpoint;
  int16_t flow;
  int16_t other_flow;  
  int32_t kp_gain;
  int32_t kd_gain;
  int32_t ki_gain;
  int32_t integral;
  int32_t prev_error;
  uint16_t pid_stable_ticks;
  uint16_t pid_unstable_ticks;
  uint8_t err;
  ValveLimitsState *valve_limits_state;
  Stepper_t *valve_state;
  uint8_t enabled;
} PidState;

extern PidState cold_valve_pid;
extern PidState hot_valve_pid;

void PID_DumpState(PidState* s, char* buff, int len);
void PID_Tick(PidState* s);
void PID_MeasureTick(void);
void PID_Disable(PidState* s, uint8_t err);
uint8_t PID_Stable(PidState* s);
void PID_Enable(PidState* s);
void PID_Setpoint(PidState* s, int16_t setpoint, int16_t other_flow);
#ifdef __cplusplus
}
#endif

#endif /* __PID_H__ */

