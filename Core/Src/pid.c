#include <stdio.h>
#include <string.h>
#include <stdlib.h>

#include "pid.h"
#include "limit.h"
#include "step.h"
#include "flow.h"
#include "serial.h"

//#define PID_DEBUG
#ifdef PID_DEBUG
static char dbg[256];
#define PID_DEBUG_PRINT(fmt, ...) do { \
    snprintf(dbg, sizeof(dbg), fmt, ##__VA_ARGS__); \
    SERIAL_TransmitNow((uint8_t*)dbg, strlen(dbg)); \
} while(0)
#else
#define PID_DEBUG_PRINT(fmt, ...) ((void)0)
#endif

PidState cold_valve_pid = {
  .setpoint = 0,
  .flow = 0,  
  .other_flow = 0, // flow thru other valve, needs to be substracted for PID to work correctly  
  .kp_gain = 1000,
  .kd_gain = 500,
  .ki_gain = 0,
  .integral = 0,
  .prev_error = 0,
  .pid_stable_ticks = 0,
  .pid_unstable_ticks = 0,
  .valve_limits_state = &cold_valve_limits_state,
  .valve_state = &cold_valve,
  .enabled = 0
};

PidState hot_valve_pid = {
  .setpoint = 0,
  .flow = 0,
  .other_flow = 0, // flow thru other valve, needs to be substracted for PID to work correctly  
  .kp_gain = 4000,
  .kd_gain = 1000,
  .ki_gain = 0,
  .integral = 0,
  .prev_error = 0,
  .pid_stable_ticks = 0,
  .pid_unstable_ticks = 0,
  .valve_limits_state = &hot_valve_limits_state,
  .valve_state = &hot_valve,
  .enabled = 0
};

void PID_DumpState(PidState* s, char* buff, int len) {
    snprintf(buff, len,
        "PID setpoint=%u flow=%u other_flow=%u kp_gain=%u kd_gain=%u ki_gain=%u integral=%u "
        "prev_error=%u pid_stable_ticks=%u pid_unstable_ticks=%u enabled=%u",
    s->setpoint,
    s->flow,
    s->other_flow,
    s->kp_gain,
    s->kd_gain,
    s->ki_gain,
    s->integral,
    s->prev_error,
    s->pid_stable_ticks,
    s->pid_unstable_ticks,
    s->enabled
    );
}

void PID_Setpoint(PidState* s, int16_t setpoint, int16_t other_flow) {
  s->setpoint = setpoint;
  s->err = 0;
  s->prev_error = 0;
  s->integral = 0;
  s->pid_stable_ticks = 0;
  s->pid_unstable_ticks = 0;
  s->other_flow = other_flow;
}

void PID_Disable(PidState* s, uint8_t err) {
  s->enabled = 0;
  s->err = err;
}

void PID_Enable(PidState* s) {
  s->enabled = 1;
}

uint8_t PID_Stable(PidState* s) {
  if (s->err != 0) {
    return s->err;
  }
  if (s->pid_stable_ticks > 0) {
    return 0;
  }
  return 0xff;
}

static int32_t pid_compute(PidState* s)
{
    int32_t measured = 0;
    if (s->flow > s->other_flow) {
      measured = s->flow - s->other_flow;
    }
    int32_t error = s->setpoint - measured;

    // --- Proportional term ---
    int32_t P = (s->kp_gain * error) / GAIN_SCALE;

    // --- Derivative term ---
    int32_t D = 0;
    if (LIMITS_GetPos(s->valve_limits_state) > PID_NO_FLOW_POS) {
        D = (s->kd_gain * (error - s->prev_error)) / GAIN_SCALE;
    }

    s->prev_error = error;

    // --- Integral term with simple integer decay ---
    if (error != 0) {
        s->integral += (s->ki_gain* error) / GAIN_SCALE;
    } else if (s->integral > 0) {
        s->integral -= 1;  // decay
    } else if (s->integral < 0) {
        s->integral += 1;  // decay
    }
    // Clamp integral to prevent runaway
    if (s->integral > PID_I_CLAMP) s->integral = PID_I_CLAMP;
    if (s->integral < -PID_I_CLAMP) s->integral = -PID_I_CLAMP;

    // --- Compute final output ---
    int32_t output = P + s->integral + D;

    if (output > PID_MAX_OUTPUT) output = PID_MAX_OUTPUT;
    if (output < PID_MIN_OUTPUT) output = PID_MIN_OUTPUT;

    PID_DEBUG_PRINT("PID meas:%u, tgt:%u, err:%ld, P:%ld, I:%ld, D:%ld, O:%ld, pos:%ld",
                    measured, s->setpoint,
                    error, P, s->integral, D, output, LIMITS_GetPos(s->valve_limits_state));
    return output;
}

// 0.3s
void PID_MeasureTick(void) {
    // Update measurements on both valves
    FLOW_Tick(); // 0.3s    
#ifdef SIMULATED_FLOW
    FLOW_SimulateTick(); // 0.3s
    hot_valve_pid.flow = \
      cold_valve_pid.flow = \
        FLOW_SimulatedRate();
#else
    cold_valve_pid.flow = \
      hot_valve_pid.flow = \
        FLOW_Rate();
#endif
}

// --- PID tick ---
void PID_Tick(PidState* s)
{
    if (!STEPPER_Homed(s->valve_state)) return;
    if (LIMITS_Stuck(s->valve_limits_state)) return;
    if (!s->enabled) return;
    // --- Compute PID ---
    int32_t delta_steps = pid_compute(s);

    // --- Stability / timeout check ---
    if (delta_steps == 0) {
        s->pid_stable_ticks++;
        if (s->pid_stable_ticks >= PID_STABLE_TICKS) {
            s->pid_unstable_ticks = 0;
        }
    } else {
        s->pid_stable_ticks = 0;
        s->pid_unstable_ticks++;
        if (s->pid_unstable_ticks > PID_TIMEOUT_TICKS) {
            PID_DEBUG_PRINT("PID: TIMEOUT delta=%ld flow=%d other_flow=%d", delta_steps, s->flow, s->other_flow);
            PID_Disable(s, 1);
        }
    }

    // --- Clamp to valve position ---
    if (delta_steps > 0 && LIMITS_GetPos(s->valve_limits_state) + delta_steps > MAX_POS) {
        delta_steps = MAX_POS - LIMITS_GetPos(s->valve_limits_state);
    }
    if (delta_steps < 0 && LIMITS_GetPos(s->valve_limits_state) + delta_steps < 0) {
        delta_steps = -LIMITS_GetPos(s->valve_limits_state);
    }

    // --- Apply movement ---
    if (delta_steps != 0) {
        if (delta_steps > 0) {
            STEPPER_UpdateMovement(s->valve_state, GPIO_PIN_SET, delta_steps);
        } else {
            STEPPER_UpdateMovement(s->valve_state, GPIO_PIN_RESET, -delta_steps);
        }
    }
}
