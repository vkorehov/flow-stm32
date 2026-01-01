#include <stdio.h>

#include "flow.h"
#include "limit.h"

static uint8_t flow_idx = 0;
static uint32_t flow_buffer[2] = {0};
static uint16_t flow_rate = 0;
static uint16_t max_flow_rate = 0;

void FLOW_DumpState(char* buff, int len) {
    snprintf(buff, len,
        "FLOW flow_idx=%u flow_buffer[0]=%u flow_buffer[1]=%u flow_rate=%d max_flow_rate=%d",
        flow_idx,
        flow_buffer[0],
        flow_buffer[1],
        flow_rate,
        max_flow_rate
    );
}

void FLOW_Tick(void) {
    // Store current TIM2 count
    flow_buffer[flow_idx] = TIM2->CNT;
    // Compute flow when buffer wraps
    if (flow_idx == 1) {
      uint32_t previous = flow_buffer[0];
      uint32_t current = flow_buffer[1];
      if (current >= previous) {
        flow_rate = current - previous;
      } else {
        flow_rate = (0xFFFFFFFF - previous) + current + 1; // Handle wrap
      } 
      if (flow_rate > max_flow_rate) {
        max_flow_rate = flow_rate;
      }
    } // Increment circular buffer index
    flow_idx = (flow_idx + 1) % 2;
}

uint16_t FLOW_Rate(void) {
  return flow_rate;
}

/**
 * Compute required hot water flow rate (integer math)
 *
 * @param cold_flow_rate  Cold water flow rate (e.g., in 0.01 L/min units)
 * @param target_temp     Desired mixed temperature (°C 100)
 * @return                Hot water flow rate in same units as cold_flow_rate
 */
uint16_t FLOW_GetHotFlow(uint16_t cold_flow_rate, uint16_t target_temp, uint16_t hot_temp)
{
    const uint16_t cold_temp = 14*16;  // 14°C constant is good enough!
    if (target_temp <= cold_temp) return 0;        // all cold
    if (target_temp >= hot_temp)  return MAX_FLOW_RATE;  // error, cannot reach

    // Use uint32_t to avoid overflow
    uint32_t numerator   = (uint32_t)cold_flow_rate * (target_temp - cold_temp);
    uint32_t denominator = hot_temp - target_temp;

    return (uint16_t)(numerator / denominator);
}

#ifdef SIMULATED_FLOW
// Lookup table for gate valve flow characteristic
static const uint8_t valve_position[] = {0, 10, 20, 30, 40, 50, 60, 70, 80, 90, 100};
static const uint8_t valve_flow[]     = {0,  1,  5, 15, 30, 50, 68, 82, 92, 97, 100};
static const uint8_t TABLE_SIZE = sizeof(valve_position) / sizeof(valve_position[0]);

static uint32_t gate_valve_flow_percent(int32_t position_percent)
{
  if (position_percent <= valve_position[0])
    return valve_flow[0];
  if (position_percent >= valve_position[TABLE_SIZE - 1])
    return valve_flow[TABLE_SIZE - 1];
  
  // Find interval
  for (uint8_t i = 0; i < TABLE_SIZE - 1; i++) {
    if (position_percent >= valve_position[i] && position_percent <= valve_position[i+1]) {
      uint32_t x0 = valve_position[i];
      uint32_t x1 = valve_position[i+1];
      uint32_t y0 = valve_flow[i];
      uint32_t y1 = valve_flow[i+1];
      
      // Linear interpolation
      return y0 + (y1 - y0) * (position_percent - x0) / (x1 - x0);
    }
  }
  return 0; // should never reach here
}

static uint32_t last_cold_valve_pos = 0;
static uint16_t simulated_flow_rate = 0;
void FLOW_SimulateTick(void) {
  simulated_flow_rate = gate_valve_flow_percent(100*last_cold_valve_pos/MAX_POS) * 2;    
  last_cold_valve_pos = LIMITS_GetPos(&cold_valve_limits_state);
}
uint16_t FLOW_SimulatedRate(void) {
  return simulated_flow_rate;
}
#endif


