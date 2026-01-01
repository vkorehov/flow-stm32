#include <stdio.h>
#include <string.h>
#include "step.h"
#include "limit.h"
#include "tim.h"
#include "serial.h"

#define USE_ACCEL

//#define STEP_DEBUG
#ifdef STEP_DEBUG
static char dbg[256];
#define STEP_DEBUG_PRINT(fmt, ...) do { \
    snprintf(dbg, sizeof(dbg), fmt, ##__VA_ARGS__); \
    SERIAL_TransmitNow((uint8_t*)dbg, strlen(dbg)); \
} while(0)
#else
#define STEP_DEBUG_PRINT(fmt, ...) ((void)0)
#endif

#define DIR_CHANGE_DELAY_US     50000     // 50 ms
#define BACKGROUND_INTERVAL_US  5000     // background FSM check every 5 ms

extern TIM_HandleTypeDef htim6;
extern TIM_HandleTypeDef htim16;
extern TIM_HandleTypeDef htim17;

#define AUTO_DISABLE_TIMEOUT_US 1000000
// --- FSM step1_state values ---
#define STEP_STATE_IDLE             0   // background check
#define STEP_STATE_PULSE_LOW        1
#define STEP_STATE_PULSE_HIGH       2
#define STEP_STATE_INTER_DELAY      3
#define STEP_STATE_DIR_DELAY        4

#define HOME_STATE_UNHOMED        0
#define HOME_STATE_HOMING         1
#define HOME_STATE_HOMING_BACKOFF 2
#define HOME_STATE_HOMED          3

#define HOMING_STEPS          20000
#define HOMING_BACKOFF_STEPS  2000

#define HOMING_DIR            GPIO_PIN_RESET
#define HOMING_BACKOFF_DIR    GPIO_PIN_SET

// --- Motion state definitions ---
#define STATE_IDLE       0
#define STATE_RAMP_UP    1
#define STATE_RUN        2
#define STATE_RAMP_DOWN  3

//#define MIN_DELAY 800
//#define MAX_DELAY 2400
#define MIN_DELAY 1200
#define MAX_DELAY 2400

// --- Motion parameters ---
const uint16_t backlashSteps = 320; // 88
const uint16_t accelSteps = 500;
const uint32_t f_min = 1000000UL / MAX_DELAY;
const uint32_t f_max = 1000000UL / MIN_DELAY;

Stepper_t cold_valve = {
  .direction = GPIO_PIN_UNKNOWN,
  .ramp_index = 1,
  .steps_remaining = 0,
  .state = STATE_IDLE,
  .next_direction = GPIO_PIN_UNKNOWN,
  .next_steps_remaining = 0,
  .version = 0,
  .next_version = 0,
  .homing_state = 0,
  .step_state = STEP_STATE_IDLE,
  .step_next_delay = 0,
  .step_next_dir = 0,
  .step_backlash_remaining = 0,
  .motor_idle_ticks = 0,
  .enabled = 0,
  .htim = &htim17,
  .pin_step = GPIO_PIN_0,
  .port_step = GPIOB,
  .pin_dir = GPIO_PIN_2,
  .port_dir = GPIOB,
  .pin_en = GPIO_PIN_1,
  .port_en = GPIOB,
  .valve_limits_state = &cold_valve_limits_state,
};

Stepper_t hot_valve = {
  .direction = GPIO_PIN_UNKNOWN,
  .ramp_index = 1,
  .steps_remaining = 0,
  .state = STATE_IDLE,
  .next_direction = GPIO_PIN_UNKNOWN,
  .next_steps_remaining = 0,
  .version = 0,
  .next_version = 0,
  .homing_state = 0,
  .step_state = STEP_STATE_IDLE,
  .step_next_delay = 0,
  .step_next_dir = 0,
  .step_backlash_remaining = 0,
  .motor_idle_ticks = 0,
  .enabled = 0,
  .htim = &htim16,
  .pin_step = GPIO_PIN_4,
  .port_step = GPIOB,
  .pin_dir = GPIO_PIN_6,
  .port_dir = GPIOB,
  .pin_en = GPIO_PIN_5,
  .port_en = GPIOB,
  .valve_limits_state = &hot_valve_limits_state,
};

void STEPPER_DumpState(Stepper_t* s, char* buff, int len) {
    snprintf(buff, len,
        "STEP direction=%u ramp_index=%u steps_remaining=%u state=%u next_direction=%u "
        "next_steps_remaining=%u version=%u next_version=%u homing_state=%u step_state=%u"
        "step_next_delay=%u step_next_dir=%u step_backlash_remaining=%u pin_step=%u pin_dir=%u pin_en=%u enabled=%u",
        s->direction,
        s->ramp_index,
        s->steps_remaining,
        s->state,
        s->next_direction,
        s->next_steps_remaining,
        s->version,
        s->next_version,
        s->homing_state,
        s->step_state,
        s->step_next_delay,
        s->step_next_dir,
        s->step_backlash_remaining,
        s->pin_step,
        s->pin_dir,
        s->pin_en,
        s->enabled
    );
}

static inline void reloadTimer(TIM_HandleTypeDef *htim, uint16_t period)
{
    __HAL_TIM_SET_AUTORELOAD(htim, period);   // set period
    __HAL_TIM_SET_COUNTER(htim, 0);           // reset counter
    __HAL_TIM_CLEAR_FLAG(htim, TIM_FLAG_UPDATE); // clear pending interrupt
}

static void delay_us(uint16_t us)
{
  uint16_t start = __HAL_TIM_GET_COUNTER(&htim6);
  while (1)
  {
    uint16_t now = __HAL_TIM_GET_COUNTER(&htim6);
    uint16_t elapsed = (uint16_t)(now - start);  // correct even if wrap occurs
    if (elapsed >= us)
      break;
  }
}

// --- Enable/disable driver ---
static void enable(Stepper_t* s)  {   
  HAL_GPIO_WritePin(s->port_en, s->pin_en, GPIO_PIN_RESET);
  s->enabled = 1;
  delay_us(2);
}

static void disable(Stepper_t* s) {
  HAL_GPIO_WritePin(s->port_en, s->pin_en, GPIO_PIN_SET);
  s->enabled = 0;
  delay_us(2);
}

static void abortMovement(Stepper_t *s) {
  LIMITS_MovementStopped(s->valve_limits_state);
  s->state = STATE_IDLE;
  STEP_DEBUG_PRINT("STEP: state=%u abortMovement", s->state);
  s->steps_remaining = 0;
  s->next_steps_remaining = 0;
  s->ramp_index = 1;
  reloadTimer(s->htim, BACKGROUND_INTERVAL_US);  
}

void STEPPER_Disable(Stepper_t *s) {
  disable(s);
}

// --- Start a single step asynchronously ---
static void startStep(Stepper_t* s, uint16_t stepDelay)
{
  s->step_next_delay = stepDelay;
  s->step_state = STEP_STATE_PULSE_HIGH;
  reloadTimer(s->htim, 4); // 4 us pulse width
}

// --- Begin direction change delay (async) ---
void startDirChangeDelay(Stepper_t* s, GPIO_PinState dir)
{
  s->step_next_dir = dir;
  s->step_state = STEP_STATE_DIR_DELAY;
  reloadTimer(s->htim, DIR_CHANGE_DELAY_US);
}

// --- Compute step delay based on ramp profile ---
static uint16_t computeStepDelay(uint16_t rampIndex)
{
  uint32_t f_range = f_max - f_min;
  if (rampIndex > accelSteps) rampIndex = accelSteps;
  uint32_t f_now = f_min + ((uint64_t)rampIndex * f_range) / accelSteps;
  return (uint16_t)(1000000UL / f_now);
}

// --- Movement command update ---
void STEPPER_UpdateMovement(Stepper_t* s, GPIO_PinState next_direction, uint16_t next_steps_remaining)
{
  s->next_direction = next_direction;
  s->next_steps_remaining = next_steps_remaining;
  s->next_version++;
  STEP_DEBUG_PRINT("STEP: NEW MOVEMENT dir=%u steps=%u", next_direction, next_steps_remaining);
}

// --- Movement command update ---
void STEPPER_Home(Stepper_t* s)
{
  s->homing_state = HOME_STATE_UNHOMED;
  STEP_DEBUG_PRINT("STEP: homing_state=%u", s->homing_state);
}

// --- Main movement FSM ---
static uint8_t moveOneStep(Stepper_t* s)
{
  // checking homing
  switch (s->homing_state)
  {
  case HOME_STATE_UNHOMED:
    STEPPER_UpdateMovement(s, HOMING_DIR, HOMING_STEPS);
    s->homing_state = HOME_STATE_HOMING;
    STEP_DEBUG_PRINT("STEP: homing_state=%u", s->homing_state);
    break;
  case HOME_STATE_HOMING:
    if (LIMITS_Stuck(s->valve_limits_state) != 0) {
      abortMovement(s);
      LIMITS_StartBackoff(s->valve_limits_state);
      STEPPER_UpdateMovement(s, HOMING_BACKOFF_DIR, HOMING_BACKOFF_STEPS);
      s->homing_state = HOME_STATE_HOMING_BACKOFF;
      STEP_DEBUG_PRINT("STEP: homing_state=%u", s->homing_state);
    } else {
      if (s->next_steps_remaining == 0) {
        Error_Handler(__FILE__,__LINE__); // failed to home
      }
    }
    break;    
  case HOME_STATE_HOMING_BACKOFF:
    if (LIMITS_Stuck(s->valve_limits_state) == 0) { // stuck got reset, we are not tripping sensor anymore
      LIMITS_ResetPos(s->valve_limits_state);
      s->homing_state = HOME_STATE_HOMED;
      STEP_DEBUG_PRINT("STEP: homing_state=%u", s->homing_state);
      abortMovement(s);
    } else if (s->next_steps_remaining == 0) {// exeeded max steps for backoff
      Error_Handler(__FILE__,__LINE__);
    }
    break;
  case HOME_STATE_HOMED:
    if (LIMITS_Stuck(s->valve_limits_state) != 0) {
      Error_Handler(__FILE__,__LINE__);
    }
    break;
  }
  
  if (s->steps_remaining == 0 &&
      s->next_steps_remaining == 0){
        reloadTimer(s->htim, BACKGROUND_INTERVAL_US);
        s->motor_idle_ticks += BACKGROUND_INTERVAL_US;
        if (s->motor_idle_ticks > AUTO_DISABLE_TIMEOUT_US) { // 1 second
          disable(s);
        }
        return 0;
  }
  // here we are doing some movemnt, reset idle ticks
  s->motor_idle_ticks = 0;
  
  switch (s->state)
  {
  case STATE_IDLE:
    enable(s);
    s->steps_remaining = s->next_steps_remaining;
    s->version = s->next_version;
    s->ramp_index = 1;
    
    if (s->next_direction != s->direction)
    {
      s->direction = s->next_direction;
      startDirChangeDelay(s, (GPIO_PinState)s->direction);
      return 1;
    }
        
    LIMITS_MovementStarted(s->valve_limits_state, (GPIO_PinState)s->direction);
    if (s->steps_remaining <= s->ramp_index) {
        s->state = STATE_RAMP_DOWN;
        STEP_DEBUG_PRINT("STEP: STATE_IDLE=>STATE_RAMP_DOWN");        
    } else {
        s->state = STATE_RAMP_UP;
        STEP_DEBUG_PRINT("STEP: STATE_IDLE=>STATE_RAMP_UP");
    }
    break;
  case STATE_RAMP_UP:
    if (s->next_version != s->version)
    {
      if (s->direction != s->next_direction)
      {
        s->state = STATE_IDLE;
        STEP_DEBUG_PRINT("STEP: STATE_RAMP_UP=>STATE_IDLE s->direction != s->next_direction");
        s->steps_remaining = 0;
        s->ramp_index = 1;
      }
      else
      {
        STEP_DEBUG_PRINT("STEP: STATE_RAMP_UP steps updated %u -> %u", s->steps_remaining, s->next_steps_remaining);
        s->steps_remaining = s->next_steps_remaining;
        s->version = s->next_version;
      }
    }
    if (s->steps_remaining <= s->ramp_index) {
      s->state = STATE_RAMP_DOWN;
      STEP_DEBUG_PRINT("STEP: STATE_RAMP_UP=>STATE_RAMP_DOWN (s->steps_remaining <= s->ramp_index)");
    }
    else if (s->ramp_index >= accelSteps) {
      s->state = STATE_RUN;
      STEP_DEBUG_PRINT("STEP: STATE_RAMP_UP=>STATE_RUN (s->ramp_index >= accelSteps)");
    }
    break;
    
  case STATE_RUN:
    if (s->next_version != s->version)
    {
      if (s->direction != s->next_direction)
      {
        s->state = STATE_IDLE;
        STEP_DEBUG_PRINT("STEP: STATE_RUN=>STATE_IDLE (s->direction != s->next_direction)");
        s->steps_remaining = 0;
        s->ramp_index = 1;
      }
      else
      {
        STEP_DEBUG_PRINT("STEP: STATE_RUN steps updated %u -> %u", s->steps_remaining, s->next_steps_remaining);
        s->steps_remaining = s->next_steps_remaining;
        s->version = s->next_version;
      }
    }
    if (s->steps_remaining <= s->ramp_index) {
      s->state = STATE_RAMP_DOWN;
      STEP_DEBUG_PRINT("STEP: STATE_RUN=>STATE_RAMP_DOWN (s->steps_remaining <= s->ramp_index)");
    }
    break;
    
  case STATE_RAMP_DOWN:
    if (s->steps_remaining == 0)
    {
      LIMITS_MovementStopped(s->valve_limits_state);
      if (s->version != s->next_version)
      {
        s->state = STATE_IDLE;
        STEP_DEBUG_PRINT("STEP: STATE_RAMP_DOWN=>STATE_IDLE (s->version != s->next_version)");
        reloadTimer(s->htim, 2);
        return 1; // call back immediately
      }
      s->state = STATE_IDLE;
      STEP_DEBUG_PRINT("STEP: STATE_RAMP_DOWN=>STATE_IDLE");
      s->steps_remaining = 0;
      s->next_steps_remaining = 0;
      s->ramp_index = 1;
      reloadTimer(s->htim, BACKGROUND_INTERVAL_US);
      return 0;
    }
    break;
  }
  
  uint16_t stepDelay = computeStepDelay(s->ramp_index);
  if (s->state == STATE_RAMP_UP && s->ramp_index < accelSteps) {
#ifdef USE_ACCEL
    if (s->valve_limits_state->full_calibration == 0) {// ramp up only if not in full calibration mode
      s->ramp_index++;
    }
#endif
  } else if (s->state == STATE_RAMP_DOWN && s->ramp_index > 1) {
    s->ramp_index--;
  }
  startStep(s, stepDelay);
  return 1;
}

// --- Unified FSM tick (called by TIM17 ISR) ---
uint8_t STEPPER_Homed(Stepper_t *s)
{
  return s->homing_state == HOME_STATE_HOMED;
}

// --- Getter functions for external access ---
uint16_t STEPPER_GetStepsRemaining(Stepper_t *s)
{
  return s->steps_remaining;
}

uint16_t STEPPER_GetNextStepsRemaining(Stepper_t *s)
{
  return s->next_steps_remaining;
}

// --- 5ms
static uint8_t flt_asserted_ticks = 0;
void STEPPER_FltCheckTick(void)
{
    // read FLT pin and fail if hardware error reported by driver
    if ((cold_valve.enabled || hot_valve.enabled) && !HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_7)) {
      flt_asserted_ticks++;
    } else {
      flt_asserted_ticks = 0;
    }
    // reliably read
    if (flt_asserted_ticks > 32) {
      Error_Handler(__FILE__,__LINE__);
    }
}

// ---
void STEPPER_Tick(Stepper_t *s)
{
  switch (s->step_state)
  {
  case STEP_STATE_IDLE: // background check
    moveOneStep(s);
    break;
    
  case STEP_STATE_PULSE_HIGH:
    HAL_GPIO_WritePin(s->port_step, s->pin_step, GPIO_PIN_SET);
    s->step_state = STEP_STATE_PULSE_LOW;
    reloadTimer(s->htim, 4);
    break;
    
  case STEP_STATE_PULSE_LOW:
    HAL_GPIO_WritePin(s->port_step, s->pin_step, GPIO_PIN_RESET);
    s->step_state = STEP_STATE_INTER_DELAY;
    reloadTimer(s->htim, s->step_next_delay);
    break;
    
  case STEP_STATE_INTER_DELAY:
    s->step_state = STEP_STATE_IDLE;
    // backlash should not be counted with onStep
    if (s->step_backlash_remaining > 0)
    {
      s->step_backlash_remaining--;        
      startStep(s, MAX_DELAY);
      return;
    }

    // non-backhash step completed, check limit switches
    if (s->homing_state == HOME_STATE_HOMED && LIMITS_OnStep(s->valve_limits_state) != 0)
    {
      // Limit hit, stop everything immediately
      abortMovement(s);
      return;
    }
    
    if (s->steps_remaining > 0)
      s->steps_remaining--;
    
    moveOneStep(s);
    break;
    
  case STEP_STATE_DIR_DELAY:
    s->step_state = STEP_STATE_IDLE;
    HAL_GPIO_WritePin(s->port_dir, s->pin_dir, (GPIO_PinState)s->step_next_dir);
    delay_us(2);
    s->step_backlash_remaining = backlashSteps;
    if (s->step_backlash_remaining > 0)
      startStep(s, MAX_DELAY);
    else
      moveOneStep(s);
    break;
  }
}
