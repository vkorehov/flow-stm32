#include <stdio.h>
#include <string.h>
#include <stdint.h>
#include <limits.h>

#include "feedforward.h"
#include "eeprom.h"
#include "serial.h"

#define HISTORY_SIZE           4
#define FF_LEARN_SHIFT         3   // divide by 8
#define FF_MAX_STEP_CORR       20   // max steps change per update
#define FF_MIN_STEPS           0
#define FF_MAX_STEPS           3000

/* -------------------------------------------------- */
/* Debug                                               */
/* -------------------------------------------------- */
//#define FF_DEBUG
#ifdef FF_DEBUG
static char dbg[256];
#define FF_DEBUG_PRINT(fmt, ...) do { \
snprintf(dbg, sizeof(dbg), fmt, ##__VA_ARGS__); \
  SERIAL_TransmitNow((uint8_t*)dbg, strlen(dbg)); \
} while (0)
#else
#define FF_DEBUG_PRINT(fmt, ...) ((void)0)
#endif

/* -------------------------------------------------- */
/* State                                               */
/* -------------------------------------------------- */

FF_State cold_ff_state = {
  .history = {
    {.eeprom_param = PARAM_3},
    {.eeprom_param = PARAM_4},
    {.eeprom_param = PARAM_5},
    {.eeprom_param = PARAM_6},
  }
};

FF_State hot_ff_state = {
  .history = {
    {.eeprom_param = PARAM_7},
    {.eeprom_param = PARAM_8},
    {.eeprom_param = PARAM_9},
    {.eeprom_param = PARAM_10},
  }
};

/* -------------------------------------------------- */
/* Diagnostics                                         */
/* -------------------------------------------------- */

void FF_DumpState(FF_State* s, char* buff, int len)
{
  snprintf(buff, len,
           "FF %d->%d,%d->%d,%d->%d,%d->%d",
           s->history[0].target, s->history[0].steps,
           s->history[1].target, s->history[1].steps,
           s->history[2].target, s->history[2].steps,
           s->history[3].target, s->history[3].steps
             );
}

/* -------------------------------------------------- */
/* Internal helper: write a history entry to EEPROM  */
/* -------------------------------------------------- */
static void write_history_entry(FF_History_Entry* h)
{
  uint32_t packed =
    ((uint32_t)(uint16_t)h->steps << 16) |
      (uint16_t)h->target;
  
  if (EEPROM_OK != EEPROM_Write(h->eeprom_param, packed)) {
    Error_Handler(__FILE__, __LINE__);
  }
}

/* -------------------------------------------------- */
/* History management (newest-first ring buffer)      */
/* -------------------------------------------------- */
void FF_SaveInHistory(FF_State* s,
                      uint16_t target,
                      uint16_t actual,
                      uint16_t steps)
{
  if (target == 0) { // DIV by zero prevention
      return;
  }
  /* try to find existing entry */
  for (uint8_t i = 0; i < HISTORY_SIZE; i++) {
    if (s->history[i].target == target) {
      int32_t error = (int32_t)target - (int32_t)actual;
      int32_t delta = (((int32_t)steps * (int32_t)error)/(int32_t)target) >> FF_LEARN_SHIFT;
      if (delta >  FF_MAX_STEP_CORR) delta =  FF_MAX_STEP_CORR;
      if (delta < -FF_MAX_STEP_CORR) delta = -FF_MAX_STEP_CORR;
      
      /* incremental learning */
      int32_t new_steps = (int32_t)steps + delta;
      
      if (new_steps < FF_MIN_STEPS) new_steps = FF_MIN_STEPS;
      if (new_steps > FF_MAX_STEPS) new_steps = FF_MAX_STEPS;
      
      /* move to front (MRU) */
      for (uint8_t j = i; j > 0; j--) {
        s->history[j] = s->history[j-1];
        write_history_entry(&s->history[j]);
      }
      
      uint16_t old_steps = s->history[0].steps;
      s->history[0].target = target;
      s->history[0].steps  = (uint16_t)new_steps;
      write_history_entry(&s->history[0]);
      
      FF_DEBUG_PRINT(
                     "FF learn tgt=%u act=%u err=%d delta=%d old_steps=%u new_steps=%u\n",
                     target, actual, error, delta, old_steps, s->history[0].steps
                       );
      return;
    }
  }
  
  /* new entry: seed with measured steps */
  for (int i = HISTORY_SIZE - 1; i > 0; i--) {
    s->history[i] = s->history[i-1];
    write_history_entry(&s->history[i]);
  }
  
  s->history[0].target = target;
  s->history[0].steps  = steps;
  write_history_entry(&s->history[0]);
  
  FF_DEBUG_PRINT(
                 "FF new tgt=%u steps=%u\n",
                 target, steps
                   );
}

/* -------------------------------------------------- */
/* Initialization                                      */
/* -------------------------------------------------- */
void FF_Init(FF_State* s)
{
  for (uint8_t i = 0; i < HISTORY_SIZE; i++) {
    uint32_t state = 0;
    EEPROM_Read(s->history[i].eeprom_param, &state);
    
    s->history[i].target = (uint16_t)(state & 0xFFFF);
    s->history[i].steps  = (uint16_t)((state >> 16) & 0xFFFF);
  }
  
  FF_DEBUG_PRINT(
                 "FF loaded history %d->%d,%d->%d,%d->%d,%d->%d",
                 s->history[0].target, s->history[0].steps,
                 s->history[1].target, s->history[1].steps,
                 s->history[2].target, s->history[2].steps,
                 s->history[3].target, s->history[3].steps
                   );
}

/* -------------------------------------------------- */
/* Initialization                                      */
/* -------------------------------------------------- */
void FF_Clear(FF_State* s)
{
  for (uint8_t i = 0; i < HISTORY_SIZE; i++) {
    s->history[i].target = 0;
    s->history[i].steps  = 0;
    write_history_entry(&s->history[i]);        
  }
  
  FF_DEBUG_PRINT(
                 "FF history cleared %d->%d,%d->%d,%d->%d,%d->%d",
                 s->history[0].target, s->history[0].steps,
                 s->history[1].target, s->history[1].steps,
                 s->history[2].target, s->history[2].steps,
                 s->history[3].target, s->history[3].steps
                   );
}

/* -------------------------------------------------- */
/* Feedforward lookup (linear interpolation)          */
/* -------------------------------------------------- */
uint16_t FF_GetSteps(FF_State* s, uint16_t target)
{
  int32_t lo_t = 0, lo_s = 0;
  int32_t hi_t = 0, hi_s = 0;
  uint8_t lo_found = 0, hi_found = 0;
  if (target == 0) {
    return 0;
  }
  
  for (uint8_t i = 0; i < HISTORY_SIZE; i++) {
    int32_t t = s->history[i].target;
    if (t == 0) continue;
    
    if (t == target) return s->history[i].steps;
    
    if (t < target) {
      if (!lo_found || t > lo_t) {
        lo_t = t;
        lo_s = s->history[i].steps;
        lo_found = 1;
      }
    } else {
      if (!hi_found || t < hi_t) {
        hi_t = t;
        hi_s = s->history[i].steps;
        hi_found = 1;
      }
    }
  }
  
  if (!lo_found && !hi_found) return 0;
  if (!lo_found) {
    return (uint16_t)(
        hi_s + ((target - hi_t) * hi_s) / hi_t
    );
  }
  if (!hi_found) {
    return (uint16_t)(
       (lo_s * target) / lo_t
    );
  }
  
  uint32_t num   = (uint32_t)(target - lo_t) * (hi_s - lo_s);
  uint32_t denom = (hi_t - lo_t);
  
  return (uint16_t)(lo_s + (num / denom));
}
