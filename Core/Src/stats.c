#include <stdio.h>
#include <string.h>
#include <stdlib.h>

#include "stats.h"
#include "eeprom.h"
#include "serial.h"
#include "ds18b20.h"

/* -------------------------------------------------- */
/* Debug                                               */
/* -------------------------------------------------- */

//#define STATS_DEBUG
#ifdef STATS_DEBUG
static char dbg[256];
#define STATS_DEBUG_PRINT(fmt, ...) do { \
snprintf(dbg, sizeof(dbg), fmt, ##__VA_ARGS__); \
  SERIAL_TransmitNow((uint8_t*)dbg, strlen(dbg)); \
} while(0)
#else
#define STATS_DEBUG_PRINT(fmt, ...) ((void)0)
#endif

/* -------------------------------------------------- */
/* Configuration                                      */
/* -------------------------------------------------- */

#define TEMP_SCALE              16      // DS18B20 raw scale
#define TEMPERATURE_SETTLED_AFTER 20    // ticks (~18s)
#define TEMP_STABLE_EPSILON     1       // ±1 LSB (0.0625°C)

#define AVG_SHIFT               2        // EMA: 1/4
#define CALIBRATION_GAIN_SHIFT  2        // 1/4 correction
#define MAX_CALIBRATION_STEP    (2 * TEMP_SCALE)  // ±2°C per shower

#define MIN_HOT_TEMP            (60 * TEMP_SCALE)
#define MAX_HOT_TEMP            (90 * TEMP_SCALE)

/* -------------------------------------------------- */
/* Global instance                                    */
/* -------------------------------------------------- */

Stats main_stats = {
  .average_temperature = 0,
  .target_temperature = 0,
  .current_temperature = 0,
  .last_temperature = 0,
  .temperature_setteled_ticks = 0,
  .max_temperature = 75 * TEMP_SCALE,
  .measure_temp_errors = 0,
  .eeprom_param = PARAM_11
};

/* -------------------------------------------------- */
/* Utilities                                          */
/* -------------------------------------------------- */

static inline int16_t clamp_i16(int16_t v, int16_t min, int16_t max)
{
  if (v < min) return min;
  if (v > max) return max;
  return v;
}

/* -------------------------------------------------- */
/* Public API                                         */
/* -------------------------------------------------- */

void STATS_DumpState(Stats* s, char* buff, int len)
{
  snprintf(buff, len,
           "STATS average_temperature=%d target_temperature=%d current_temperature=%d last_temperature=%d temperature_setteled_ticks=%u max_temperature=%d measure_temp_errors=%u eeprom_param=%u",
           s->average_temperature,
           s->target_temperature,
           s->current_temperature,
           s->last_temperature,
           s->temperature_setteled_ticks,
           s->max_temperature,
           s->measure_temp_errors,
           s->eeprom_param
             );
}

/* -------------------------------------------------- */
/* Core logic                                         */
/* -------------------------------------------------- */

void STATS_Tick(Stats* s)   // ~0.9 sec
{
  /* ---------------- Temperature read ---------------- */
  if (DS18B20_Ready()) {
    if (DS18B20_Status() != TEMP_OK) {
      s->measure_temp_errors++;
    }
    
    s->last_temperature = s->current_temperature;
    s->current_temperature = DS18B20_GetTemp();
    
    /* EMA average */
    if (s->average_temperature == 0) {
      s->average_temperature = s->current_temperature;
    } else {
      s->average_temperature +=
        (s->current_temperature - s->average_temperature) >> AVG_SHIFT;
    }
    
    DS18B20_StartTempRead();
  }
  
  /* ---------------- Settling detection ---------------- */
  if (abs(s->current_temperature - s->last_temperature)
      <= TEMP_STABLE_EPSILON) {
        s->temperature_setteled_ticks++;
      } else {
        s->temperature_setteled_ticks = 0;
      }
  
  /* ---------------- Calibration ---------------- */
  if (s->target_temperature != 0 &&
      s->temperature_setteled_ticks > TEMPERATURE_SETTLED_AFTER)
  {
    int16_t error =
      s->average_temperature - s->target_temperature;
    
    /* incremental correction */
    int32_t delta = (((int32_t)error * (int32_t)s->max_temperature) / (int32_t)s->target_temperature) >> CALIBRATION_GAIN_SHIFT;
    
    /* clamp correction step */
    delta = clamp_i16(delta,
                      -MAX_CALIBRATION_STEP,
                      MAX_CALIBRATION_STEP);
    
    int16_t last_max_temperature = s->max_temperature;
    
    s->max_temperature += delta;
    
    /* clamp absolute limits */
    s->max_temperature =
      clamp_i16(s->max_temperature,
                MIN_HOT_TEMP,
                MAX_HOT_TEMP);
    
    if (s->max_temperature != last_max_temperature) {
      /* persist result */
      if (EEPROM_OK != EEPROM_Write(s->eeprom_param, s->max_temperature)) {
        Error_Handler(__FILE__, __LINE__);
      }      
    }
    
    STATS_DEBUG_PRINT(
                      "STATS: tgt=%d avg=%d tgt_c=%d avg_c=%d err=%d delta=%d old_max=%d new_max=%d old_max_c=%d new_max_c=%d\n",
                      s->target_temperature,
                      s->average_temperature,
                      s->target_temperature / 16,
                      s->average_temperature / 16,                      
                      error,
                      delta,
                      last_max_temperature,
                      s->max_temperature,
                      last_max_temperature / 16,
                      s->max_temperature / 16
    );
    
    /* one-shot calibration */
    s->target_temperature = 0;
  }
}

/* -------------------------------------------------- */
/* Initialization                                     */
/* -------------------------------------------------- */

void STATS_Init(Stats* s)
{
  uint32_t state = 75 * TEMP_SCALE;
  EEPROM_Read(s->eeprom_param, &state);
  
  s->max_temperature =
    clamp_i16(state, MIN_HOT_TEMP, MAX_HOT_TEMP);
  
  STATS_DEBUG_PRINT(
                    "STATS init max_temperature=%d\n",
                    s->max_temperature
                      );
}

void STATS_SetTargetTemp(Stats* s, int16_t target_temp)
{
  s->target_temperature = target_temp;
  s->temperature_setteled_ticks = 0;
}

int16_t STATS_GetMaxTemperature(Stats* s)
{
  return s->max_temperature;
}
