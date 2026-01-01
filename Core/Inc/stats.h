#ifndef __STATS_H__
#define __STATS_H__

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "main.h"

typedef struct {
    int16_t  target_temperature;
    int16_t  current_temperature;
    int16_t  last_temperature;
    uint16_t  temperature_setteled_ticks;
    int16_t  average_temperature;
    int32_t  max_temperature; // slowly re-calibrate max temperature to ensure long term stability
    uint16_t measure_temp_errors;
    uint32_t eeprom_param;
} Stats;

void STATS_DumpState(Stats*s, char* buff, int len);

extern Stats main_stats;

/* ---------------- Public API ---------------- */
void STATS_Init(Stats* s);
void STATS_Tick(Stats* s); // 1 sec
int16_t STATS_GetMaxTemperature(Stats* s);
void STATS_SetTargetTemp(Stats* s, int16_t target_temp);

#ifdef __cplusplus
}
#endif

#endif /* __STATS_H__ */
