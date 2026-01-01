#ifndef __FF_H__
#define __FF_H__

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "main.h"

typedef struct {
  uint16_t target;
  uint16_t steps;
  uint32_t eeprom_param;
} FF_History_Entry;

typedef struct {
  FF_History_Entry history[4];
} FF_State;

extern FF_State cold_ff_state;
extern FF_State hot_ff_state;

void FF_Init(FF_State* s);
uint16_t FF_GetSteps(FF_State* s, uint16_t target);
void FF_SaveInHistory(FF_State* s,
                      uint16_t target,
                      uint16_t actual,
                      uint16_t steps);
void FF_DumpState(FF_State* s, char* buff, int len);
void FF_Clear(FF_State* s);

#ifdef __cplusplus
}
#endif

#endif /* __FF_H__ */

