#include <stdio.h>
#include <string.h>

#include "leak.h"
#include "adc.h"

Leak_t leak;

//#define LEAK_DEBUG
#ifdef LEAK_DEBUG
static char dbg[128];
#define LEAK_DEBUG_PRINT(fmt, ...) do { \
    snprintf(dbg, sizeof(dbg), fmt, ##__VA_ARGS__); \
    SERIAL_TransmitNow((uint8_t*)dbg, strlen(dbg)); \
} while(0)
#else
#define LEAK_DEBUG_PRINT(fmt, ...) ((void)0)
#endif

void LEAK_DumpState(Leak_t* s, char* buff, int len) {
    snprintf(buff, len,
        "LEAK value=%u",
    s->value
    );
}

void LEAK_Init(Leak_t *l) {
  if(HAL_OK != HAL_ADC_Start_DMA(&hadc, (uint32_t *)&l->value, 1)) {
    Error_Handler(__FILE__,__LINE__);
  }
}

uint16_t LEAK_Get(Leak_t *l) {
  return l->value;
}
