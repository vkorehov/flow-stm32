#include <stdio.h>
#include <string.h>

#include "sw.h"
#include "serial.h"

#define MAX_SW_TICKS 240000 // each tick is 0.005s it is 1200sec

//#define SW_DEBUG
#ifdef SW_DEBUG
static char dbg[128];
#define SW_DEBUG_PRINT(fmt, ...) do { \
    snprintf(dbg, sizeof(dbg), fmt, ##__VA_ARGS__); \
    SERIAL_TransmitNow((uint8_t*)dbg, strlen(dbg)); \
} while(0)
#else
#define SW_DEBUG_PRINT(fmt, ...) ((void)0)
#endif

SW_t cold_sw = {
  .pin_sense = GPIO_PIN_9, 
  .port_sense = GPIOB,
  .pin_sw = GPIO_PIN_8,
  .port_sw = GPIOB,
  .state = 0,
  .ticks = 0,  
};
SW_t hot_sw = {
  .pin_sense = GPIO_PIN_14,
  .port_sense = GPIOC,
  .pin_sw = GPIO_PIN_13, 
  .port_sw = GPIOC,
  .state = 0,
  .ticks = 0,  
};
SW_t shower1_sw = {
  .pin_sense = GPIO_PIN_7,
  .port_sense = GPIOF,
  .pin_sw = GPIO_PIN_6,
  .port_sw = GPIOF,
  .state = 0,
  .ticks = 0,  
};
SW_t shower2_sw = {
  .pin_sense = GPIO_PIN_15,
  .port_sense = GPIOB,
  .pin_sw = GPIO_PIN_14,
  .port_sw = GPIOB,
  .state = 0,
  .ticks = 0,  
};
SW_t shower3_sw = {
  .pin_sense = GPIO_PIN_13,
  .port_sense = GPIOB,
  .pin_sw = GPIO_PIN_12,
  .port_sw = GPIOB,
  .state = 0,
  .ticks = 0,  
};

void SW_DumpState(SW_t* s, char* buff, int len) {
    snprintf(buff, len,
        "SW pin_sense=%u pin_sw=%u state=%u ticks=%u",
    s->pin_sense,
    s->pin_sw,
    s->state,    
    s->ticks
    );
}


void SW_Tick(SW_t* sw) { // 5ms
  if (sw->state) {
    sw->ticks++;
    if (sw->ticks > MAX_SW_TICKS) {
      SW_DEBUG_PRINT("SW: TOO LONG ON pin=%u", sw->pin_sw);
      Error_Handler(__FILE__,__LINE__);
    }
  } else {
    sw->ticks = 0;
  }
  // fault detection
  if (sw->state && GPIO_PIN_RESET == HAL_GPIO_ReadPin(sw->port_sense, sw->pin_sense)) {
    SW_DEBUG_PRINT("SW: FAULT DETECTED pin=%u", sw->pin_sense);
    Error_Handler(__FILE__,__LINE__);
  }
}

void SW_On(SW_t* sw) {
  HAL_GPIO_WritePin(sw->port_sw, sw->pin_sw, GPIO_PIN_SET);
  sw->state = 1;
  SW_DEBUG_PRINT("SW: ON pin=%u", sw->pin_sw);
}

void SW_Off(SW_t* sw) {
  sw->state = 0;
  HAL_GPIO_WritePin(sw->port_sw, sw->pin_sw, GPIO_PIN_RESET);
  SW_DEBUG_PRINT("SW: OFF pin=%u", sw->pin_sw);
}
