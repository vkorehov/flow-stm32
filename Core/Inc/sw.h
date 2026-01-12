#ifndef __SW_H__
#define __SW_H__

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "main.h"

typedef struct {
    uint8_t state;
    uint32_t ticks;
    uint16_t pin_sense;
    GPIO_TypeDef *port_sense;
    uint16_t pin_sw;
    GPIO_TypeDef *port_sw;
} SW_t;

extern SW_t cold_sw;
extern SW_t hot_sw;
extern SW_t shower1_sw;
extern SW_t shower2_sw;
extern SW_t shower3_sw;

void SW_DumpState(SW_t* s, char* buff, int len);
void SW_Tick(SW_t* sw);
void SW_On(SW_t* sw);
void SW_Off(SW_t* sw);

#ifdef __cplusplus
}
#endif

#endif /* __SW_H__ */
