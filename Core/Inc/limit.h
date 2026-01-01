#ifndef __LIMIT_H__
#define __LIMIT_H__

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "main.h"

typedef struct {
    uint8_t  stuck;
    uint32_t edge_index; // 32 bit to be compatible with eeprom read/write
    uint16_t edge_count;
    uint8_t  full_calibration;
    uint8_t  backoff;    
    int32_t  pos;                // current position in steps
    uint16_t last_edge_ticks;
    uint8_t  move_direction;
    uint16_t input_oversampling_bit_index;
    uint16_t input_state;
    uint16_t input_last_state;
    uint16_t pin_input;
    GPIO_TypeDef *port_input; 
    uint32_t eeprom_param;
} ValveLimitsState;

void LIMITS_DumpState(ValveLimitsState*s, char* buff, int len);

extern ValveLimitsState cold_valve_limits_state;
extern ValveLimitsState hot_valve_limits_state;

/* ---------------- Constants ---------------- */
#define FULLY_OPEN_EDGE_INDEX   6
#define MAX_EDGE_COUNT   10

#define STUCK_WITH_LIMIT_HIT    1
#define STUCK_WITHOUT_LIMIT_HIT 2

#define STUCK_TICK_COUNT_WHILE_SET   100 // for 20ms updateEdge()
#define STUCK_TICK_COUNT_WHILE_RESET 500 // for 20ms updateEdge()
#define MAX_TICKS                    0x1FFF
#define GPIO_PIN_UNKNOWN             0xFF  // unknown state of input pin or direction
#define EDGE_INDEX_UNKNOWN           0xFF  // unknown state of input pin or direction
#define MAX_POS                      11500

/* ---------------- Public API ---------------- */
void     LIMITS_Init(ValveLimitsState* s, uint8_t fullCalibration);
void     LIMITS_Tick(ValveLimitsState* s);
void     LIMITS_MovementStopped(ValveLimitsState* s);
void     LIMITS_MovementStarted(ValveLimitsState* s, GPIO_PinState dir);
uint8_t  LIMITS_OnStep(ValveLimitsState* s);
uint16_t LIMITS_GetPos(ValveLimitsState* s);
uint8_t  LIMITS_Stuck(ValveLimitsState* s);
void     LIMITS_ResetPos(ValveLimitsState* s);
void     LIMITS_StartBackoff(ValveLimitsState* s);

#ifdef __cplusplus
}
#endif

#endif /* __LIMIT_H__ */
