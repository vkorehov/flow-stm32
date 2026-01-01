#ifndef __STEP_H__
#define __STEP_H__

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "limit.h"

typedef struct {
    volatile uint16_t steps_remaining; // steps left to move
    volatile uint16_t next_steps_remaining; // steps left to move
    uint16_t ramp_index;
    uint8_t direction; // forward/backward
    uint8_t next_direction; // forward/backward
    uint8_t state; // main state machine
    uint8_t homing_state; // homing sequence
    uint8_t version; // catch up with next_version when we can accept next command
    uint8_t next_version; // increments after UpdateMovement            
    uint8_t step_state; // step pulse state machine
    uint16_t step_next_delay; // delay to apply after pulse will become low
    uint8_t step_next_dir; // new direction to set after bachlash is fully compensated
    uint16_t step_backlash_remaining; // how many steps remaining for backlash compensation (after dir change)
    uint32_t motor_idle_ticks; // ticks counts motors idle, used to autodisable motor
    uint8_t enabled;
    TIM_HandleTypeDef *htim; // dedicated timer for this Stepper
    uint16_t pin_step;
    GPIO_TypeDef *port_step;
    uint16_t pin_dir;
    GPIO_TypeDef *port_dir;
    uint16_t pin_en;
    GPIO_TypeDef *port_en;  
    ValveLimitsState* valve_limits_state;
} Stepper_t;

extern Stepper_t cold_valve;
extern Stepper_t hot_valve;

void STEPPER_DumpState(Stepper_t* s, char* buff, int len);
void STEPPER_Disable(Stepper_t* s);
uint8_t STEPPER_Homed(Stepper_t* s);
void STEPPER_Home(Stepper_t* s);
void STEPPER_Tick(Stepper_t* s);
void STEPPER_UpdateMovement(Stepper_t* s, GPIO_PinState next_direction, uint16_t next_steps_remaining);
uint16_t STEPPER_GetStepsRemaining(Stepper_t* s); // unit testing only
uint16_t STEPPER_GetNextStepsRemaining(Stepper_t* s); // unit testing only
void STEPPER_FltCheckTick(void);

#ifdef __cplusplus
}
#endif

#endif /* __STEP_H__ */

