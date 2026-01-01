#include <stdio.h>
#include <string.h>

#include "limit.h"
#include "eeprom.h"
#include "serial.h"

//#define LIMIT_DEBUG
#ifdef LIMIT_DEBUG
static char dbg[256];
#define LIMIT_DEBUG_PRINT(fmt, ...) do { \
    snprintf(dbg, sizeof(dbg), fmt, ##__VA_ARGS__); \
    SERIAL_TransmitNow((uint8_t*)dbg, strlen(dbg)); \
} while(0)
#else
#define LIMIT_DEBUG_PRINT(fmt, ...) ((void)0)
#endif

/* ---------------- Constants ---------------- */
#define INPUT_STATE_RESET 0x0
#define INPUT_STATE_SET   0xF

/* ---------------- Static Instance ---------------- */
ValveLimitsState cold_valve_limits_state = {
    .stuck          = 0,
    .edge_index     = EDGE_INDEX_UNKNOWN,
    .edge_count     = 0,    
    .full_calibration = 0,
    .pos            = 0,
    .last_edge_ticks= 0,
    .move_direction = GPIO_PIN_UNKNOWN,
    .input_oversampling_bit_index = 0,
    .input_state    = 0x0,   // COLD_VALVE_STATE_RESET
    .input_last_state = 0x0,
    .port_input = GPIOB,
    .pin_input = GPIO_PIN_3,
    .eeprom_param = PARAM_1,
};

ValveLimitsState hot_valve_limits_state = {
    .stuck          = 0,
    .edge_index     = EDGE_INDEX_UNKNOWN,
    .edge_count     = 0,    
    .full_calibration = 0,
    .pos            = 0,
    .last_edge_ticks= 0,
    .move_direction = GPIO_PIN_UNKNOWN,
    .input_oversampling_bit_index = 0,
    .input_state    = 0x0,   // VALVE_STATE_RESET
    .input_last_state = 0x0,
    .port_input = GPIOA,
    .pin_input = GPIO_PIN_1,
    .eeprom_param = PARAM_2,    
};

void LIMITS_DumpState(ValveLimitsState* s, char* buff, int len) {
    snprintf(buff, len,
        "LIMITS stuck=%u edge_index=%u edge_count=%u full_calibration=%u pos=%u last_edge_ticks=%u "
        "move_direction=%u input_oversampling_bit_index=%u input_state=%u input_last_state=%u pin_input=%u eeprom_param=%d",
    s->stuck,
    s->edge_index,
    s->edge_count,    
    s->full_calibration,
    s->pos,
    s->last_edge_ticks,
    s->move_direction,
    s->input_oversampling_bit_index,
    s->input_state,
    s->input_last_state,
    s->pin_input,
    s->eeprom_param
    );
}

void LIMITS_Init(ValveLimitsState* s, uint8_t fullCalibration) {
  // read saved state from EEPROM
  EEPROM_Read(s->eeprom_param, &s->edge_index);
  LIMIT_DEBUG_PRINT("LIMITS EDGE eeprom_param(%d) = %d READ", s->eeprom_param, s->edge_index);
  if (fullCalibration) {
    s->edge_index = EDGE_INDEX_UNKNOWN;
    s->full_calibration = 1;    
    if (EEPROM_OK != EEPROM_Write(s->eeprom_param, s->edge_index)) {
      Error_Handler(__FILE__,__LINE__);
    }
    LIMIT_DEBUG_PRINT("LIMITS EDGE eeprom_param(%d) = %d FULLCAL", s->eeprom_param, s->edge_index);
  }
  // turn on full calibration in case unknown edge counts found
  if (s->edge_index == EDGE_INDEX_UNKNOWN) {
    s->full_calibration = 1;
  }
}

/* ---------------- API ---------------- */
void LIMITS_MovementStopped(ValveLimitsState *s) {
    s->move_direction = GPIO_PIN_UNKNOWN;
    s->last_edge_ticks = 0;
    s->edge_count = 0;
}

void LIMITS_MovementStarted(ValveLimitsState *s, GPIO_PinState dir) {
    s->move_direction = dir;
    s->last_edge_ticks = 0;
    s->edge_count = 0;    
}

uint8_t LIMITS_OnStep(ValveLimitsState *s) {
    if (s->backoff == 0 && s->stuck) {
        return 1;
    }
    // track position
    if (s->move_direction == GPIO_PIN_RESET && s->pos > 0) {
        s->pos -= 1;
    }
    if (s->move_direction == GPIO_PIN_SET && s->pos < MAX_POS) {
        s->pos += 1;
    }
    return 0;
}

/* ---------------- Internal Helpers ---------------- */
static inline void updateEdge(ValveLimitsState *s) {
    if ((s->input_state == INPUT_STATE_RESET && s->input_last_state == INPUT_STATE_SET) ||
        (s->input_state == INPUT_STATE_SET   && s->input_last_state == INPUT_STATE_RESET)) {    
        
        s->input_last_state = s->input_state;
        
        if (s->move_direction == GPIO_PIN_SET) {
            s->edge_index++;
            s->edge_count++;
            if (EEPROM_OK != EEPROM_Write(s->eeprom_param, s->edge_index)) {
                Error_Handler(__FILE__,__LINE__);
            }
            LIMIT_DEBUG_PRINT("LIMITS EDGE eeprom_param(%d) = %d", s->eeprom_param, s->edge_index);
        } else if (s->move_direction == GPIO_PIN_RESET && s->edge_index > 0) {
            s->edge_index--;
            s->edge_count++;            
            if (EEPROM_OK != EEPROM_Write(s->eeprom_param, s->edge_index)) {
                Error_Handler(__FILE__,__LINE__);
            }
            LIMIT_DEBUG_PRINT("LIMITS EDGE eeprom_param(%d) = %d", s->eeprom_param, s->edge_index);
        }
        s->last_edge_ticks = 0;
    }
}

static inline void checkStuck(ValveLimitsState *s) {
    // failsafe in case some random switching between set and unset in case of mechanical elastic bouncing!
    if (s->edge_count > MAX_EDGE_COUNT) {
      if (s->stuck == 0 && s->move_direction == GPIO_PIN_RESET) {
        if (s->input_state == INPUT_STATE_SET) {
          s->edge_index = 1; // reset          
        }
        if (s->input_state == INPUT_STATE_RESET) {
          s->edge_index = 0; // reset          
        }
        s->full_calibration = 0;
        if (EEPROM_OK != EEPROM_Write(s->eeprom_param, s->edge_index)) {
            Error_Handler(__FILE__,__LINE__);
        }
        LIMIT_DEBUG_PRINT("LIMITS EDGE eeprom_param(%d) = %d BOUNCING", s->eeprom_param, s->edge_index);
      }
      s->stuck = 5;
      LIMIT_DEBUG_PRINT("LIMITS STUCK5 eeprom_param(%d)", s->eeprom_param);
      return;
    }
    // hard limits (until physically stuck)
    if (s->input_state == INPUT_STATE_SET && s->last_edge_ticks > STUCK_TICK_COUNT_WHILE_SET) {
      // reset EEPROM edge count only if moving to closing direction
      if (s->stuck == 0 && s->move_direction == GPIO_PIN_RESET) {
        s->edge_index = 1; // reset
        s->full_calibration = 0;
        if (EEPROM_OK != EEPROM_Write(s->eeprom_param, s->edge_index)) {
            Error_Handler(__FILE__,__LINE__);
        }
        LIMIT_DEBUG_PRINT("LIMITS EDGE eeprom_param(%d) = %d STUCK1", s->eeprom_param, s->edge_index);
      }
      s->stuck = 1;
      LIMIT_DEBUG_PRINT("LIMITS STUCK1 eeprom_param(%d)", s->eeprom_param);
      return;
    } else if (s->input_state == INPUT_STATE_RESET && s->last_edge_ticks > STUCK_TICK_COUNT_WHILE_RESET) {
      // reset EEPROM edge count only if moving to closing direction
      if (s->stuck == 0 && s->move_direction == GPIO_PIN_RESET) {
        s->edge_index = 0; // it overshoot past sensor and have to be moved back to have input state ON and edge_index to become 1
        s->full_calibration = 0;
        if (EEPROM_OK != EEPROM_Write(s->eeprom_param, s->edge_index)) {
            Error_Handler(__FILE__,__LINE__);
        }
        LIMIT_DEBUG_PRINT("LIMITS EDGE eeprom_param(%d) = %d STUCK2", s->eeprom_param, s->edge_index);
      }
      s->stuck = 2;
      LIMIT_DEBUG_PRINT("LIMITS STUCK2 eeprom_param(%d)", s->eeprom_param);
      return;
    }
    // soft limits (use eeprom values to detect last edges before hitting physically limit)
    if (s->full_calibration == 0 && s->edge_index == 0 && s->input_state == INPUT_STATE_RESET) {
      s->stuck = 3; // overshooted soft limit hit
      LIMIT_DEBUG_PRINT("LIMITS STUCK3 eeprom_param(%d)", s->eeprom_param);
      return;
    }
    if (s->full_calibration == 0 && s->edge_index == 1 && s->input_state == INPUT_STATE_SET) {
      s->stuck = 3;
      LIMIT_DEBUG_PRINT("LIMITS STUCK3 eeprom_param(%d)", s->eeprom_param);
      return;
    }
    // reset stuck states (during homing backoff only)
    if (s->full_calibration == 0 &&
        s->backoff == 1 &&
        s->edge_index > 1 &&
        s->edge_index < FULLY_OPEN_EDGE_INDEX &&
        s->input_state == INPUT_STATE_RESET) {
      s->stuck = 0;
      s->backoff = 0;
      LIMIT_DEBUG_PRINT("LIMITS RESET eeprom_param(%d)", s->eeprom_param);
      return;
    }
}

void LIMITS_StartBackoff(ValveLimitsState* s) {
    s->backoff = 1;
}

uint16_t LIMITS_GetPos(ValveLimitsState *s) {
    return s->pos;
}

uint8_t LIMITS_Stuck(ValveLimitsState *s) {
    return s->stuck;
}

void LIMITS_ResetPos(ValveLimitsState *s) {
    s->pos = 0;
}

/* ---------------- Tick Handler (5 ms) ---------------- */
void LIMITS_Tick(ValveLimitsState *s) {
    // Oversampling
    if (HAL_GPIO_ReadPin(s->port_input, s->pin_input)) {
        s->input_state |= (uint16_t)(1 << s->input_oversampling_bit_index);
    } else {
        s->input_state &= ~((uint16_t)(1 << s->input_oversampling_bit_index));
    }  
    s->input_oversampling_bit_index = (s->input_oversampling_bit_index + 1) & 0x3;
    if (s->input_oversampling_bit_index != 0) {
        return;// skip processing if not full
    }  
    // edge detection every 20ms
    if (s->move_direction != GPIO_PIN_UNKNOWN) {
      updateEdge(s);
      if (s->last_edge_ticks < MAX_TICKS) {
        s->last_edge_ticks++;
      }
      checkStuck(s);
    }
}
