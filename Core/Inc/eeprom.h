#ifndef __EEPROM_H__
#define __EEPROM_H__

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "main.h"

#include "stm32f0xx_hal.h"

/* Declarations and definitions ----------------------------------------------*/

#define PAGE_DATA_OFFSET                                                8
#define PAGE_DATA_SIZE                                                  8

#define PARAM_1                                                         0x12121212
#define PARAM_2                                                         0x34343434

#define PARAM_3                                                         0x45454545
#define PARAM_4                                                         0x56565656
#define PARAM_5                                                         0x67676767
#define PARAM_6                                                         0x78787878
#define PARAM_7                                                         0x89898989
#define PARAM_8                                                         0x9a9a9a9a
#define PARAM_9                                                         0xabababab
#define PARAM_10                                                        0xbcbcbcbc
#define PARAM_11                                                        0xcdcdcdcd

#define VAR_NUM                                                         11

#define PAGE_0_ADDRESS                                                  0x0800F800
#define PAGE_1_ADDRESS                                                  0x0800FC00
#define PAGE_SIZE                                                       1024

void EEPROM_DumpState(char* buff, int len);

typedef enum {
  PAGE_CLEARED = 0xFFFFFFFF,
  PAGE_ACTIVE = 0x00000000,
  PAGE_RECEIVING_DATA = 0x55555555,
} PageState;

typedef enum {
  PAGE_0 = 0,
  PAGE_1 = 1,
  PAGES_NUM = 2,
} PageIdx;

typedef enum {
  EEPROM_OK = 0,
  EEPROM_ERROR = 1,
} EepromResult;



/* Functions -----------------------------------------------------------------*/

extern EepromResult EEPROM_Init();
extern EepromResult EEPROM_Read(uint32_t varId, uint32_t *varValue);
extern EepromResult EEPROM_Write(uint32_t varId, uint32_t varValue);


#ifdef __cplusplus
}
#endif

#endif /* __EEPROM_H__ */
