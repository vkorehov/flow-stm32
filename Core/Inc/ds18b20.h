#ifndef __DS18B20_H__
#define __DS18B20_H__

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "main.h"

// Status of the last operation
typedef enum { TEMP_OK = 0, TEMP_ERROR = 1 } TempStatus_t;

/* ============================================================================
 * DS18B20 Temperature Sensor API Functions
 * ============================================================================ */

void DS18B20_DumpState(char* buff, int len);

/**
 * @brief  Initialize DS18B20 sensor
 * @retval None
 */
void DS18B20_Init(void);

/**
 * @brief  Start temperature reading process
 * @retval None
 */
void DS18B20_StartTempRead(void);

/**
 * @brief  Temperature sensor tick function (call periodically)
 * @retval None
 */
void DS18B20_Tick(void);

void DS18B20_SimulateTick(void);

/**
 * @brief  Get temperature value (scaled by 16)
 * @retval Temperature value in 1/16 degrees Celsius
 */
int16_t DS18B20_GetTemp(void);

int16_t DS18B20_GetSimulatedTemp(void);

/**
 * @brief  Check if temperature reading is ready
 * @retval 1 if ready, 0 if still processing
 */
uint8_t DS18B20_Ready(void);

/**
 * @brief  Get status of last temperature operation
 * @retval TEMP_OK or TEMP_ERROR
 */
TempStatus_t DS18B20_Status(void);

/* ============================================================================
 * UART Callback Wrapper Functions
 * ============================================================================ */
/**
 * @brief  Handle UART transmit complete event
 * @retval None
 */
void DS18B20_TxCplt(void);

/**
 * @brief  Handle UART receive complete event
 * @retval None
 */
void DS18B20_RxCplt(void);

#ifdef __cplusplus
}
#endif

#endif /* __DS18B20_H__ */

