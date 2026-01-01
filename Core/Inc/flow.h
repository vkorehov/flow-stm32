#ifndef __FLOW_H__
#define __FLOW_H__

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "main.h"
#define MAX_FLOW_RATE 100

void FLOW_DumpState(char* buff, int len);

/**
 * @brief  Flow simulation tick function (call periodically)
 * @retval None
 */
void FLOW_SimulateTick(void); 

/**
 * @brief  Flow measurement tick function (call periodically)
 * @retval None
 */
void FLOW_Tick(void);

/**
 * @brief  Get current flow rate
 * @retval Current flow rate value
 */
uint16_t FLOW_Rate(void);

/**
 * Compute required hot water flow rate (integer math)
 *
 * @param cold_flow_rate  Cold water flow rate (e.g., in 0.01 L/min units)
 * @param target_temp     Desired mixed temperature (°C × 100)
 * @param hot_temp     Hot pipe temperature (°C × 100)
 * @return                Hot water flow rate in same units as cold_flow_rate
 */
uint16_t FLOW_GetHotFlow(uint16_t cold_flow_rate, uint16_t target_temp, uint16_t hot_temp);

/**
 * @brief  Get current simulated flow rate
 * @retval Current simulated flow rate value
 */
uint16_t FLOW_SimulatedRate(void); 

#ifdef __cplusplus
}
#endif

#endif /* __FLOW_H__ */

