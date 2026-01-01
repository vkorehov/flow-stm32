/* USER CODE BEGIN Header */
/**
******************************************************************************
* @file           : main.c
* @brief          : Main program body
******************************************************************************
* @attention
*
* Copyright (c) 2025 STMicroelectronics.
* All rights reserved.
*
* This software is licensed under terms that can be found in the LICENSE file
* in the root directory of this software component.
* If no LICENSE file comes with this software, it is provided AS-IS.
*
******************************************************************************
*/
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include <stdio.h>
#include <string.h>
#include <stdlib.h>

#include "main.h"
#include "eeprom.h"
#include "dma.h"
#include "tim.h"
#include "usart.h"
#include "gpio.h"
#include "sw.h"
#include "ds18b20.h"
#include "limit.h"
#include "flow.h"
#include "pid.h"
#include "step.h"
#include "serial.h"
#include "feedforward.h"
#include "stats.h"

#define CMD_IDLE 0xff

void SystemClock_Config(void);
char msg[512];

/* 
* Blocking microsecond delay macro (approximate)
* Assumes CPU clock = 48 MHz
* 1 loop iteration ~ 4 cycles => 12 iterations per 1 us
*/
#define DELAY_US(us)                     \
do {                                 \
  volatile uint32_t _i;            \
    for (_i = (us) * 12; _i != 0; _i--) { __NOP(); } \
} while (0)

static uint16_t pid_every = 0;
static uint16_t stats_every = 0;
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim) {
  if (htim->Instance == TIM1) {
    SW_Tick(&cold_sw); //0.005s
    SW_Tick(&hot_sw); //0.005s
    SW_Tick(&shower1_sw); //0.005s
    SW_Tick(&shower2_sw); //0.005s
    
    STEPPER_FltCheckTick(); //0.005s
    LIMITS_Tick(&cold_valve_limits_state); //0.005s
    LIMITS_Tick(&hot_valve_limits_state); //0.005s    
    if (pid_every >= 60) {
      PID_MeasureTick();//0.3s      
      PID_Tick(&cold_valve_pid); //0.3s
      PID_Tick(&hot_valve_pid); //0.3s      
      pid_every = 0;
    } else {
      pid_every++;
    }
    if (stats_every >= 180) {
      STATS_Tick(&main_stats);//0.9s
      stats_every = 0;
    } else {
      stats_every++;
    }    
  } else if (htim->Instance == TIM15) {
    DS18B20_Tick();
  } else if (htim->Instance == TIM16) {
    STEPPER_Tick(&hot_valve);
  } else if (htim->Instance == TIM17) {
    STEPPER_Tick(&cold_valve);
  }
}

/**
* @brief  UART Transmit Complete Callback
* @param  huart: UART handle
* @retval None
*/
void HAL_UART_TxCpltCallback(UART_HandleTypeDef *huart)
{
  if (huart == &huart1) {
    SERIAL_TxCplt();
  }
  if (huart == &huart2) {
    // Call DS18B20 wrapper function
    DS18B20_TxCplt();    
  }
}

/**
* @brief  UART Receive Complete Callback
* @param  huart: UART handle
* @retval None
*/
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
  if (huart == &huart1) {
    SERIAL_RxCplt();
  }
  if (huart == &huart2) {
    // Call DS18B20 wrapper function
    DS18B20_RxCplt();
  }
}

void HAL_UART_ErrorCallback(UART_HandleTypeDef *huart) {
  /* Clear all UART error flags */
  __HAL_UART_CLEAR_OREFLAG(huart);
  __HAL_UART_CLEAR_FEFLAG(huart);
  __HAL_UART_CLEAR_NEFLAG(huart);
  __HAL_UART_CLEAR_PEFLAG(huart);
  if (huart == &huart1) {
    /* Restart Serial module */
    SERIAL_Init();
  }
  if (huart == &huart2) {
    Error_Handler(__FILE__,__LINE__);
  }
}

typedef struct {
  uint8_t  cmd;
  uint8_t  param1;    
  uint8_t  param2;
  uint8_t  param3;    
  uint8_t  param4;
  uint8_t  param5;
  uint8_t  param6;  
} Command_t;

volatile Command_t next_command = {.cmd = CMD_IDLE, .param1 = 0x00};
uint16_t current_cold_flow = 0;

void SERIAL_OnReceive(const uint8_t *payload, uint16_t len)
{
  if (len < 1) {
    return; // invalid len
  }
  if (next_command.cmd == CMD_IDLE) {
    next_command.param1 = (len > 1 ? payload[1] : 0x00);
    next_command.param2 = (len > 2 ? payload[2] : 0x00);    
    next_command.param3 = (len > 3 ? payload[3] : 0x00);    
    next_command.param4 = (len > 4 ? payload[4] : 0x00);    
    next_command.param5 = (len > 5 ? payload[5] : 0x00);    
    next_command.param6 = (len > 6 ? payload[6] : 0x00);        
    next_command.cmd = payload[0];
  }
}

/**
* @brief  The application entry point.
* @retval int
*/
int main(void)
{
  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();
  /* Configure the system clock */
  SystemClock_Config();
  
  if (EEPROM_OK != EEPROM_Init()) {
    Error_Handler(__FILE__,__LINE__);
  }
  // read state from EEPROM
  LIMITS_Init(&cold_valve_limits_state, 0);
  LIMITS_Init(&hot_valve_limits_state, 0);  
  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_DMA_Init();
  MX_USART1_UART_Init();
  MX_USART2_UART_Init();
  MX_TIM1_Init();
  MX_TIM2_Init();
  MX_TIM3_Init();
  MX_TIM6_Init();
  MX_TIM14_Init();
  MX_TIM15_Init();
  MX_TIM16_Init();
  MX_TIM17_Init();
  if (HAL_TIM_Base_Start(&htim3) != HAL_OK)
  {
    Error_Handler(__FILE__,__LINE__);
  }  
  if (HAL_TIM_Base_Start(&htim6) != HAL_OK)
  {
    Error_Handler(__FILE__,__LINE__);
  }
  if (HAL_TIM_Base_Start_IT(&htim15) != HAL_OK)
  {
    Error_Handler(__FILE__,__LINE__);
  }
  if (HAL_TIM_Base_Start_IT(&htim16) != HAL_OK)
  {
    Error_Handler(__FILE__,__LINE__);
  }
  if (HAL_TIM_Base_Start_IT(&htim17) != HAL_OK)
  {
    Error_Handler(__FILE__,__LINE__);
  }
  if (HAL_TIM_Base_Start(&htim2) != HAL_OK)
  {
    Error_Handler(__FILE__,__LINE__);
  }
  if (HAL_TIM_Base_Start_IT(&htim1) != HAL_OK)
  {
    Error_Handler(__FILE__,__LINE__);
  }  
  DS18B20_Init();
  DS18B20_StartTempRead();  
  SERIAL_Init(); 
  
  FF_Init(&cold_ff_state);
  FF_Init(&hot_ff_state);

  STATS_Init(&main_stats); 
  
  snprintf(msg, sizeof(msg), "FLOW: BOOTED");
  SERIAL_TransmitNow((uint8_t*)msg, strlen(msg));  
  while (1)
  {
    Command_t cmd;
    //SERIAL_Poll();
    if (next_command.cmd != CMD_IDLE) {
      __disable_irq();
      cmd = next_command;
      // mark command as processed.
      next_command.cmd = CMD_IDLE;
      __enable_irq(); 
      
      uint8_t cold_pid_status = 0xff;
      uint8_t hot_pid_status = 0xff;
      uint16_t target_flow_cold = 0;      
      uint16_t target_flow_hot = 0;
      uint16_t target_pos_hot = 0;
      uint16_t target_pos_cold = 0;      
      uint16_t target_temp = 0;
      uint16_t saved_steps_cold = 0;
      uint16_t saved_steps_hot = 0;      
      DELAY_US(10000);
      switch(cmd.cmd) {
      case 0:
        // safety, turn off all
        SW_Off(&cold_sw);
        SW_Off(&hot_sw);
        // allow to turn off valve, because water is not compressable!
        if (cmd.param2 == 0x00) {
          SW_On(&shower1_sw);
          SW_Off(&shower2_sw);
        }
        if (cmd.param2 == 0x01) {
          SW_Off(&shower1_sw);
          SW_On(&shower2_sw);
        }
        PID_Disable(&cold_valve_pid, 0);
        PID_Disable(&hot_valve_pid, 0);
        snprintf(msg, sizeof(msg), "NOFLOW: STARTED");
        SERIAL_TransmitNow((uint8_t*)msg, strlen(msg));
        STEPPER_Home(&cold_valve);
        STEPPER_Home(&hot_valve);
        while(!STEPPER_Homed(&cold_valve) || !STEPPER_Homed(&hot_valve)) {}
        SW_Off(&shower1_sw);
        SW_Off(&shower2_sw);        
        snprintf(msg, sizeof(msg), "NOFLOW: OK");
        SERIAL_TransmitNow((uint8_t*)msg, strlen(msg));
        break;
      case 1: // pid flow cold shower
        // safety, turn off all
        SW_Off(&cold_sw);
        SW_Off(&hot_sw);
        STATS_SetTargetTemp(&main_stats, 0);
        target_flow_cold = ((uint16_t)cmd.param1 << 8) | cmd.param2;        
        // allow to turn off valve, because water is not compressable!
        switch(cmd.param3) {
        case 0:
          SW_On(&shower1_sw);
          SW_Off(&shower2_sw);
          break;
        case 1:
          SW_On(&shower1_sw);
          SW_Off(&shower2_sw);
          break;
        }
        if (cmd.param4 == 1) { // clear history and force use PID
            FF_Clear(&cold_ff_state);
        }
        PID_Disable(&hot_valve_pid, 0);
        PID_Disable(&cold_valve_pid, 0);
        snprintf(msg, sizeof(msg), "COLDFLOW: STARTING");
        SERIAL_TransmitNow((uint8_t*)msg, strlen(msg));        
        STEPPER_Home(&cold_valve);
        STEPPER_Home(&hot_valve);
        while(!STEPPER_Homed(&cold_valve) || !STEPPER_Homed(&hot_valve)) {}
        snprintf(msg, sizeof(msg), "COLDFLOW: HOMED");
        saved_steps_cold = FF_GetSteps(&cold_ff_state, target_flow_cold);
        // turn water flow
        SW_On(&cold_sw);
        if (saved_steps_cold == 0) { // no info yet, lets use PID
          PID_Setpoint(&cold_valve_pid, target_flow_cold, 0);
          PID_Enable(&cold_valve_pid);
          while(0xff == (cold_pid_status = PID_Stable(&cold_valve_pid))) {}
          if (cold_pid_status == 0) {
            PID_Disable(&cold_valve_pid, 0);
            snprintf(msg, sizeof(msg), "COLDFLOW: OK pos=%u pid=true flow=%u", LIMITS_GetPos(&cold_valve_limits_state), cold_valve_pid.flow);
            SERIAL_TransmitNow((uint8_t*)msg, strlen(msg));
            // save historic data
            FF_SaveInHistory(&cold_ff_state, target_flow_cold, cold_valve_pid.flow, LIMITS_GetPos(&cold_valve_limits_state));
          } else {
            snprintf(msg, sizeof(msg), "COLDFLOW: PID_ERROR pos=%u pid=true flow=%u", LIMITS_GetPos(&cold_valve_limits_state), cold_valve_pid.flow);
            SERIAL_TransmitNow((uint8_t*)msg, strlen(msg));
            SW_Off(&cold_sw);
            SW_Off(&hot_sw);
            SW_Off(&shower1_sw);
            SW_Off(&shower2_sw);
            PID_Disable(&cold_valve_pid, 0);
            PID_Disable(&hot_valve_pid, 0);
          }
        } else {
          STEPPER_UpdateMovement(&cold_valve, GPIO_PIN_SET, saved_steps_cold);
          while(cold_valve.next_steps_remaining != 0) {}
          // wait for next measurement
          DELAY_US(350000);
          // correcting historic data
          FF_SaveInHistory(&cold_ff_state, target_flow_cold, cold_valve_pid.flow, saved_steps_cold);
          snprintf(msg, sizeof(msg), "COLDFLOW: OK pos=%u pid=false flow=%u", LIMITS_GetPos(&cold_valve_limits_state), cold_valve_pid.flow);
          SERIAL_TransmitNow((uint8_t*)msg, strlen(msg));
        }
        break;
      case 2: // pid flow warm shower
        SW_Off(&cold_sw);
        SW_Off(&hot_sw);        
        target_flow_cold = ((uint16_t)cmd.param1 << 8) | cmd.param2; // cold water flow baseline
        target_temp = ((uint16_t)cmd.param3 << 8) | cmd.param4;
        STATS_SetTargetTemp(&main_stats, target_temp);
        switch(cmd.param5) {
        case 0:
          SW_On(&shower1_sw);
          SW_Off(&shower2_sw);
          break;
        case 1:
          SW_On(&shower1_sw);
          SW_Off(&shower2_sw);
          break;
        }
        if (cmd.param6 == 1) { // clear history and force use PID
            FF_Clear(&cold_ff_state);
        }
        PID_Disable(&cold_valve_pid, 0);
        PID_Disable(&hot_valve_pid, 0);
        snprintf(msg, sizeof(msg), "HOTFLOW: STARTED");
        SERIAL_TransmitNow((uint8_t*)msg, strlen(msg));
        STEPPER_Home(&cold_valve);
        STEPPER_Home(&hot_valve);
        while(!STEPPER_Homed(&cold_valve) || !STEPPER_Homed(&hot_valve)) {}
        snprintf(msg, sizeof(msg), "HOTFLOW: HOMED");

        SW_On(&cold_sw);
        saved_steps_cold = FF_GetSteps(&cold_ff_state, target_flow_cold);
        if (saved_steps_cold == 0) { // no info yet, lets use PID
          PID_Setpoint(&cold_valve_pid, target_flow_cold, 0);
          PID_Enable(&cold_valve_pid);
          while(0xff == (cold_pid_status = PID_Stable(&cold_valve_pid))) {}
          if (cold_pid_status == 0) {
            PID_Disable(&cold_valve_pid, 0);
            snprintf(msg, sizeof(msg), "HOTFLOW: COLD_OK pos=%u pid=true flow=%u", LIMITS_GetPos(&cold_valve_limits_state), cold_valve_pid.flow);
            SERIAL_TransmitNow((uint8_t*)msg, strlen(msg));
            // save historic data
            FF_SaveInHistory(&cold_ff_state, target_flow_cold, cold_valve_pid.flow, LIMITS_GetPos(&cold_valve_limits_state));
          } else {
            snprintf(msg, sizeof(msg), "HOTFLOW: COLD_PID_ERROR pos=%u pid=true flow=%u", LIMITS_GetPos(&cold_valve_limits_state), cold_valve_pid.flow);
            SERIAL_TransmitNow((uint8_t*)msg, strlen(msg));
            SW_Off(&cold_sw);
            SW_Off(&hot_sw);
            SW_Off(&shower1_sw);
            SW_Off(&shower2_sw);
            PID_Disable(&cold_valve_pid, 0);
            PID_Disable(&hot_valve_pid, 0);
            break;
          }
        } else {
          STEPPER_UpdateMovement(&cold_valve, GPIO_PIN_SET, saved_steps_cold);
          while(cold_valve.next_steps_remaining != 0) {}
          // wait for next measurement
          DELAY_US(350000);
          // correcting historic data
          FF_SaveInHistory(&cold_ff_state, target_flow_cold, cold_valve_pid.flow, saved_steps_cold);
          snprintf(msg, sizeof(msg), "HOTFLOW: COLD_OK pos=%u pid=false flow=%u", LIMITS_GetPos(&cold_valve_limits_state), cold_valve_pid.flow);
          SERIAL_TransmitNow((uint8_t*)msg, strlen(msg));
        }
        // add hot flow, always use PID
        SW_On(&hot_sw);
        target_flow_hot = FLOW_GetHotFlow(target_flow_cold, target_temp, STATS_GetMaxTemperature(&main_stats));
        PID_Setpoint(&hot_valve_pid, target_flow_hot, target_flow_cold);
        PID_Enable(&hot_valve_pid);
        while(0xff == (hot_pid_status = PID_Stable(&hot_valve_pid))) {}
        if (hot_pid_status == 0) {
          PID_Disable(&hot_valve_pid, 0);          
          // save historic data
          FF_SaveInHistory(&hot_ff_state, target_flow_cold + target_flow_hot, hot_valve_pid.flow, LIMITS_GetPos(&hot_valve_limits_state));
          snprintf(msg, sizeof(msg), "HOTFLOW: HOT_OK pos=%u pid=true flow=%u", LIMITS_GetPos(&hot_valve_limits_state), hot_valve_pid.flow);
          SERIAL_TransmitNow((uint8_t*)msg, strlen(msg));
        } else {
          snprintf(msg, sizeof(msg), "HOTFLOW: HOT_PID_ERROR pos=%u pid=true flow=%u", LIMITS_GetPos(&hot_valve_limits_state), hot_valve_pid.flow);
          SERIAL_TransmitNow((uint8_t*)msg, strlen(msg));
          SW_Off(&cold_sw);
          SW_Off(&hot_sw);
          SW_Off(&shower1_sw);
          SW_Off(&shower2_sw);
          PID_Disable(&cold_valve_pid, 0);
          PID_Disable(&hot_valve_pid, 0);
        }
        break;
      case 3: // pure feedforward flow warm shower (in cases we cannot allow water flowing ot measure it!)
        SW_Off(&cold_sw);
        SW_Off(&hot_sw);        
        SW_Off(&shower1_sw);
        SW_Off(&shower2_sw);        
        target_flow_cold = ((uint16_t)cmd.param1 << 8) | cmd.param2; // cold water flow baseline        
        target_temp = ((uint16_t)cmd.param3 << 8) | cmd.param4;
        STATS_SetTargetTemp(&main_stats, 0); // we don't correct max temp because this flow is problematic and inprecise!
        PID_Disable(&cold_valve_pid, 0);
        PID_Disable(&hot_valve_pid, 0);
        snprintf(msg, sizeof(msg), "FFFLOW: STARTED");
        SERIAL_TransmitNow((uint8_t*)msg, strlen(msg));
        STEPPER_Home(&cold_valve);
        STEPPER_Home(&hot_valve);
        while(!STEPPER_Homed(&cold_valve) || !STEPPER_Homed(&hot_valve)) {}
        snprintf(msg, sizeof(msg), "FFFLOW: HOMED");
        SERIAL_TransmitNow((uint8_t*)msg, strlen(msg));
        SW_On(&cold_sw);
        SW_On(&hot_sw);        
        saved_steps_cold = FF_GetSteps(&cold_ff_state, target_flow_cold);
        target_flow_hot = FLOW_GetHotFlow(target_flow_cold, target_temp, STATS_GetMaxTemperature(&main_stats));
        saved_steps_hot = FF_GetSteps(&hot_ff_state, target_flow_cold + target_flow_hot);
        if (saved_steps_cold == 0 || saved_steps_hot == 0 ) { // no info yet, fail this flow
          SW_Off(&cold_sw);
          SW_Off(&hot_sw);
          snprintf(msg, sizeof(msg), "FFFLOW: NO_HISTORY");
          SERIAL_TransmitNow((uint8_t*)msg, strlen(msg));          
          break;
        }
        STEPPER_UpdateMovement(&cold_valve, GPIO_PIN_SET, saved_steps_cold);
        STEPPER_UpdateMovement(&hot_valve, GPIO_PIN_SET, saved_steps_hot);
        while(cold_valve.next_steps_remaining != 0 ||
              hot_valve.next_steps_remaining != 0) {}
        snprintf(msg, sizeof(msg), "FFFLOW: OK cold_pos=%u hot_pos=%u pid=false flow=%u", LIMITS_GetPos(&cold_valve_limits_state), LIMITS_GetPos(&hot_valve_limits_state), hot_valve_pid.flow);
        SERIAL_TransmitNow((uint8_t*)msg, strlen(msg));
        break;
      case 4:
        // safety, turn off all
        SW_Off(&cold_sw);
        SW_Off(&hot_sw);
        STATS_SetTargetTemp(&main_stats, 0);
        target_pos_cold = (((uint16_t)cmd.param1 << 8) | cmd.param2);
        target_pos_hot = (((uint16_t)cmd.param3 << 8) | cmd.param4);        
        // allow to turn off valve, because water is not compressable!
        switch(cmd.param5) {
        case 0:
          SW_On(&shower1_sw);
          SW_Off(&shower2_sw);
          break;
        case 1:
          SW_On(&shower1_sw);
          SW_Off(&shower2_sw);
          break;
        }
        PID_Disable(&cold_valve_pid, 0);
        PID_Disable(&hot_valve_pid, 0);
        snprintf(msg, sizeof(msg), "POSFLOW: STARTED");
        SERIAL_TransmitNow((uint8_t*)msg, strlen(msg));
        STEPPER_Home(&cold_valve);
        STEPPER_Home(&hot_valve);
        while(!STEPPER_Homed(&cold_valve) || !STEPPER_Homed(&hot_valve)) {}
        snprintf(msg, sizeof(msg), "POSFLOW: HOMED");        
        SERIAL_TransmitNow((uint8_t*)msg, strlen(msg));        
        // turn water only if have some pos
        if (target_pos_cold > 0) {
          SW_On(&cold_sw);
        }
        if (target_pos_hot > 0) {        
          SW_On(&hot_sw);
        }
        STEPPER_UpdateMovement(&cold_valve, GPIO_PIN_SET, target_pos_cold);
        STEPPER_UpdateMovement(&hot_valve, GPIO_PIN_SET, target_pos_hot);
        while(cold_valve.next_steps_remaining != 0 ||
              hot_valve.next_steps_remaining != 0) {}
        snprintf(msg, sizeof(msg), "POSFLOW: OK");
        SERIAL_TransmitNow((uint8_t*)msg, strlen(msg));
        break;
      case 10: // get current pos
        snprintf(msg, sizeof(msg), "GETPOS: OK cold_pos=%u hot_pos=%u", LIMITS_GetPos(&cold_valve_limits_state), LIMITS_GetPos(&hot_valve_limits_state));
        SERIAL_TransmitNow((uint8_t*)msg, strlen(msg));
        break;                
      case 11: // get current flow
        snprintf(msg, sizeof(msg), "GETFLOW: OK flow=%u", cold_valve_pid.flow);
        SERIAL_TransmitNow((uint8_t*)msg, strlen(msg));
        break;
      case 12: // get current temp
        snprintf(msg, sizeof(msg), "GETTEMP: OK temp=%u temp_c=%u", main_stats.current_temperature, main_stats.current_temperature/16);
        SERIAL_TransmitNow((uint8_t*)msg, strlen(msg));
        break;
      case 55:
        // call reset
        snprintf(msg, sizeof(msg), "RESET: OK crashed=0");
        SERIAL_LL_TransmitNow((uint8_t*)msg, strlen(msg));
        // Trigger actual MCU reset
        NVIC_SystemReset();
        break; // never reaches this
      case 77: // cleanup
        STATS_SetTargetTemp(&main_stats, 0);        
        // safety, turn off all
        SW_Off(&cold_sw);
        SW_Off(&hot_sw);
        SW_Off(&shower1_sw);
        SW_Off(&shower2_sw);
        PID_Disable(&cold_valve_pid, 0);
        PID_Disable(&hot_valve_pid, 0);
        snprintf(msg, sizeof(msg), "CLEANUP: STARTED");
        SERIAL_TransmitNow((uint8_t*)msg, strlen(msg));
        STEPPER_Home(&cold_valve);
        STEPPER_Home(&hot_valve);
        while(!STEPPER_Homed(&cold_valve) || !STEPPER_Homed(&hot_valve)) {}
        snprintf(msg, sizeof(msg), "CLEANUP: HOMED");
        SERIAL_TransmitNow((uint8_t*)msg, strlen(msg));
        snprintf(msg, sizeof(msg), "CLEANUP: OPEN");
        SERIAL_TransmitNow((uint8_t*)msg, strlen(msg));
        while(cold_valve.next_steps_remaining != 0 ||
              hot_valve.next_steps_remaining != 0) {}
        STEPPER_UpdateMovement(&cold_valve, GPIO_PIN_SET, 3000);
        STEPPER_UpdateMovement(&hot_valve, GPIO_PIN_SET, 3000);
        while(cold_valve.next_steps_remaining != 0 ||
              hot_valve.next_steps_remaining != 0) {}
        snprintf(msg, sizeof(msg), "CLEANUP: OPENED");
        SERIAL_TransmitNow((uint8_t*)msg, strlen(msg));
        STEPPER_Home(&cold_valve);
        STEPPER_Home(&hot_valve);
        while(!STEPPER_Homed(&cold_valve) || !STEPPER_Homed(&hot_valve)) {}
        snprintf(msg, sizeof(msg), "CLEANUP: REHOMED");
        SERIAL_TransmitNow((uint8_t*)msg, strlen(msg));        
        snprintf(msg, sizeof(msg), "CLEANUP: OK");
        SERIAL_TransmitNow((uint8_t*)msg, strlen(msg));
        break;
      case 88: // fully calibrate
        STATS_SetTargetTemp(&main_stats, 0);        
        // safety, turn off all
        SW_Off(&cold_sw);
        SW_Off(&hot_sw);
        SW_Off(&shower1_sw);
        SW_Off(&shower2_sw);
        PID_Disable(&cold_valve_pid, 0);
        PID_Disable(&hot_valve_pid, 0);      
        snprintf(msg, sizeof(msg), "CALIBRATION: STARTED");
        SERIAL_TransmitNow((uint8_t*)msg, strlen(msg));
        // reinit state, forcing full_calibration
        LIMITS_Init(&cold_valve_limits_state, 1);
        LIMITS_Init(&hot_valve_limits_state, 1);  
        STEPPER_Home(&cold_valve);
        STEPPER_Home(&hot_valve);
        while(!STEPPER_Homed(&cold_valve) || !STEPPER_Homed(&hot_valve)) {}
        snprintf(msg, sizeof(msg), "CALIBRATION: HOMED");      
        SERIAL_TransmitNow((uint8_t*)msg, strlen(msg));            
        snprintf(msg, sizeof(msg), "CALIBRATION: OK");
        SERIAL_TransmitNow((uint8_t*)msg, strlen(msg));      
        break;
      case 99: // dump state
        snprintf(msg, sizeof(msg), "DUMP: file:%s:%d", __FILE__, __LINE__);
        SERIAL_TransmitNow((uint8_t*)msg, strlen(msg));
        DS18B20_DumpState(msg, sizeof(msg));
        SERIAL_TransmitNow((uint8_t*)msg, strlen(msg));
        EEPROM_DumpState(msg, sizeof(msg));
        SERIAL_TransmitNow((uint8_t*)msg, strlen(msg));
        FLOW_DumpState(msg, sizeof(msg));
        SERIAL_TransmitNow((uint8_t*)msg, strlen(msg));
        LIMITS_DumpState(&cold_valve_limits_state, msg, sizeof(msg));
        SERIAL_TransmitNow((uint8_t*)msg, strlen(msg));
        LIMITS_DumpState(&hot_valve_limits_state, msg, sizeof(msg));
        SERIAL_TransmitNow((uint8_t*)msg, strlen(msg));
        PID_DumpState(&cold_valve_pid, msg, sizeof(msg));
        SERIAL_TransmitNow((uint8_t*)msg, strlen(msg));
        PID_DumpState(&hot_valve_pid, msg, sizeof(msg));
        SERIAL_TransmitNow((uint8_t*)msg, strlen(msg));
        SERIAL_DumpState(msg, sizeof(msg));
        SERIAL_TransmitNow((uint8_t*)msg, strlen(msg));
        STEPPER_DumpState(&cold_valve, msg, sizeof(msg));
        SERIAL_TransmitNow((uint8_t*)msg, strlen(msg));
        STEPPER_DumpState(&hot_valve, msg, sizeof(msg));
        SERIAL_TransmitNow((uint8_t*)msg, strlen(msg));
        FF_DumpState(&cold_ff_state, msg, sizeof(msg));
        SERIAL_TransmitNow((uint8_t*)msg, strlen(msg));
        FF_DumpState(&hot_ff_state, msg, sizeof(msg));
        SERIAL_TransmitNow((uint8_t*)msg, strlen(msg));        
        STATS_DumpState(&main_stats, msg, sizeof(msg));
        SERIAL_TransmitNow((uint8_t*)msg, strlen(msg));        
        SW_DumpState(&cold_sw, msg, sizeof(msg));
        SERIAL_TransmitNow((uint8_t*)msg, strlen(msg));
        SW_DumpState(&hot_sw, msg, sizeof(msg));
        SERIAL_TransmitNow((uint8_t*)msg, strlen(msg));
        SW_DumpState(&shower1_sw, msg, sizeof(msg));
        SERIAL_TransmitNow((uint8_t*)msg, strlen(msg));
        SW_DumpState(&shower2_sw, msg, sizeof(msg));
        SERIAL_TransmitNow((uint8_t*)msg, strlen(msg));      
        snprintf(msg, sizeof(msg), "DUMP: OK");
        SERIAL_TransmitNow((uint8_t*)msg, strlen(msg));      
        break;
      }
    }
  }
}

/**
* @brief System Clock Configuration
* @retval None
*/
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};
  RCC_PeriphCLKInitTypeDef PeriphClkInit = {0};
  
  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL3;
  RCC_OscInitStruct.PLL.PREDIV = RCC_PREDIV_DIV1;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler(__FILE__,__LINE__);
  }
  
  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
    |RCC_CLOCKTYPE_PCLK1;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  
  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_1) != HAL_OK)
  {
    Error_Handler(__FILE__,__LINE__);
  }
  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_USART1;
  PeriphClkInit.Usart1ClockSelection = RCC_USART1CLKSOURCE_PCLK1;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
  {
    Error_Handler(__FILE__,__LINE__);
  }
}

/**
* @brief  This function is executed in case of error occurrence.
* @retval None
*/
void Error_Handler(const char *file, int line)
{
  __disable_irq();
  // safety, turn off all
  SW_Off(&cold_sw);
  SW_Off(&hot_sw);
  SW_Off(&shower1_sw);
  SW_Off(&shower2_sw);
  STEPPER_Disable(&cold_valve);
  STEPPER_Disable(&hot_valve);
  
  SERIAL_LL_Init();
  while (1)
  {
    if (SERIAL_LL_WaitReset(5000) == 0x01) {
      DELAY_US(10000);
      // call reset
      snprintf(msg, sizeof(msg), "RESET: OK crashed=1");
      SERIAL_LL_TransmitNow((uint8_t*)msg, strlen(msg));
      // Trigger actual MCU reset
      NVIC_SystemReset();      
    }
    // --- Debug messages ---
    snprintf(msg, sizeof(msg), "CRASH file:%s:%d", file, line);
    SERIAL_LL_TransmitNow((uint8_t*)msg, strlen(msg));
    DS18B20_DumpState(msg, sizeof(msg));
    SERIAL_LL_TransmitNow((uint8_t*)msg, strlen(msg));
    EEPROM_DumpState(msg, sizeof(msg));
    SERIAL_LL_TransmitNow((uint8_t*)msg, strlen(msg));
    FLOW_DumpState(msg, sizeof(msg));
    SERIAL_LL_TransmitNow((uint8_t*)msg, strlen(msg));
    LIMITS_DumpState(&cold_valve_limits_state, msg, sizeof(msg));
    SERIAL_LL_TransmitNow((uint8_t*)msg, strlen(msg));
    LIMITS_DumpState(&hot_valve_limits_state, msg, sizeof(msg));
    SERIAL_LL_TransmitNow((uint8_t*)msg, strlen(msg));
    PID_DumpState(&cold_valve_pid, msg, sizeof(msg));
    SERIAL_LL_TransmitNow((uint8_t*)msg, strlen(msg));
    PID_DumpState(&hot_valve_pid, msg, sizeof(msg));
    SERIAL_LL_TransmitNow((uint8_t*)msg, strlen(msg));
    SERIAL_DumpState(msg, sizeof(msg));
    SERIAL_LL_TransmitNow((uint8_t*)msg, strlen(msg));
    STEPPER_DumpState(&cold_valve, msg, sizeof(msg));
    SERIAL_LL_TransmitNow((uint8_t*)msg, strlen(msg));
    STEPPER_DumpState(&hot_valve, msg, sizeof(msg));
    SERIAL_LL_TransmitNow((uint8_t*)msg, strlen(msg));
    FF_DumpState(&cold_ff_state, msg, sizeof(msg));
    SERIAL_LL_TransmitNow((uint8_t*)msg, strlen(msg));
    FF_DumpState(&hot_ff_state, msg, sizeof(msg));
    SERIAL_LL_TransmitNow((uint8_t*)msg, strlen(msg));
    STATS_DumpState(&main_stats, msg, sizeof(msg));
    SERIAL_LL_TransmitNow((uint8_t*)msg, strlen(msg));
    SW_DumpState(&cold_sw, msg, sizeof(msg));
    SERIAL_LL_TransmitNow((uint8_t*)msg, strlen(msg));
    SW_DumpState(&hot_sw, msg, sizeof(msg));
    SERIAL_LL_TransmitNow((uint8_t*)msg, strlen(msg));
    SW_DumpState(&shower1_sw, msg, sizeof(msg));
    SERIAL_LL_TransmitNow((uint8_t*)msg, strlen(msg));
    SW_DumpState(&shower2_sw, msg, sizeof(msg));
    SERIAL_LL_TransmitNow((uint8_t*)msg, strlen(msg));    
  }
}

#ifdef USE_FULL_ASSERT
/**
* @brief  Reports the name of the source file and the source line number
*         where the assert_param error has occurred.
* @param  file: pointer to the source file name
* @param  line: assert_param error line source number
* @retval None
*/
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
  ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
