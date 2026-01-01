#include <string.h>
#include <stdio.h>

#include "ds18b20.h"
#include "limit.h"
#include "stm32f0xx_hal.h"

// --- External handles from CubeMX ---
extern UART_HandleTypeDef huart2;   // USART2 half-duplex
extern TIM_HandleTypeDef htim15;    // TIM15 1ms timer

// --- Resolution configuration ---
#define DS18B20_RESOLUTION 10   // 9..12 bits
static const uint16_t conv_time_table[4] = {94, 188, 375, 750};
#define DS18B20_CONV_MS conv_time_table[DS18B20_RESOLUTION - 9]
static const uint8_t ds_config_byte_table[4] = {0x1F, 0x3F, 0x5F, 0x7F};
#define DS18B20_CONFIG_BYTE ds_config_byte_table[DS18B20_RESOLUTION - 9]

// Alarm registers (optional, for WRITE SCRATCHPAD)
#define DS18B20_TH 0x7F
#define DS18B20_TL 0x80

// --- Timeout configuration ---
#define ASYNC_TIMEOUT_MAX 10   // 10 ms per async UART operation
static volatile uint16_t async_timeout = 0;

// --- FSM states ---
typedef enum {
    TEMP_IDLE,
    TEMP_RESET1,
    TEMP_SKIP_ROM1,
    TEMP_WRITE_TH,
    TEMP_WRITE_TL,
    TEMP_WRITE_CONFIG,
    TEMP_RESET_CONVERT,
    TEMP_SKIP_ROM_CONVERT,
    TEMP_SEND_CONVERT_T,
    TEMP_WAIT_CONVERT,
    TEMP_RESET2,
    TEMP_SKIP_ROM2,
    TEMP_READ_CMD,
    TEMP_READ_LSB,
    TEMP_READ_MSB
} TempState_t;

// --- Exposed values ---
static volatile TempState_t temp_state = TEMP_IDLE;
static volatile uint16_t temp_value = 0;
static volatile uint8_t temp_fsm_active = 0; // 1 = running
static volatile TempStatus_t temp_status = TEMP_OK;
static volatile uint16_t conv_ticks_remaining = 0;

// --- UART 1-Wire states ---
typedef enum {
    DS_STATE_IDLE,
    DS_STATE_RESET_SEND,
    DS_STATE_RESET_RX,
    DS_STATE_WRITE_SEND,
    DS_STATE_WRITE_RX,
    DS_STATE_READ_SEND,
    DS_STATE_READ_RX
} DS_State_t;

static volatile DS_State_t ds_state = DS_STATE_IDLE;
static uint8_t ds_txbuf, ds_rxbuf, ds_byte, ds_bitIndex;
static volatile uint8_t ds_result, ds_done;
typedef enum { OP_NONE, OP_RESET, OP_WRITE_BYTE, OP_READ_BYTE } DS_Op_t;
static volatile DS_Op_t current_op = OP_NONE;

void DS18B20_DumpState(char* buff, int len) {
    snprintf(buff, len,
        "DS18B20 temp_state=%u ds_state=%u current_op=%u temp_value=%u fsm_active=%u conv_ticks_remaining=%u "
        "ds_txbuf=%u ds_rxbuf=%u ds_byte=%u ds_bitIndex=%u ds_result=%u ds_done=%u",
        temp_state,
        ds_state,
        current_op,
        temp_value,
        temp_fsm_active,
        conv_ticks_remaining,
        ds_txbuf,
        ds_rxbuf,
        ds_byte,
        ds_bitIndex,
        ds_result,
        ds_done
    );
}

// --- UART helper ---
static void set_baud(uint32_t baud)
{
    huart2.Init.BaudRate = baud;
    if (HAL_HalfDuplex_Init(&huart2) != HAL_OK)
        Error_Handler(__FILE__,__LINE__);
}

// --- Async 1-Wire ops ---
static void reset_start(void)
{
    ds_result = 0;
    ds_done = 0;
    ds_bitIndex = 0;
    current_op = OP_RESET;
    ds_state = DS_STATE_RESET_SEND;
    async_timeout = ASYNC_TIMEOUT_MAX;
    set_baud(9600);
    ds_txbuf = 0xF0;
    if (HAL_UART_Transmit_IT(&huart2, &ds_txbuf, 1) != HAL_OK)
        Error_Handler(__FILE__,__LINE__);
}

static uint8_t reset_done(void)
{
    if (ds_done)
        async_timeout = 0;
    return ds_done;
}

static uint8_t reset_presence(void)
{
    return ds_result;
}

static void write_start(uint8_t byte)
{
    ds_byte = byte;
    ds_bitIndex = 0;
    ds_done = 0;
    current_op = OP_WRITE_BYTE;
    ds_state = DS_STATE_WRITE_SEND;
    async_timeout = ASYNC_TIMEOUT_MAX;
    set_baud(115200);
    ds_txbuf = (ds_byte & (1u << ds_bitIndex)) ? 0xFF : 0x00;
    if (HAL_UART_Transmit_IT(&huart2, &ds_txbuf, 1) != HAL_OK)
        Error_Handler(__FILE__,__LINE__);
}

static uint8_t write_done(void)
{
    if (ds_done)
        async_timeout = 0;
    return ds_done;
}

static void read_start(void)
{
    ds_bitIndex = 0;
    ds_result = 0;
    ds_done = 0;
    current_op = OP_READ_BYTE;
    ds_state = DS_STATE_READ_SEND;
    async_timeout = ASYNC_TIMEOUT_MAX;
    set_baud(115200);
    ds_txbuf = 0xFF;
    if (HAL_UART_Transmit_IT(&huart2, &ds_txbuf, 1) != HAL_OK)
        Error_Handler(__FILE__,__LINE__);
}

static uint8_t read_done(void)
{
    if (ds_done)
        async_timeout = 0;
    return ds_done;
}

static uint8_t read_get(void)
{
    return ds_result;
}

/* =====================================================================
 * UART ISR Callbacks
 * ===================================================================== */
void DS18B20_TxCplt(void)
{
    switch (ds_state)
    {
        case DS_STATE_RESET_SEND:
            ds_state = DS_STATE_RESET_RX;
            if (HAL_UART_Receive_IT(&huart2, &ds_rxbuf, 1) != HAL_OK)
                Error_Handler(__FILE__,__LINE__);
            break;

        case DS_STATE_WRITE_SEND:
            ds_state = DS_STATE_WRITE_RX;
            if (HAL_UART_Receive_IT(&huart2, &ds_rxbuf, 1) != HAL_OK)
                Error_Handler(__FILE__,__LINE__);
            break;

        case DS_STATE_READ_SEND:
            ds_state = DS_STATE_READ_RX;
            if (HAL_UART_Receive_IT(&huart2, &ds_rxbuf, 1) != HAL_OK)
                Error_Handler(__FILE__,__LINE__);
            break;

        default:
            break;
    }
}

void DS18B20_RxCplt(void)
{
    switch (ds_state)
    {
        case DS_STATE_RESET_RX:
            ds_result = (ds_rxbuf != 0xF0);
            ds_done = 1;
            set_baud(115200);
            ds_state = DS_STATE_IDLE;
            current_op = OP_NONE;
            break;

        case DS_STATE_WRITE_RX:
            if (ds_rxbuf == 0xFF)
                ds_result |= (1u << ds_bitIndex);
            ds_bitIndex++;

            if (ds_bitIndex < 8)
            {
                ds_txbuf = (ds_byte & (1u << ds_bitIndex)) ? 0xFF : 0x00;
                ds_state = DS_STATE_WRITE_SEND;
                if (HAL_UART_Transmit_IT(&huart2, &ds_txbuf, 1) != HAL_OK)
                    Error_Handler(__FILE__,__LINE__);
            }
            else
            {
                ds_done = 1;
                ds_state = DS_STATE_IDLE;
                current_op = OP_NONE;
            }
            break;

        case DS_STATE_READ_RX:
            if (ds_rxbuf == 0xFF)
                ds_result |= (1u << ds_bitIndex);
            ds_bitIndex++;

            if (ds_bitIndex < 8)
            {
                ds_txbuf = 0xFF;
                ds_state = DS_STATE_READ_SEND;
                if (HAL_UART_Transmit_IT(&huart2, &ds_txbuf, 1) != HAL_OK)
                    Error_Handler(__FILE__,__LINE__);
            }
            else
            {
                ds_done = 1;
                ds_state = DS_STATE_IDLE;
                current_op = OP_NONE;
            }
            break;

        default:
            break;
    }
}

/* =====================================================================
 * 1ms Tick FSM
 * ===================================================================== */
void DS18B20_Tick(void)
{
    if (DS18B20_Ready())
        return;

    if (async_timeout > 0)
    {
        async_timeout--;
        if (async_timeout == 0)
        {
            temp_status = TEMP_ERROR;
            temp_fsm_active = 0;
            return;
        }
    }

    switch (temp_state)
    {
        case TEMP_IDLE: break;

        case TEMP_RESET1:
            if (reset_done())
            {
                if (!reset_presence())
                {
                    temp_status = TEMP_ERROR;
                    temp_fsm_active = 0;
                }
                else
                {
                    write_start(0xCC); // Skip ROM
                    temp_state = TEMP_SKIP_ROM1;
                }
            }
            break;

        case TEMP_SKIP_ROM1:
            if (write_done())
            {
                write_start(0x4E); // Write Scratchpad
                temp_state = TEMP_WRITE_TH;
            }
            break;

        case TEMP_WRITE_TH:
            if (write_done())
            {
                write_start(DS18B20_TH);
                temp_state = TEMP_WRITE_TL;
            }
            break;

        case TEMP_WRITE_TL:
            if (write_done())
            {
                write_start(DS18B20_CONFIG_BYTE);
                temp_state = TEMP_WRITE_CONFIG;
            }
            break;

        case TEMP_WRITE_CONFIG:
            if (write_done())
            {
                // Done writing config, now trigger conversion
                reset_start();
                temp_state = TEMP_RESET_CONVERT;
            }
            break;

        case TEMP_RESET_CONVERT:
            if (reset_done())
            {
                if (!reset_presence())
                {
                    temp_status = TEMP_ERROR;
                    temp_fsm_active = 0;
                }
                else
                {
                    write_start(0xCC); // Skip ROM
                    temp_state = TEMP_SKIP_ROM_CONVERT;
                }
            }
            break;

        case TEMP_SKIP_ROM_CONVERT:
            if (write_done())
            {
                write_start(0x44); // Convert T
                temp_state = TEMP_SEND_CONVERT_T;
            }
            break;

        case TEMP_SEND_CONVERT_T:
            if (write_done())
            {
                conv_ticks_remaining = DS18B20_CONV_MS;
                temp_state = TEMP_WAIT_CONVERT;
            }
            break;

        case TEMP_WAIT_CONVERT:
            if (conv_ticks_remaining > 0)
                conv_ticks_remaining--;
            else
            {
                reset_start();
                temp_state = TEMP_RESET2;
            }
            break;

        case TEMP_RESET2:
            if (reset_done())
            {
                if (!reset_presence())
                {
                    temp_status = TEMP_ERROR;
                    temp_fsm_active = 0;
                }
                else
                {
                    write_start(0xCC); // Skip ROM
                    temp_state = TEMP_SKIP_ROM2;
                }
            }
            break;

        case TEMP_SKIP_ROM2:
            if (write_done())
            {
                write_start(0xBE); // Read Scratchpad
                temp_state = TEMP_READ_CMD;
            }
            break;

        case TEMP_READ_CMD:
            if (write_done())
            {
                read_start();
                temp_state = TEMP_READ_LSB;
            }
            break;

        case TEMP_READ_LSB:
            if (read_done())
            {
                temp_value = read_get();
                read_start();
                temp_state = TEMP_READ_MSB;
            }
            break;

        case TEMP_READ_MSB:
            if (read_done())
            {
                temp_value |= (read_get() << 8);
                temp_status = TEMP_OK;
                temp_fsm_active = 0;
            }
            break;

        default:
            temp_status = TEMP_ERROR;
            temp_fsm_active = 0;
            break;
    }
}

/* =====================================================================
 * Public API
 * ===================================================================== */
void DS18B20_StartTempRead(void)
{
    temp_state = TEMP_RESET1;
    temp_status = TEMP_OK;
    temp_value = 0;
    temp_fsm_active = 1;
    reset_start();
}

int16_t DS18B20_GetTemp(void) { return (int16_t)temp_value; }

uint8_t DS18B20_Ready(void) { return !temp_fsm_active; }

TempStatus_t DS18B20_Status(void) { return temp_status; }

void DS18B20_Init(void)
{
    set_baud(115200);
    ds_state = DS_STATE_IDLE;
    current_op = OP_NONE;
    ds_done = 0;
    ds_result = 0;
    temp_state = TEMP_IDLE;
    temp_fsm_active = 0;
    conv_ticks_remaining = 0;
    async_timeout = 0;
    temp_status = TEMP_OK;
}
