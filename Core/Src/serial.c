#include <stdio.h>
#include <string.h>
#include "stm32f0xx_ll_rcc.h"
#include "stm32f0xx_ll_usart.h"
#include "stm32f0xx_ll_cortex.h"
#include "stm32f0xx_ll_system.h"
#include "stm32f0xx_ll_bus.h"
#include "stm32f0xx_ll_gpio.h"
#include "stm32f0xx_hal.h"
#include "serial.h"

/* External HAL handles created by CubeMX */
extern UART_HandleTypeDef huart1;
extern DMA_HandleTypeDef hdma_usart1_rx;
//extern DMA_HandleTypeDef hdma_usart1_tx;

/* RX circular buffer */
static uint8_t rx_buf[SERIAL_RX_BUF_SIZE];

/* TX scratch buffer and state */
static uint8_t tx_frame_buf[SERIAL_TX_MAX_FRAME + 4]; /* header + 2-byte len + payload + CRC8 */
static volatile uint8_t tx_busy = 0;

/* helper state */
static volatile uint16_t last_rx_dma_count = 0;

/* Parser state */
static uint8_t parser_state = 0; // 0=wait header, 1=len_hi, 2=len_lo, 3=payload+crc
static uint16_t payload_len = 0;
static uint16_t frame_pos = 0;
static uint16_t expected_len = 0;
static uint8_t frame_buf[SERIAL_RX_BUF_SIZE];

/* --- CRC8 (poly=0x07, lookup table) --- */
static const uint8_t crc8Table[256] = {
    0x00,0x07,0x0e,0x09,0x1c,0x1b,0x12,0x15,
    0x38,0x3f,0x36,0x31,0x24,0x23,0x2a,0x2d,
    0x70,0x77,0x7e,0x79,0x6c,0x6b,0x62,0x65,
    0x48,0x4f,0x46,0x41,0x54,0x53,0x5a,0x5d,
    0xe0,0xe7,0xee,0xe9,0xfc,0xfb,0xf2,0xf5,
    0xd8,0xdf,0xd6,0xd1,0xc4,0xc3,0xca,0xcd,
    0x90,0x97,0x9e,0x99,0x8c,0x8b,0x82,0x85,
    0xa8,0xaf,0xa6,0xa1,0xb4,0xb3,0xba,0xbd,
    0xc7,0xc0,0xc9,0xce,0xdb,0xdc,0xd5,0xd2,
    0xff,0xf8,0xf1,0xf6,0xe3,0xe4,0xed,0xea,
    0xb7,0xb0,0xb9,0xbe,0xab,0xac,0xa5,0xa2,
    0x8f,0x88,0x81,0x86,0x93,0x94,0x9d,0x9a,
    0x27,0x20,0x29,0x2e,0x3b,0x3c,0x35,0x32,
    0x1f,0x18,0x11,0x16,0x03,0x04,0x0d,0x0a,
    0x57,0x50,0x59,0x5e,0x4b,0x4c,0x45,0x42,
    0x6f,0x68,0x61,0x66,0x73,0x74,0x7d,0x7a,
    0x89,0x8e,0x87,0x80,0x95,0x92,0x9b,0x9c,
    0xb1,0xb6,0xbf,0xb8,0xad,0xaa,0xa3,0xa4,
    0xf9,0xfe,0xf7,0xf0,0xe5,0xe2,0xeb,0xec,
    0xc1,0xc6,0xcf,0xc8,0xdd,0xda,0xd3,0xd4,
    0x69,0x6e,0x67,0x60,0x75,0x72,0x7b,0x7c,
    0x51,0x56,0x5f,0x58,0x4d,0x4a,0x43,0x44,
    0x19,0x1e,0x17,0x10,0x05,0x02,0x0b,0x0c,
    0x21,0x26,0x2f,0x28,0x3d,0x3a,0x33,0x34,
    0x4e,0x49,0x40,0x47,0x52,0x55,0x5c,0x5b,
    0x76,0x71,0x78,0x7f,0x6a,0x6d,0x64,0x63,
    0x3e,0x39,0x30,0x37,0x22,0x25,0x2c,0x2b,
    0x06,0x01,0x08,0x0f,0x1a,0x1d,0x14,0x13,
    0xae,0xa9,0xa0,0xa7,0xb2,0xb5,0xbc,0xbb,
    0x96,0x91,0x98,0x9f,0x8a,0x8d,0x84,0x83,
    0xde,0xd9,0xd0,0xd7,0xc2,0xc5,0xcc,0xcb,
    0xe6,0xe1,0xe8,0xef,0xfa,0xfd,0xf4,0xf3
};

static inline uint16_t tim3_ms(void)
{
    return TIM3->CNT;      // returns 0..65535 ms
}

#define TIMEOUT_EXPIRED(start, timeout_ms) ((uint16_t)(tim3_ms() - (start)) >= (timeout_ms))

static uint8_t crc8_calc(const uint8_t *data, size_t len)
{
    uint8_t crc = 0x00;
    for (size_t i = 0; i < len; ++i) {
        crc = crc8Table[crc ^ data[i]];
    }
    return crc;
}

/* --- Initialize UART RX DMA circular --- */
void SERIAL_Init(void)
{
    HAL_UART_AbortReceive(&huart1);
    HAL_UART_Receive_DMA(&huart1, rx_buf, SERIAL_RX_BUF_SIZE);
    __HAL_UART_CLEAR_IDLEFLAG(&huart1);
    __HAL_UART_ENABLE_IT(&huart1, UART_IT_IDLE);
    last_rx_dma_count = hdma_usart1_rx.Instance->CNDTR;
}

/* --- Dump current state --- */
void SERIAL_DumpState(char* buff, int len)
{
    snprintf(buff, len,
        "SERIAL tx_busy=%u last_rx_dma_count=%u parser_state=%u expected_len=%u payload_len=%u frame_pos=%u frame_buf[0]=%u frame_buf[1]=%u frame_buf[2]=%u frame_buf[3]=%u frame_buf[4]=%u frame_buf[5]=%u frame_buf[6]=%u frame_buf[7]=%u frame_buf[8]=%u",
        tx_busy,
        last_rx_dma_count,
        parser_state,
        expected_len,
        payload_len,
        frame_pos,
        frame_buf[0],
        frame_buf[1],
        frame_buf[2],
        frame_buf[3],
        frame_buf[4],
        frame_buf[5],
        frame_buf[6],
        frame_buf[7],
        frame_buf[8]
    );
}

/* --- Transmit a frame (2-byte length) --- */
int SERIAL_Transmit(const uint8_t *payload, uint16_t payload_len)
{
    if (payload_len > SERIAL_TX_MAX_FRAME) return -1;
    if (tx_busy) return -1;

    uint16_t frame_len = 0;
    tx_frame_buf[frame_len++] = OUR_ADDR;
    tx_frame_buf[frame_len++] = (payload_len >> 8) & 0xFF; // LEN_H
    tx_frame_buf[frame_len++] = payload_len & 0xFF;        // LEN_L

    if (payload_len) {
        memcpy(&tx_frame_buf[frame_len], payload, payload_len);
        frame_len += payload_len;
    }

    uint8_t crc = crc8_calc(tx_frame_buf, frame_len);
    tx_frame_buf[frame_len++] = crc;

    tx_busy = 1;
    if (HAL_UART_Transmit_DMA(&huart1, tx_frame_buf, frame_len) != HAL_OK) {
        tx_busy = 0;
        return -1;
    }
    return 0;
}

/* --- Transmit fatal error synchronously --- */
int SERIAL_TransmitNow(const uint8_t *payload, uint16_t payload_len)
{
    if (payload_len > SERIAL_TX_MAX_FRAME) return -1;

    uint16_t frame_len = 0;
    tx_frame_buf[frame_len++] = OUR_ADDR;
    tx_frame_buf[frame_len++] = (payload_len >> 8) & 0xFF;
    tx_frame_buf[frame_len++] = payload_len & 0xFF;

    if (payload_len) {
        memcpy(&tx_frame_buf[frame_len], payload, payload_len);
        frame_len += payload_len;
    }

    uint8_t crc = crc8_calc(tx_frame_buf, frame_len);
    tx_frame_buf[frame_len++] = crc;

    if (HAL_UART_Transmit(&huart1, tx_frame_buf, frame_len, 100) != HAL_OK) {
        return -1;
    }
    return 0;
}

/* TX complete callback */
void SERIAL_TxCplt(void) { tx_busy = 0; }

void SERIAL_RxCplt(void)
{
    //process_rx_data();
}

#define RESET_CMD 55
void SERIAL_LL_Init(void)
{
    /* Stop HAL UART cleanly */
    HAL_UART_Abort(&huart1);

    /* Disable DMA streams */
    HAL_DMA_Abort(&hdma_usart1_rx);
    //HAL_DMA_Abort(&hdma_usart1_tx);

    /* Disable USART IRQ */
    NVIC_DisableIRQ(USART1_IRQn);

    /* Reset USART */
    __HAL_RCC_USART1_FORCE_RESET();
    __HAL_RCC_USART1_RELEASE_RESET();

    /* Enable clocks */
    LL_APB1_GRP2_EnableClock(LL_APB1_GRP2_PERIPH_USART1);
    LL_AHB1_GRP1_EnableClock(LL_AHB1_GRP1_PERIPH_GPIOA);

    /* GPIO PA9 TX, PA10 RX, PA12 DE */
    LL_GPIO_SetPinMode(GPIOA, LL_GPIO_PIN_9,  LL_GPIO_MODE_ALTERNATE);
    LL_GPIO_SetPinMode(GPIOA, LL_GPIO_PIN_10, LL_GPIO_MODE_ALTERNATE);
    LL_GPIO_SetPinMode(GPIOA, LL_GPIO_PIN_12, LL_GPIO_MODE_ALTERNATE); // DE pin

    LL_GPIO_SetAFPin_8_15(GPIOA, LL_GPIO_PIN_9,  LL_GPIO_AF_1);
    LL_GPIO_SetAFPin_8_15(GPIOA, LL_GPIO_PIN_10, LL_GPIO_AF_1);
    LL_GPIO_SetAFPin_8_15(GPIOA, LL_GPIO_PIN_12, LL_GPIO_AF_1); // DE pin AF1

    LL_GPIO_SetPinPull(GPIOA, LL_GPIO_PIN_10, LL_GPIO_PULL_UP); // RX pull-up
    LL_GPIO_SetPinSpeed(GPIOA, LL_GPIO_PIN_9, LL_GPIO_SPEED_FREQ_HIGH);
    LL_GPIO_SetPinSpeed(GPIOA, LL_GPIO_PIN_10, LL_GPIO_SPEED_FREQ_HIGH);
    LL_GPIO_SetPinSpeed(GPIOA, LL_GPIO_PIN_12, LL_GPIO_SPEED_FREQ_HIGH);

    /* USART config */
    LL_USART_Disable(USART1);
    LL_USART_SetTransferDirection(USART1, LL_USART_DIRECTION_TX_RX);
    LL_USART_ConfigCharacter(USART1, LL_USART_DATAWIDTH_8B, LL_USART_PARITY_NONE, LL_USART_STOPBITS_1);
    LL_USART_SetBaudRate(USART1, SystemCoreClock, LL_USART_OVERSAMPLING_16, 115200);

    /* RS-485 automatic DE configuration */
    LL_USART_SetDEAssertionTime(USART1, 0);    // DE asserted before first bit
    LL_USART_SetDEDeassertionTime(USART1, 0);  // DE deasserted after last bit
    LL_USART_SetDESignalPolarity(USART1, LL_USART_DE_POLARITY_HIGH); // DE active high
    LL_USART_EnableDEMode(USART1);

    /* Enable USART */
    LL_USART_Enable(USART1);
}

/* --- Wait for reset command 0x55 with XX sec timeout --- */
uint8_t SERIAL_LL_WaitReset(uint16_t timeout)
{
    uint8_t state = 0;
    uint16_t len = 0, pos = 0;
    uint8_t buf[SERIAL_RX_BUF_SIZE];

    uint16_t start = tim3_ms();

    LL_USART_ClearFlag_ORE(USART1);
    LL_USART_ClearFlag_FE(USART1);
    LL_USART_ClearFlag_NE(USART1);

    while (!TIMEOUT_EXPIRED(start, timeout))
    {
        if (LL_USART_IsActiveFlag_RXNE(USART1))
        {
            uint8_t b = LL_USART_ReceiveData8(USART1);

            switch (state)
            {
            case 0: // wait header
                if (b == OUR_ADDR)
                    state = 1;
                break;
            case 1: // length high
                len = ((uint16_t)b) << 8;
                state = 2;
                break;
            case 2: // length low
                len |= b;
                if (len > SERIAL_RX_BUF_SIZE - 1)
                {
                    state = 0;
                    break;
                }
                pos = 0;
                state = 3;
                break;
            case 3: // payload + CRC
                buf[pos++] = b;
                if (pos >= len + 1) // payload + CRC received
                {
                    uint8_t crc_buf[3 + SERIAL_RX_BUF_SIZE];
                    size_t crc_len = 0;
                    crc_buf[crc_len++] = OUR_ADDR;
                    crc_buf[crc_len++] = (len >> 8) & 0xFF;
                    crc_buf[crc_len++] = len & 0xFF;
                    memcpy(&crc_buf[crc_len], buf, len);
                    crc_len += len;

                    uint8_t rx_crc   = buf[len];
                    uint8_t calc_crc = crc8_calc(crc_buf, crc_len);

                    if (rx_crc == calc_crc && len > 0)
                    {
                        if (buf[0] == RESET_CMD)
                            return 1; // Reset command detected
                    }

                    state = 0; // restart parser
                }
                break;
            }
        }
    }
    return 0; // timeout
}

/* --- Transmit fatal error synchronously --- */
int SERIAL_LL_TransmitNow(const uint8_t *payload, uint16_t payload_len)
{
    uint16_t i;
    uint16_t frame_len = 0;    
    if (payload_len > SERIAL_TX_MAX_FRAME) return -1;

    tx_frame_buf[frame_len++] = OUR_ADDR;
    tx_frame_buf[frame_len++] = (payload_len >> 8) & 0xFF;
    tx_frame_buf[frame_len++] = payload_len & 0xFF;

    if (payload_len) {
        memcpy(&tx_frame_buf[frame_len], payload, payload_len);
        frame_len += payload_len;
    }

    uint8_t crc = crc8_calc(tx_frame_buf, frame_len);
    tx_frame_buf[frame_len++] = crc;
    
    for (i = 0; i < frame_len; i++)
    {
        /* Wait until TX data register empty */
        while (!LL_USART_IsActiveFlag_TXE(USART1)){ }
        /* Send byte */
        LL_USART_TransmitData8(USART1, tx_frame_buf[i]);
    }

    /* Wait for transmission complete */
    while (!LL_USART_IsActiveFlag_TC(USART1)){}
    return 0;
}

/* --- RX parser state --- */
void process_rx_data(void)
{
    uint16_t cndtr = hdma_usart1_rx.Instance->CNDTR;
    uint16_t bytes_received;

    if (last_rx_dma_count >= cndtr)
        bytes_received = last_rx_dma_count - cndtr;
    else
        bytes_received = last_rx_dma_count + (SERIAL_RX_BUF_SIZE - cndtr);

    last_rx_dma_count = cndtr;
    if (bytes_received == 0) return;

    uint16_t index = (SERIAL_RX_BUF_SIZE - last_rx_dma_count - bytes_received) % SERIAL_RX_BUF_SIZE;

    for (uint16_t i = 0; i < bytes_received; ++i) {
        uint8_t b = rx_buf[(index + i) % SERIAL_RX_BUF_SIZE];

        switch (parser_state) {
        case 0: /* wait for header */
            if (b == OUR_ADDR) parser_state = 1;
            break;
        case 1: /* length high byte */
            payload_len = b << 8;
            parser_state = 2;
            break;
        case 2: /* length low byte */
            payload_len |= b;
            if (payload_len > SERIAL_TX_MAX_FRAME) { parser_state = 0; break; }
            frame_pos = 0;
            expected_len = payload_len + 1; /* payload + CRC */
            parser_state = 3;
            break;
        case 3: /* read payload + CRC */
            frame_buf[frame_pos++] = b;
            if (frame_pos >= expected_len) {
                /* Build full frame for CRC */
                uint8_t crc_buf[SERIAL_RX_BUF_SIZE];
                size_t crc_len = 0;
                crc_buf[crc_len++] = OUR_ADDR;
                crc_buf[crc_len++] = (payload_len >> 8) & 0xFF;
                crc_buf[crc_len++] = payload_len & 0xFF;
                if (payload_len) {
                    memcpy(&crc_buf[crc_len], frame_buf, payload_len);
                    crc_len += payload_len;
                }
                uint8_t rx_crc = frame_buf[payload_len];
                uint8_t calc_crc = crc8_calc(crc_buf, crc_len);

                if (rx_crc == calc_crc && payload_len)
                    SERIAL_OnReceive(frame_buf, payload_len);

                parser_state = 0;
                frame_pos = 0;
                expected_len = 0;
                payload_len = 0;
            }
            break;
        }
    }
}

void SERIAL_Poll(void)
{
    process_rx_data();
}

/* --- Called from USART IDLE IRQ --- */
void SERIAL_OnIdle(void)
{
    __HAL_UART_CLEAR_IDLEFLAG(&huart1);
    process_rx_data();
}
