#ifndef __SERIAL_H__
#define __SERIAL_H__

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* Configure these as needed */
#define SERIAL_RX_BUF_SIZE 256
#define SERIAL_TX_MAX_FRAME 512   /* max payload length (choose <= buffer- overhead) */
#define OUR_ADDR 0x55

void SERIAL_DumpState(char* buff, int len);
void SERIAL_Init(void);

/* API */
int SERIAL_Transmit(const uint8_t *payload, uint16_t payload_len); /* returns 0 on started, -1 if busy/err */
int SERIAL_TransmitNow(const uint8_t *payload, uint16_t payload_len); /* returns 0 on started, -1 if busy/err */

void SERIAL_Poll(void); /* optional — call from main loop if you don't want to handle in IRQ */
/* Callbacks you may implement in your app to receive frames */
void SERIAL_OnReceive(const uint8_t *payload, uint16_t len);

void SERIAL_TxCplt(void);
void SERIAL_RxCplt(void);
void SERIAL_OnIdle(void);

void SERIAL_LL_Init(void);
uint8_t SERIAL_LL_WaitReset(uint16_t timeout);
int SERIAL_LL_TransmitNow(const uint8_t *payload, uint16_t payload_len);

#ifdef __cplusplus
}
#endif

#endif /* __SERIAL_H__ */
