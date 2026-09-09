#ifndef __CAN_UART_GATEWAY_H__
#define __CAN_UART_GATEWAY_H__

#ifdef __cplusplus
extern "C" {
#endif

#include "main.h"

/*
 * UART wire format (all multi-byte fields are little-endian):
 * AA 55 | BODY_LEN | SEQ(2) | CAN_ID(4) | FLAGS | DLC | DATA(0..64)
 *       | CRC8-ATM | 55 AA
 * BODY_LEN = 8 + DLC. CRC covers BODY_LEN through the final DATA byte.
 * FLAGS: bit0=extended ID, bit1=CAN FD, bit2=BRS, bit3=remote frame.
 */
HAL_StatusTypeDef CanUartGateway_Init(void);
void CanUartGateway_Process(void);

#ifdef __cplusplus
}
#endif

#endif /* __CAN_UART_GATEWAY_H__ */
