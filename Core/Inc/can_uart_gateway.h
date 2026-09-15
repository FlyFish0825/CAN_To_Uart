#ifndef __CAN_UART_GATEWAY_H__
#define __CAN_UART_GATEWAY_H__

#ifdef __cplusplus
extern "C" {
#endif

#include "main.h"

/*
 * 串口报文格式（多字节字段均为小端序）：
 * AA 55 | BODY_LEN | SEQ(2) | CAN_ID(4) | FLAGS | LEN | DATA(0..64)
 *       | CRC8-ATM | 55 AA
 * BODY_LEN = 8 + LEN；CRC 覆盖 BODY_LEN 至 DATA 最后一个字节。
 * FLAGS：bit0=扩展 ID，bit1=CAN FD，bit2=BRS，bit3=远程帧。
 *
 * 数据路径 1（电脑发 CAN）：串口 DMA -> 协议解析 -> CAN 软件队列 -> FDCAN1。
 * 数据路径 2（CAN 上报电脑）：FDCAN1 RX FIFO -> CAN 软件队列 -> 串口 DMA。
 */
HAL_StatusTypeDef CanUartGateway_Init(void);
void CanUartGateway_Process(void);

#ifdef __cplusplus
}
#endif

#endif /* __CAN_UART_GATEWAY_H__ */
