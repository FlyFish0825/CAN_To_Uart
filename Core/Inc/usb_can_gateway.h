#ifndef __USB_CAN_GATEWAY_H__
#define __USB_CAN_GATEWAY_H__

#ifdef __cplusplus
extern "C" {
#endif

#include "main.h"

/*
 * USB CDC 只是字节传输层，协议仍沿用现有 AA 55 CAN 网关格式。
 * USB 接收环形缓冲用于吸收 CDC 分包、粘包和主循环调度抖动。
 */
#define USB_CAN_RX_RING_SIZE       (8U * 1024U)
#define USB_CAN_TX_QUEUE_SIZE      256U
#define USB_CAN_PACKET_SIZE        78U

_Static_assert((USB_CAN_RX_RING_SIZE & (USB_CAN_RX_RING_SIZE - 1U)) == 0U,
               "USB CAN RX ring size must be a power of two");
_Static_assert((USB_CAN_TX_QUEUE_SIZE & (USB_CAN_TX_QUEUE_SIZE - 1U)) == 0U,
               "USB CAN TX queue size must be a power of two");

/* USB CDC 接收回调只调用此函数，不能在回调中解析协议。 */
void UsbCanGateway_RxPush(const uint8_t *data, uint16_t len);

/* 主循环调用：处理 USB RX 字节，并启动 USB TX 队列发送。 */
void UsbCanGateway_Process(void);

/* CAN 网关生成的协议包同时复制到 USB 可靠发送队列。 */
HAL_StatusTypeDef UsbCanGateway_TxEnqueue(const uint8_t *data, uint16_t len);

/* CDC 发送完成回调调用；这里只更新队列状态。 */
void UsbCanGateway_TxComplete(void);

/* USB 重新枚举后清除上一次未完成的发送状态，但保留待发送队列。 */
void UsbCanGateway_OnConfigured(void);

#ifdef __cplusplus
}
#endif

#endif /* __USB_CAN_GATEWAY_H__ */
