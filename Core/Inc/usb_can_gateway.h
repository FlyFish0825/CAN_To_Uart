#ifndef __USB_CAN_GATEWAY_H__
#define __USB_CAN_GATEWAY_H__

#ifdef __cplusplus
extern "C" {
#endif

#include "can_gateway_core.h"

/*
 * USB CDC 只是字节传输层，协议仍沿用现有 AA 55 CAN 网关格式。
 * USB 接收环形缓冲用于吸收 CDC 分包、粘包和主循环调度抖动。
 *
 * 本模块不解析 AA55 协议：CDC 回调只把收到的字节复制到 RX 环形缓冲，
 * 主循环再由 UsbCanGateway_Process() 取出，并交给 CanGateway_RxFeed()。
 * 这样 USB 中断上下文不会执行 CRC、CAN 入队或阻塞式发送。
 */
#define USB_CAN_RX_RING_SIZE       (8U * 1024U)
#define USB_CAN_TX_QUEUE_SIZE      256U
#define USB_CAN_PACKET_SIZE        78U

_Static_assert((USB_CAN_RX_RING_SIZE & (USB_CAN_RX_RING_SIZE - 1U)) == 0U,
               "USB CAN RX ring size must be a power of two");
_Static_assert((USB_CAN_TX_QUEUE_SIZE & (USB_CAN_TX_QUEUE_SIZE - 1U)) == 0U,
               "USB CAN TX queue size must be a power of two");

/**
 * @brief 将 USB CDC OUT 回调收到的字节复制到 RX 环形缓冲。
 * @param data CDC 回调提供的接收缓冲区。
 * @param len  本次收到的字节数。
 *
 * 函数只做内存复制并快速返回，适合在 USB 中断/回调中调用。缓冲区满时
 * 不覆盖尚未处理的数据，而是丢弃超出的字节并累加丢包计数。
 */
void UsbCanGateway_RxPush(const uint8_t *data, uint16_t len);

/**
 * @brief USB 传输层主循环服务函数。
 *
 * 先把 RX 环形缓冲中的字节逐个交给 CanGateway_RxFeed()，再尝试启动一
 * 个尚未发送的 TX 队列包。发送完成由 CDC_TransmitCplt_FS() 回调通知，
 * 因此本函数不会等待 USB IN 传输结束。
 */
void UsbCanGateway_Process(void);

/**
 * @brief 获取 USB CDC 的通用传输适配器。
 * @return 只读操作表指针，交给 CanGateway_Init() 注册。
 *
 * 上层不需要知道 USB 队列的实现细节；未来更换为 UART/TCP 时，只需返回
 * 另一份具有相同 CanGatewayTransportOps_t 类型的操作表。
 */
const CanGatewayTransportOps_t *UsbCanGateway_GetTransport(void);

/**
 * @brief 将一个完整电脑协议包复制到 USB TX 队列。
 * @param data  协议包首地址。
 * @param len   协议包长度，不能超过 USB_CAN_PACKET_SIZE。
 * @return HAL_OK 已复制入队；HAL_BUSY 队列满；HAL_ERROR 参数非法。
 *
 * 函数只负责入队，不等待 USB。入队成功后调用方可以立即复用 data。
 */
HAL_StatusTypeDef UsbCanGateway_TxEnqueue(const uint8_t *data, uint16_t len);

/**
 * @brief 通知 USB TX 队列当前包已经由 CDC 发送完成。
 *
 * 只能由 CDC_TransmitCplt_FS() 调用；函数仅推进队列尾指针并清除 busy
 * 标志，不执行协议解析或新的阻塞操作。
 */
void UsbCanGateway_TxComplete(void);

/**
 * @brief USB 重新枚举/配置完成时重置发送 busy 状态。
 *
 * 主机断开或重新枚举可能使旧的发送完成回调永远不会到达，因此配置完成
 * 后清除 busy 标志，让主循环能够继续尝试发送队列中的数据；已排队数据
 * 保留不清空。
 */
void UsbCanGateway_OnConfigured(void);

#ifdef __cplusplus
}
#endif

#endif /* __USB_CAN_GATEWAY_H__ */
