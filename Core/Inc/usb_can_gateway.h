#ifndef __USB_CAN_GATEWAY_H__
#define __USB_CAN_GATEWAY_H__

#ifdef __cplusplus
extern "C" {
#endif

#include "can_gateway_core.h"
#include "system_heartbeat.h"

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
/* 每轮最多解析固定数量的输入字节，避免连续输入长期独占主循环。 */
#define USB_CAN_RX_PROCESS_BUDGET  256U
/* USB FS CDC 单个 OUT 包最大 64 字节，重新接收前至少预留一个包空间。 */
#define USB_CAN_RX_PACKET_RESERVE  64U
/* USB IN 完成回调异常丢失时，超过该时间自动恢复发送状态。 */
#define USB_CAN_TX_STALL_TIMEOUT_MS 1000U
/* 实际环形队列保留一个空槽，容量为 255；高/低水位用于输入反压。 */
#define USB_CAN_TX_HIGH_WATERMARK  192U
#define USB_CAN_TX_LOW_WATERMARK   64U

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
 * @brief 判断 RX 环形缓冲是否能完整容纳下一包输入。
 *
 * USB 回调收到当前数据包后调用此函数。它只判断 RX 环形缓存是否能容纳
 * 已经到达的数据，不会因为下游水位变化而丢弃当前包。
 */
uint8_t UsbCanGateway_RxCanAccept(uint16_t len);

/**
 * @brief 判断是否允许重新提交下一次 USB OUT 接收。
 *
 * 除 RX 缓存空间外，还检查 USB TX 和 CAN 软件队列高水位；返回 0 时暂
 * 不重新提交，让 USB 端点通过 NAK 对主机形成自然反压。
 */
uint8_t UsbCanGateway_RxCanRearm(uint16_t len);

/**
 * @brief 标记 USB OUT 暂停，待主循环清出空间后恢复。
 */
void UsbCanGateway_RxMarkPaused(void);

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
 * @brief 获取同一外部传输队列的系统心跳适配器。
 *
 * 心跳使用独立的 AA58 System 协议，但仍复用本模块提供的可靠发送队列；
 * 该函数返回的接口与 CAN 网关核心接口相互独立。
 */
const SystemHeartbeatTransportOps_t *UsbCanGateway_GetSystemTransport(void);

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

/**
 * @brief USB CDC 被断开/反初始化时清理发送状态。
 *
 * 当前发送包不会从队列删除，重新枚举后会从同一个队列槽重新尝试，
 * 避免断开瞬间静默丢包。
 */
void UsbCanGateway_OnDeconfigured(void);

#ifdef __cplusplus
}
#endif

#endif /* __USB_CAN_GATEWAY_H__ */
