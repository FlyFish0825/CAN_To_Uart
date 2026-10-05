/**
 * @file system_heartbeat.h
 * @brief AA58 系统心跳协议的传输适配接口。
 *
 * 心跳是独立于 AA55 CAN 数据的公共链路状态包；模块只负责定时组包，
 * 不直接依赖 USB、CAN 或其他具体外设。
 */
#ifndef __SYSTEM_HEARTBEAT_H__
#define __SYSTEM_HEARTBEAT_H__

#ifdef __cplusplus
extern "C" {
#endif

#include "main.h"

/**
 * @brief 系统心跳发送结果。
 *
 * 心跳模块只关心“数据是否已经复制到外部传输层队列”，不关心具体的
 * USB、串口或其他物理接口。传输层必须在返回前复制数据，不能保存本地
 * 临时数组的地址。
 */
typedef enum
{
  /** 心跳数据已复制到传输层发送队列。 */
  SYSTEM_HEARTBEAT_IO_OK = 0,
  /** 传输层队列暂满，本次心跳未发送。 */
  SYSTEM_HEARTBEAT_IO_BUSY,
  /** 参数或底层传输失败，本次心跳未发送。 */
  SYSTEM_HEARTBEAT_IO_ERROR
} SystemHeartbeatIoResult_t;

/**
 * @brief 系统心跳使用的通用发送函数类型。
 *
 * @param context 传输层私有上下文，不使用时传 NULL。
 * @param data    待发送的完整协议帧。
 * @param length  协议帧长度，单位为字节。
 * @return 数据已入传输层队列、队列暂忙或发送失败。
 */
typedef SystemHeartbeatIoResult_t (*SystemHeartbeatSendPacketFn)(
    void *context,
    const uint8_t *data,
    uint16_t length);

/**
 * @brief 心跳中携带的运行状态百分比。
 *
 * 所有字段均表示对应软件缓冲区当前已使用的比例，取值范围 0~100，
 * 其中 100 表示达到该队列的可用容量上限。该结构只描述通用运行状态，
 * 不包含任何具体外设名称，便于其他传输实现复用同一套心跳协议。
 */
typedef struct
{
  /** 心跳输入缓冲区的当前占用百分比。 */
  uint8_t input_buffer_percent;
  /** 心跳输出缓冲区的当前占用百分比。 */
  uint8_t output_buffer_percent;
  /** CAN 接收软件队列的当前占用百分比。 */
  uint8_t can_rx_buffer_percent;
  /** CAN 发送软件队列的当前占用百分比。 */
  uint8_t can_tx_buffer_percent;
  /** 固件可靠传输队列的当前占用百分比。 */
  uint8_t flow_buffer_percent;
} SystemHeartbeatMetrics_t;

/**
 * @brief 获取本次心跳需要上报的缓冲区占用率。
 *
 * 回调必须快速返回，不得阻塞或执行外设发送。未使用某个状态项时填 0。
 */
typedef void (*SystemHeartbeatMetricsFn)(
    void *context,
    SystemHeartbeatMetrics_t *metrics);

/**
 * @brief 系统心跳与具体传输接口之间的最小适配表。
 */
typedef struct
{
  /** 心跳协议包发送回调；返回前必须复制 data 指向的数据。 */
  SystemHeartbeatSendPacketFn send_packet;
  /** send_packet 使用的私有上下文。 */
  void *context;
  /** 填充本次心跳状态的回调；可以为 NULL 表示不提供指标。 */
  SystemHeartbeatMetricsFn get_metrics;
  /** get_metrics 使用的私有上下文。 */
  void *metrics_context;
} SystemHeartbeatTransportOps_t;

/**
 * @brief 注册心跳的外部发送接口并初始化定时器。
 *
 * 只需在底层传输初始化完成后调用一次。注册成功后，主循环持续调用
 * SystemHeartbeat_Process()，模块会按固定周期向上位机发送 AA58 PING，
 * 并在 PING 的 payload 中携带可选的运行状态百分比。
 */
HAL_StatusTypeDef SystemHeartbeat_Init(
    const SystemHeartbeatTransportOps_t *transport);

/**
 * @brief 轮询系统心跳调度器。
 *
 * 函数非阻塞；未到发送周期时立即返回。发送队列暂忙时本次心跳不阻塞
 * 主循环，下一周期会继续尝试。
 */
void SystemHeartbeat_Process(void);

#ifdef __cplusplus
}
#endif

#endif /* __SYSTEM_HEARTBEAT_H__ */
