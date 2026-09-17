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
  SYSTEM_HEARTBEAT_IO_OK = 0,
  SYSTEM_HEARTBEAT_IO_BUSY,
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
 * @brief 系统心跳与具体传输接口之间的最小适配表。
 */
typedef struct
{
  SystemHeartbeatSendPacketFn send_packet;
  void *context;
} SystemHeartbeatTransportOps_t;

/**
 * @brief 注册心跳的外部发送接口并初始化定时器。
 *
 * 只需在底层传输初始化完成后调用一次。注册成功后，主循环持续调用
 * SystemHeartbeat_Process()，模块会按固定周期向上位机发送 AA58 PING。
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
