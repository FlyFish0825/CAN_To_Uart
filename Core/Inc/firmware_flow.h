/**
 * @file firmware_flow.h
 * @brief AA59 固件块可靠转发协议的常量和轮询接口。
 *
 * 输入是可分片的 AA59 字节流，合法逻辑块先进入固定队列，再由主循环
 * 拆成经典 CAN/CAN FD 帧；FLOW_ACK 通过注册的外部传输接口返回。
 */
#ifndef __FIRMWARE_FLOW_H__
#define __FIRMWARE_FLOW_H__

#ifdef __cplusplus
extern "C" {
#endif

#include "can_gateway_core.h"

/* 一个逻辑块最多携带 64 个真实数据字节。 */
#define FW_FLOW_BLOCK_SIZE             64U /* 一个可靠传输逻辑块携带的最大数据字节数。 */
#define FW_FLOW_QUEUE_BLOCKS           32U /* 固件块软件队列的槽位数量。 */
#define FW_FLOW_INITIAL_CREDIT         16U /* BEGIN 成功后允许发送方预先提交的块数。 */
#define FW_FLOW_ACK_INTERVAL           4U /* 累积达到该数量时请求一次 ACK。 */
#define FW_FLOW_HIGH_WATERMARK         24U /* 队列达到此占用量时触发接收侧降速。 */
#define FW_FLOW_LOW_WATERMARK          12U /* 队列降至此占用量时允许恢复接收。 */
#define FW_FLOW_ACK_INTERVAL_MS        10U /* ACK 最小时间间隔，单位为毫秒。 */

/* AA59 DATA_BLOCK 负载：CAN_ID(4)+FLAGS(1)+INDEX(4)+OFFSET(4)+LEN(1)+DATA。 */
#define FW_FLOW_DATA_HEADER_SIZE       14U /* DATA_BLOCK 中 CAN 元数据的字节数。 */
#define FW_FLOW_MAX_PAYLOAD            (FW_FLOW_DATA_HEADER_SIZE + FW_FLOW_BLOCK_SIZE) /* 最大命令负载。 */
#define FW_FLOW_RX_PACKET_SIZE         (20U + FW_FLOW_MAX_PAYLOAD) /* 接收缓存可容纳的完整包上限。 */

/* AA59 固件扩展命令。0x01~0x05 保留给公共固件会话命令。 */
#define FW_FLOW_CMD_BEGIN              0x06U /* 开始一次固件可靠传输会话。 */
#define FW_FLOW_CMD_DATA_BLOCK         0x10U /* 提交一个待转发的 CAN 数据块。 */
#define FW_FLOW_CMD_END                0x11U /* 请求结束当前固件传输会话。 */
#define FW_FLOW_CMD_FLOW_ACK           0x80U /* 网关返回的流控确认命令。 */

/* ACK 负载中的状态码。 */
#define FW_FLOW_STATUS_OK              0x00U /* 命令或数据块处理成功。 */
#define FW_FLOW_STATUS_BUSY            0x01U /* 会话或下游状态暂不允许处理。 */
#define FW_FLOW_STATUS_QUEUE_FULL      0x02U /* 固件块软件队列没有空闲槽位。 */
#define FW_FLOW_STATUS_INVALID_BLOCK   0x03U /* 块序号、偏移、长度或协议校验非法。 */
#define FW_FLOW_STATUS_FORWARD_FAILED  0x04U /* CAN 转发提交失败。 */
#define FW_FLOW_STATUS_INTERNAL_ERROR  0x05U /* 内部状态无法完成请求。 */

/* 逻辑块负载中的 CAN FLAGS，与 AA55 CAN FLAGS 保持相同位定义。 */
#define FW_FLOW_CAN_FLAG_EXTENDED      0x01U /* CAN ID 使用扩展格式。 */
#define FW_FLOW_CAN_FLAG_FD            0x02U /* CAN FD 帧。 */
#define FW_FLOW_CAN_FLAG_BRS           0x04U /* CAN FD 启用 bit-rate switching。 */
#define FW_FLOW_CAN_FLAG_REMOTE        0x08U /* 远程帧标志。 */

/**
 * @brief 注册 ACK 的外部发送接口。
 *
 * 发送接口与核心使用同一抽象：返回 OK 只表示 ACK 已复制到传输层队列，
 * 返回 BUSY 时 ACK 保留在本模块中，下一轮继续尝试，不会丢 credit。
 */
HAL_StatusTypeDef FirmwareFlow_Init(
    const CanGatewayTransportOps_t *transport);

/**
 * @brief 向 AA59 固件块解析器输入任意粒度的字节流。
 *
 * 该入口只解析并把合法块复制到固定队列；不在 USB 回调中访问 CAN 硬件。
 */
void FirmwareFlow_RxFeed(const uint8_t *data, uint16_t length);

/**
 * @brief 主循环服务逻辑块转发和累计 ACK。
 *
 * 每轮只在 CAN 核心队列能接收时继续拆发，队列忙时保留当前块和当前
 * 分片位置。调用者应在 while(1) 中持续调用，不要在中断中调用。
 */
void FirmwareFlow_Process(void);

/**
 * @brief 获取 AA59 逻辑块队列占用百分比，范围为 0~100。
 */
uint8_t FirmwareFlow_GetQueueUsage(void);

#ifdef __cplusplus
}
#endif

#endif /* __FIRMWARE_FLOW_H__ */
