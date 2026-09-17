#include "system_heartbeat.h"

/*
 * 心跳属于 AA58 System/ROV 公共链路协议，不属于 AA55 CAN Gateway 协议。
 * 因此本文件不包含 CAN/FDCAN 头文件，也不访问 CAN 队列或 CAN 控制器。
 */

#define SYSTEM_HEARTBEAT_PERIOD_MS   1000U
#define SYSTEM_HEARTBEAT_PACKET_SIZE 20U

#define SYSTEM_FRAME_START            0xAAU
#define SYSTEM_FRAME_FAMILY           0x58U
#define SYSTEM_FRAME_VERSION          0x01U
#define SYSTEM_CMD_PING               0x01U
#define SYSTEM_FRAME_FLAGS            0x00U
#define SYSTEM_TARGET_SYSTEM          0x00U
#define SYSTEM_FRAME_END_FAMILY       0x58U
#define SYSTEM_FRAME_END              0xAAU

static SystemHeartbeatTransportOps_t heartbeat_transport = {0};
static uint32_t heartbeat_last_tick = 0U;
static uint32_t heartbeat_sequence = 0U;

/**
 * @brief 按规范计算 CRC16-CCITT。
 *
 * 多项式为 0x1021，初值为 0xFFFF。输入从 FAMILY 开始，到 TIMESTAMP_US
 * 最后一个字节结束；不包含帧头 AA、CRC 自身和帧尾 58 AA。
 */
static uint16_t SystemHeartbeat_Crc16(const uint8_t *data, uint16_t length)
{
  uint16_t crc = 0xFFFFU;
  uint16_t i;
  uint8_t bit;

  if (data == NULL)
  {
    return 0U;
  }

  for (i = 0U; i < length; i++)
  {
    crc ^= (uint16_t)data[i] << 8U;
    for (bit = 0U; bit < 8U; bit++)
    {
      if ((crc & 0x8000U) != 0U)
      {
        crc = (uint16_t)((crc << 1U) ^ 0x1021U);
      }
      else
      {
        crc <<= 1U;
      }
    }
  }
  return crc;
}

/**
 * @brief 以小端序写入 32 位公共字段。
 */
static void SystemHeartbeat_WriteU32Le(uint8_t *dst, uint32_t value)
{
  dst[0] = (uint8_t)(value & 0xFFU);
  dst[1] = (uint8_t)((value >> 8U) & 0xFFU);
  dst[2] = (uint8_t)((value >> 16U) & 0xFFU);
  dst[3] = (uint8_t)((value >> 24U) & 0xFFU);
}

/**
 * @brief 组装并发送一帧无 payload 的 AA58 System PING。
 *
 * 固定帧格式如下（共 20 字节）：
 *
 *   AA 58 01 01 00 00 SEQ(4) 00 00 TIMESTAMP_US(4) CRC16 58 AA
 *
 * 该包只代表设备与上位机之间的公共通信链路仍在工作，不携带 CAN ID、
 * CAN 数据或 CAN 状态，因此上位机可以独立于 CAN 网关解析它。
 */
static void SystemHeartbeat_Send(void)
{
  uint8_t packet[SYSTEM_HEARTBEAT_PACKET_SIZE] = {0};
  uint16_t crc;
  uint32_t timestamp_us;
  SystemHeartbeatIoResult_t result;

  packet[0] = SYSTEM_FRAME_START;
  packet[1] = SYSTEM_FRAME_FAMILY;
  packet[2] = SYSTEM_FRAME_VERSION;
  packet[3] = SYSTEM_CMD_PING;
  packet[4] = SYSTEM_FRAME_FLAGS;
  packet[5] = SYSTEM_TARGET_SYSTEM;

  SystemHeartbeat_WriteU32Le(&packet[6], heartbeat_sequence);
  /* 无 payload，PAYLOAD_LEN 位于偏移 10，保持 0。 */

  /* 当前系统时基为毫秒，将其换算成协议要求的微秒时间戳。 */
  timestamp_us = HAL_GetTick() * 1000U;
  SystemHeartbeat_WriteU32Le(&packet[12], timestamp_us);

  /* CRC 覆盖偏移 1..15：FAMILY 至 TIMESTAMP_US。 */
  crc = SystemHeartbeat_Crc16(&packet[1], 15U);
  packet[16] = (uint8_t)(crc & 0xFFU);
  packet[17] = (uint8_t)((crc >> 8U) & 0xFFU);
  packet[18] = SYSTEM_FRAME_END_FAMILY;
  packet[19] = SYSTEM_FRAME_END;

  result = heartbeat_transport.send_packet(heartbeat_transport.context,
                                            packet,
                                            sizeof(packet));
  if (result == SYSTEM_HEARTBEAT_IO_OK)
  {
    heartbeat_sequence++;
  }
}

HAL_StatusTypeDef SystemHeartbeat_Init(
    const SystemHeartbeatTransportOps_t *transport)
{
  if ((transport == NULL) || (transport->send_packet == NULL))
  {
    return HAL_ERROR;
  }

  heartbeat_transport = *transport;
  heartbeat_last_tick = HAL_GetTick();
  heartbeat_sequence = 0U;
  return HAL_OK;
}

void SystemHeartbeat_Process(void)
{
  uint32_t now = HAL_GetTick();

  if ((uint32_t)(now - heartbeat_last_tick) < SYSTEM_HEARTBEAT_PERIOD_MS)
  {
    return;
  }

  heartbeat_last_tick = now;
  SystemHeartbeat_Send();
}
