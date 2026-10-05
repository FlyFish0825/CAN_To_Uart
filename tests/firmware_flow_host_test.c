/**
 * @file firmware_flow_host_test.c
 * @brief 不依赖 STM32 HAL 的 AA59 固件流控主机测试。
 *
 * 本文件提供最小 HAL/CAN 桩并直接编译 firmware_flow.c，覆盖协议 CRC、
 * 半包输入、经典 CAN/CAN FD 分帧、credit、队列反压和 ACK 重试；它不代表
 * 已在真实 USB、FDCAN 或目标板上运行。
 */
#include <assert.h>
#include <stdint.h>
#include <stdio.h>
#include <string.h>

/*
 * 主机单元测试直接包含固件实现；先定义核心头文件保护和最小 HAL/网关接口，
 * 避免把 Cortex-M7 寄存器头文件带入 Windows GCC。
 */
#define __CAN_GATEWAY_CORE_H__

#define HAL_OK    0
#define HAL_ERROR 1
typedef int HAL_StatusTypeDef;

typedef enum
{
  CAN_GATEWAY_IO_OK = 0,
  CAN_GATEWAY_IO_BUSY,
  CAN_GATEWAY_IO_ERROR
} CanGatewayIoResult_t;

typedef CanGatewayIoResult_t (*CanGatewaySendPacketFn)(
    void *context, const uint8_t *data, uint16_t length);

typedef struct
{
  CanGatewaySendPacketFn send_packet;
  void *context;
} CanGatewayTransportOps_t;

typedef struct
{
  uint32_t id;
  uint8_t flags;
  uint8_t len;
  uint8_t data[64];
} CanGatewayCanFrame_t;

typedef void (*CanGatewayTxCompletionFn)(void *context,
                                         uint32_t token,
                                         uint8_t success);

#include "../Core/Inc/firmware_flow.h"

#define TEST_MAX_CAN_FRAMES  4096U /* CAN 入队桩可记录的最大帧数。 */
#define TEST_MAX_ACK_PACKETS 1024U /* 外部 ACK 发送桩可记录的最大包数。 */

static uint32_t test_tick; /* HAL_GetTick() 主机桩返回的可控时间。 */
static uint8_t test_can_ready = 1U; /* 主机桩报告的 CAN 队列可用状态。 */
static CanGatewayIoResult_t test_can_result = CAN_GATEWAY_IO_OK; /* 入队桩的预设返回值。 */
static CanGatewayIoResult_t test_send_result = CAN_GATEWAY_IO_OK; /* 外部发送桩的预设返回值。 */
static CanGatewayCanFrame_t test_can_frames[TEST_MAX_CAN_FRAMES]; /* 记录已入队 CAN 帧。 */
static uint32_t test_can_frame_count; /* 已记录 CAN 帧数。 */
static uint8_t test_ack_packets[TEST_MAX_ACK_PACKETS][64]; /* 记录发送桩收到的 ACK 包。 */
static uint16_t test_ack_lengths[TEST_MAX_ACK_PACKETS]; /* 每个 ACK 包的实际长度。 */
static uint32_t test_ack_count; /* 已记录 ACK 包数。 */
static CanGatewayTxCompletionFn test_completion_callback; /* 固件流控注册的完成回调。 */
static void *test_completion_context; /* 完成回调的上下文桩值。 */

uint32_t HAL_GetTick(void)
{
  /* 用可控时钟替代 MCU SysTick，使 ACK 间隔和解析超时可重复测试。 */
  return test_tick;
}

uint8_t CanGateway_CanTxReady(void)
{
  return test_can_ready;
}

CanGatewayIoResult_t CanGateway_QueueCanFrame(
    const CanGatewayCanFrame_t *frame)
{
  /* 该桩只记录逻辑入队结果，不模拟真实 FDCAN FIFO。 */
  if (test_can_result != CAN_GATEWAY_IO_OK)
  {
    return test_can_result;
  }
  assert(frame != NULL);
  assert(test_can_frame_count < TEST_MAX_CAN_FRAMES);
  test_can_frames[test_can_frame_count++] = *frame;
  return CAN_GATEWAY_IO_OK;
}

CanGatewayIoResult_t CanGateway_QueueTrackedCanFrame(
    const CanGatewayCanFrame_t *frame,
    uint32_t token)
{
  CanGatewayIoResult_t result = CanGateway_QueueCanFrame(frame);
  if ((result == CAN_GATEWAY_IO_OK) && (test_completion_callback != NULL))
  {
    /* 主机桩立即模拟“FDCAN 已接受”，硬件上的真实回调由主循环稍后产生。 */
    test_completion_callback(test_completion_context, token, 1U);
  }
  return result;
}

HAL_StatusTypeDef CanGateway_SetTxCompletionCallback(
    CanGatewayTxCompletionFn callback,
    void *context)
{
  if (callback == NULL)
  {
    return HAL_ERROR;
  }
  test_completion_callback = callback;
  test_completion_context = context;
  return HAL_OK;
}

/** 测试传输回调：复制发送数据并根据预设结果模拟队列状态。 */
static CanGatewayIoResult_t Test_SendPacket(
    void *context, const uint8_t *data, uint16_t length)
{
  (void)context;
  if (test_send_result != CAN_GATEWAY_IO_OK)
  {
    return test_send_result;
  }
  assert(data != NULL);
  assert(length <= sizeof(test_ack_packets[0]));
  assert(test_ack_count < TEST_MAX_ACK_PACKETS);
  memcpy(test_ack_packets[test_ack_count], data, length);
  test_ack_lengths[test_ack_count] = length;
  test_ack_count++;
  return CAN_GATEWAY_IO_OK;
}

#include "../Core/Src/firmware_flow.c"

static const CanGatewayTransportOps_t test_transport =
{
  Test_SendPacket,
  NULL
};

static uint16_t Test_ReadU16Le(const uint8_t *data)
{
  /* 测试协议字段均按小端序编码。 */
  return (uint16_t)data[0] | ((uint16_t)data[1] << 8U);
}

/** 从测试 ACK 的固定字段读取小端序 32 位值。 */
static uint32_t Test_ReadU32Le(const uint8_t *data)
{
  return (uint32_t)data[0] |
         ((uint32_t)data[1] << 8U) |
         ((uint32_t)data[2] << 16U) |
         ((uint32_t)data[3] << 24U);
}

/** 向测试协议缓冲区写入小端序 16 位值。 */
static void Test_WriteU16Le(uint8_t *data, uint16_t value)
{
  data[0] = (uint8_t)value;
  data[1] = (uint8_t)(value >> 8U);
}

/** 向测试协议缓冲区写入小端序 32 位值。 */
static void Test_WriteU32Le(uint8_t *data, uint32_t value)
{
  data[0] = (uint8_t)value;
  data[1] = (uint8_t)(value >> 8U);
  data[2] = (uint8_t)(value >> 16U);
  data[3] = (uint8_t)(value >> 24U);
}

/** 按 AA59 实际格式构造测试包，并计算协议 CRC 与帧尾。 */
static uint16_t Test_BuildPacket(uint8_t command,
                                 const uint8_t *payload,
                                 uint16_t payload_length,
                                 uint8_t *packet)
{
  uint16_t crc;
  uint16_t crc_index;

  memset(packet, 0, FW_FLOW_RX_PACKET_SIZE);
  packet[0] = 0xAAU;
  packet[1] = 0x59U;
  packet[2] = 0x01U;
  packet[3] = command;
  packet[5] = 0x01U;
  Test_WriteU16Le(&packet[10], payload_length);
  if ((payload != NULL) && (payload_length != 0U))
  {
    memcpy(&packet[16], payload, payload_length);
  }
  crc_index = (uint16_t)(16U + payload_length);
  crc = FirmwareFlow_Crc16(&packet[1], (uint16_t)(15U + payload_length));
  Test_WriteU16Le(&packet[crc_index], crc);
  packet[crc_index + 2U] = 0x59U;
  packet[crc_index + 3U] = 0xAAU;
  return (uint16_t)(crc_index + 4U);
}

/** 以变化长度分片喂包，覆盖半包、边界和连续输入路径。 */
static void Test_FeedPacket(const uint8_t *packet, uint16_t length)
{
  uint16_t offset = 0U;
  uint16_t chunk;

  /* 故意使用 1、7、13 字节循环分块，覆盖 USB 半包和粘包边界。 */
  while (offset < length)
  {
    chunk = (uint16_t)(1U + ((offset * 6U) % 13U));
    if (chunk > (uint16_t)(length - offset))
    {
      chunk = (uint16_t)(length - offset);
    }
    FirmwareFlow_RxFeed(&packet[offset], chunk);
    offset = (uint16_t)(offset + chunk);
  }
}

/** 清理主机桩状态并重新初始化被测固件流控模块。 */
static void Test_Reset(void)
{
  test_tick = 0U;
  test_can_ready = 1U;
  test_can_result = CAN_GATEWAY_IO_OK;
  test_send_result = CAN_GATEWAY_IO_OK;
  test_can_frame_count = 0U;
  test_ack_count = 0U;
  test_completion_callback = NULL;
  test_completion_context = NULL;
  memset(test_can_frames, 0, sizeof(test_can_frames));
  memset(test_ack_packets, 0, sizeof(test_ack_packets));
  memset(test_ack_lengths, 0, sizeof(test_ack_lengths));
  assert(FirmwareFlow_Init(&test_transport) == HAL_OK);
}

/** 构造 BEGIN 包并检查初始 ACK 的会话、credit 和队列容量字段。 */
static void Test_SendBegin(uint32_t firmware_size)
{
  uint8_t payload[4]; /* BEGIN 的固件总长度字段。 */
  uint8_t packet[FW_FLOW_RX_PACKET_SIZE]; /* AA59 测试包缓存。 */
  uint16_t length; /* 实际构造出的完整包长度。 */

  Test_WriteU32Le(payload, firmware_size);
  length = Test_BuildPacket(FW_FLOW_CMD_BEGIN,
                            payload, sizeof(payload), packet);
  Test_FeedPacket(packet, length);
  FirmwareFlow_Process();
  assert(test_ack_count == 1U);
  assert(test_ack_lengths[0] == 34U);
  assert(test_ack_packets[0][3] == FW_FLOW_CMD_FLOW_ACK);
  assert(Test_ReadU32Le(&test_ack_packets[0][16]) == 0xFFFFFFFFUL);
  assert(Test_ReadU16Le(&test_ack_packets[0][20]) == 0U);
  assert(Test_ReadU16Le(&test_ack_packets[0][22]) == FW_FLOW_QUEUE_BLOCKS);
  assert(test_ack_packets[0][24] == FW_FLOW_STATUS_OK);
}

/** 构造 DATA_BLOCK 包；数据内容按 offset 递增便于验证转发结果。 */
static void Test_SendBlock(uint32_t can_id,
                           uint8_t can_flags,
                           uint32_t block_index,
                           uint32_t offset,
                           uint8_t valid_len)
{
  uint8_t payload[FW_FLOW_MAX_PAYLOAD]; /* DATA_BLOCK 头和数据。 */
  uint8_t packet[FW_FLOW_RX_PACKET_SIZE]; /* AA59 测试包缓存。 */
  uint16_t length; /* 实际构造出的完整包长度。 */
  uint8_t i; /* 填充递增测试数据的索引。 */

  memset(payload, 0, sizeof(payload));
  Test_WriteU32Le(&payload[0], can_id);
  payload[4] = can_flags;
  Test_WriteU32Le(&payload[5], block_index);
  Test_WriteU32Le(&payload[9], offset);
  payload[13] = valid_len;
  for (i = 0U; i < valid_len; i++)
  {
    payload[14U + i] = (uint8_t)(offset + i);
  }
  length = Test_BuildPacket(FW_FLOW_CMD_DATA_BLOCK,
                            payload,
                            (uint16_t)(FW_FLOW_DATA_HEADER_SIZE + valid_len),
                            packet);
  Test_FeedPacket(packet, length);
}

static uint16_t Test_AckCreditSum(uint32_t start_index)
{
  uint32_t i; /* ACK 记录遍历索引。 */
  uint16_t total = 0U; /* 累计返还的 credit。 */
  for (i = start_index; i < test_ack_count; i++)
  {
    total = (uint16_t)(total + Test_ReadU16Le(&test_ack_packets[i][20]));
  }
  return total;
}

/** 验证经典 CAN 按 8 字节拆帧以及每块只返还一次 credit。 */
static void Test_ClassicLengths(void)
{
  static const uint8_t lengths[] = {1U, 7U, 8U, 9U, 23U, 63U, 64U};
  uint32_t case_index;

  for (case_index = 0U;
       case_index < sizeof(lengths) / sizeof(lengths[0]);
       case_index++)
  {
    uint8_t length = lengths[case_index];
    uint8_t expected_frames = (uint8_t)((length + 7U) / 8U);
    uint8_t frame_index;

    Test_Reset();
    Test_SendBegin(length);
    Test_SendBlock(0x100U, 0U, 0U, 0U, length);
    FirmwareFlow_Process();
    assert(test_can_frame_count == expected_frames);
    for (frame_index = 0U; frame_index < expected_frames; frame_index++)
    {
      uint8_t expected_len = (uint8_t)(length - frame_index * 8U);
      if (expected_len > 8U) expected_len = 8U;
      assert(test_can_frames[frame_index].id == 0x100U + frame_index);
      assert(test_can_frames[frame_index].len == expected_len);
    }
    assert(test_ack_count == 2U);
    assert(Test_ReadU32Le(&test_ack_packets[1][16]) == 0U);
    assert(Test_ReadU16Le(&test_ack_packets[1][20]) == 1U);
    assert(test_ack_packets[1][24] == FW_FLOW_STATUS_OK);
  }
}

/** 将任意逻辑长度映射为 CAN FD 的合法 DLC 数据长度。 */
static uint8_t Test_ExpectedFdLength(uint8_t length)
{
  if (length <= 8U) return length;
  if (length <= 12U) return 12U;
  if (length <= 16U) return 16U;
  if (length <= 20U) return 20U;
  if (length <= 24U) return 24U;
  if (length <= 32U) return 32U;
  if (length <= 48U) return 48U;
  return 64U;
}

/** 验证 CAN FD 单帧长度向上取整和填充字节规则。 */
static void Test_FdLengths(void)
{
  static const uint8_t lengths[] = {1U, 7U, 8U, 9U, 23U, 63U, 64U};
  uint32_t case_index;

  for (case_index = 0U;
       case_index < sizeof(lengths) / sizeof(lengths[0]);
       case_index++)
  {
    uint8_t length = lengths[case_index];
    uint8_t i;

    Test_Reset();
    Test_SendBegin(length);
    Test_SendBlock(0x600U,
                   FW_FLOW_CAN_FLAG_FD | FW_FLOW_CAN_FLAG_BRS,
                   0U, 0U, length);
    FirmwareFlow_Process();
    assert(test_can_frame_count == 1U);
    assert(test_can_frames[0].len == Test_ExpectedFdLength(length));
    for (i = length; i < test_can_frames[0].len; i++)
    {
      assert(test_can_frames[0].data[i] == 0xFFU);
    }
    assert(Test_ReadU16Le(&test_ack_packets[1][20]) == 1U);
  }
}

/** 验证多块连续传输、顺序推进、末块标记和累计 credit。 */
static void Test_MultiBlock(uint32_t firmware_size)
{
  uint32_t block_index = 0U;
  uint32_t offset = 0U;
  uint32_t first_ack;

  Test_Reset();
  Test_SendBegin(firmware_size);
  first_ack = test_ack_count;

  while (offset < firmware_size)
  {
    uint32_t batch = 0U;
    while ((batch < FW_FLOW_INITIAL_CREDIT) && (offset < firmware_size))
    {
      uint32_t remaining = firmware_size - offset;
      uint8_t valid_len = (remaining > 64U) ? 64U : (uint8_t)remaining;
      Test_SendBlock(0x100U, 0U, block_index, offset, valid_len);
      offset += valid_len;
      block_index++;
      batch++;
    }
    while (fw_queue_count != 0U)
    {
      FirmwareFlow_Process();
      test_tick++;
    }
  }

  FirmwareFlow_Process();
  assert(fw_last_completed_block == block_index - 1U);
  assert(fw_next_offset == firmware_size);
  assert(fw_last_block_received != 0U);
  assert(Test_AckCreditSum(first_ack) == block_index);
}

/** 验证初始 credit/高水位满载时拒绝新块，并在腾空后重试。 */
static void Test_QueueFullAndRetry(void)
{
  uint32_t i;
  uint8_t packet[FW_FLOW_RX_PACKET_SIZE];
  uint8_t payload[FW_FLOW_MAX_PAYLOAD];
  uint16_t length;

  Test_Reset();
  Test_SendBegin(17U * 64U);
  for (i = 0U; i < FW_FLOW_INITIAL_CREDIT; i++)
  {
    Test_SendBlock(0x100U, 0U, i, i * 64U, 64U);
  }
  assert(fw_queue_count == FW_FLOW_INITIAL_CREDIT);
  assert(fw_credit_available == 0U);

  memset(payload, 0, sizeof(payload));
  Test_WriteU32Le(&payload[0], 0x100U);
  Test_WriteU32Le(&payload[5], 16U);
  Test_WriteU32Le(&payload[9], 16U * 64U);
  payload[13] = 64U;
  length = Test_BuildPacket(FW_FLOW_CMD_DATA_BLOCK,
                            payload, sizeof(payload), packet);
  Test_FeedPacket(packet, length);
  FirmwareFlow_ProcessAck();
  assert(test_ack_packets[test_ack_count - 1U][24] ==
         FW_FLOW_STATUS_QUEUE_FULL);
  assert(fw_next_block_index == 16U);

  while (fw_queue_count != 0U)
  {
    FirmwareFlow_Process();
  }
  Test_SendBlock(0x100U, 0U, 16U, 16U * 64U, 64U);
  FirmwareFlow_Process();
  assert(fw_last_completed_block == 16U);
}

/** 验证 CAN 下游不可用时保留队首块，恢复后继续发送。 */
static void Test_CanBusyAndFailure(void)
{
  Test_Reset();
  Test_SendBegin(64U);
  Test_SendBlock(0x100U, 0U, 0U, 0U, 64U);
  test_can_ready = 0U;
  FirmwareFlow_Process();
  assert(test_can_frame_count == 0U);
  assert(fw_queue_count == 1U);
  assert(fw_credit_available == 15U);
  test_can_ready = 1U;
  FirmwareFlow_Process();
  assert(test_can_frame_count == 8U);
  assert(fw_queue_count == 0U);

  Test_Reset();
  Test_SendBegin(64U);
  Test_SendBlock(0x100U, 0U, 0U, 0U, 64U);
  test_can_result = CAN_GATEWAY_IO_BUSY;
  FirmwareFlow_Process();
  assert(test_can_frame_count == 0U);
  assert(fw_queue_count == 1U);
  assert(fw_queue[fw_queue_tail].frame_cursor == 0U);
  test_can_result = CAN_GATEWAY_IO_OK;
  FirmwareFlow_Process();
  assert(test_can_frame_count == 8U);
  assert(fw_queue_count == 0U);

  Test_Reset();
  Test_SendBegin(8U);
  Test_SendBlock(0x100U, 0U, 0U, 0U, 8U);
  test_can_result = CAN_GATEWAY_IO_ERROR;
  FirmwareFlow_Process();
  assert(fw_queue_count == 0U);
  assert(fw_credit_available == 15U);
  assert(test_ack_packets[test_ack_count - 1U][24] ==
         FW_FLOW_STATUS_FORWARD_FAILED);
  assert(Test_ReadU16Le(&test_ack_packets[test_ack_count - 1U][20]) == 0U);
}

/** 验证 ACK 传输层 BUSY 时不重复扣 credit，恢复后只发送一次。 */
static void Test_AckBusyRetry(void)
{
  uint32_t ack_before;

  Test_Reset();
  Test_SendBegin(8U);
  ack_before = test_ack_count;
  Test_SendBlock(0x100U, 0U, 0U, 0U, 8U);
  test_send_result = CAN_GATEWAY_IO_BUSY;
  FirmwareFlow_Process();
  assert(test_ack_count == ack_before);
  assert(fw_ack_credit_return == 1U);
  FirmwareFlow_Process();
  assert(test_ack_count == ack_before);
  assert(fw_ack_credit_return == 1U);

  test_send_result = CAN_GATEWAY_IO_OK;
  FirmwareFlow_Process();
  assert(test_ack_count == ack_before + 1U);
  assert(Test_ReadU16Le(&test_ack_packets[ack_before][20]) == 1U);
  FirmwareFlow_Process();
  assert(test_ack_count == ack_before + 1U);
}

/** 验证 CRC 错误包被拒绝并产生 INVALID_BLOCK 强制 ACK。 */
static void Test_InvalidCrc(void)
{
  uint8_t payload[4];
  uint8_t packet[FW_FLOW_RX_PACKET_SIZE];
  uint16_t length;

  Test_Reset();
  Test_WriteU32Le(payload, 64U);
  length = Test_BuildPacket(FW_FLOW_CMD_BEGIN,
                            payload, sizeof(payload), packet);
  packet[16] ^= 0x01U;
  Test_FeedPacket(packet, length);
  FirmwareFlow_Process();
  assert(test_ack_count == 1U);
  assert(test_ack_packets[0][24] == FW_FLOW_STATUS_INVALID_BLOCK);
  assert(fw_session_active == 0U);
}

int main(void)
{
  Test_ClassicLengths();
  Test_FdLengths();
  Test_MultiBlock(128U);
  Test_MultiBlock(64U * 16U);
  Test_MultiBlock(1000U);
  Test_MultiBlock(10243U);
  Test_QueueFullAndRetry();
  Test_CanBusyAndFailure();
  Test_AckBusyRetry();
  Test_InvalidCrc();
  puts("firmware_flow_host_test: PASS");
  return 0;
}
