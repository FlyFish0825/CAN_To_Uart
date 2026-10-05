/**
 * @file firmware_flow.c
 * @brief AA59 固件块解析、信用流控、CAN 分帧和 FLOW_ACK 调度。
 *
 * 接收端先验证完整包的版本、长度、CRC 和帧尾，再按 block_index/offset
 * 顺序入固定队列。主循环只在 CAN 核心可接收时推进 frame_cursor；所有
 * 分片进入 FDCAN 硬件 FIFO 后才返还 credit，避免把软件入队误报为完成。
 */
#include "firmware_flow.h"

#include <string.h>

typedef enum
{
  FW_PARSER_WAIT_START = 0U, /* 等待 AA 帧头。 */
  FW_PARSER_WAIT_FAMILY, /* 已收到 AA，等待协议族 59。 */
  FW_PARSER_READ_HEADER, /* 正在读取固定头部字段。 */
  FW_PARSER_READ_REST /* 正在读取负载、CRC 和帧尾。 */
} FirmwareFlowParserState_t;

typedef struct
{
  /** 该逻辑块转发到的 CAN 标识符。 */
  uint32_t can_id; /* 块对应的 CAN 标识符。 */
  uint32_t block_index; /* 块在固件传输中的逻辑序号。 */
  uint32_t offset; /* 块数据在固件镜像中的字节偏移。 */
  uint8_t can_flags; /* 扩展、FD、BRS 和远程帧标志。 */
  uint8_t valid_len; /* data 中实际有效的固件字节数。 */
  uint8_t is_last; /* 该块是否覆盖镜像末尾。 */
  uint8_t frame_cursor; /* 下一个待提交的 CAN 分帧索引。 */
  uint8_t frame_count; /* 该逻辑块需要拆成的 CAN 帧数。 */
  uint8_t completed_frames; /* 已收到成功完成通知的 CAN 帧数。 */
  uint8_t forward_failed; /* 是否已有分帧提交失败。 */
  uint32_t completion_token; /* 交给核心完成回调的唯一令牌。 */
  uint8_t data[FW_FLOW_BLOCK_SIZE]; /* 块数据的本地副本。 */
} FirmwareFlowBlock_t;

#define FW_FLOW_FRAME_HEADER_SIZE 16U /* AA59 固定头部长度。 */
#define FW_FLOW_ACK_PAYLOAD_SIZE  14U /* FLOW_ACK 负载字段总长度。 */
#define FW_FLOW_PARSER_TIMEOUT_MS 1000U /* 半帧无新字节时的解析超时。 */
#define FW_FLOW_FAMILY            0x59U /* 固件可靠传输协议族字节。 */
#define FW_FLOW_VERSION           0x01U /* 当前固件流控协议版本。 */
#define FW_FLOW_TARGET_SYSTEM     0x00U /* 默认目标系统编号。 */

static FirmwareFlowBlock_t fw_queue[FW_FLOW_QUEUE_BLOCKS]; /* 待转发固件块环形队列。 */
static uint16_t fw_queue_head = 0U; /* 新块写入位置。 */
static uint16_t fw_queue_tail = 0U; /* 当前消费块位置。 */
static uint16_t fw_queue_count = 0U; /* 队列中尚未完成的块数。 */

static FirmwareFlowParserState_t fw_parser_state = FW_PARSER_WAIT_START;
static uint8_t fw_rx_packet[FW_FLOW_RX_PACKET_SIZE]; /* 当前 AA59 半帧缓存。 */
static uint16_t fw_rx_index = 0U; /* 当前缓存下一个写入偏移。 */
static uint16_t fw_rx_expected_size = 0U; /* 固定头和 payload 确定后的完整包长度。 */
static uint32_t fw_parser_last_tick = 0U; /* 最近一次收到 AA59 字节的时间。 */

static CanGatewayTransportOps_t fw_transport = {0}; /* 固件流控使用的 CAN 发送接口。 */
static uint8_t fw_session_active = 0U; /* 是否存在可接收 DATA_BLOCK 的会话。 */
static uint8_t fw_last_block_received = 0U; /* 是否已经收到覆盖镜像末尾的块。 */
static uint8_t fw_target = FW_FLOW_TARGET_SYSTEM; /* 当前会话目标系统编号。 */
static uint32_t fw_size = 0U; /* 当前固件镜像总字节数。 */
static uint32_t fw_next_block_index = 0U; /* 下一个期望的逻辑块序号。 */
static uint32_t fw_next_offset = 0U; /* 下一个期望的镜像偏移。 */
static uint16_t fw_credit_available = 0U; /* 允许发送方继续提交的块数。 */

static uint8_t fw_ack_pending = 0U; /* 是否有待发送 FLOW_ACK。 */
static uint8_t fw_ack_force = 0U; /* 是否忽略累计/时间阈值立即发送 ACK。 */
static uint8_t fw_ack_status = FW_FLOW_STATUS_OK; /* 当前 ACK 的状态码。 */
static uint16_t fw_ack_credit_return = 0U; /* 尚未在 ACK 中返还的 credit 数。 */
static uint32_t fw_ack_sequence = 0U; /* 下一个 FLOW_ACK 的序号。 */
static uint32_t fw_last_ack_tick = 0U; /* 最近一次成功提交 ACK 的时间。 */
static uint32_t fw_last_completed_block = 0xFFFFFFFFUL; /* 最近完成块序号，无完成时为无效值。 */
/* 每个已接收块使用唯一 token，避免新会话与旧的在途回调发生编号冲突。 */
static uint32_t fw_next_completion_token = 1U;

/* 调试统计：这些变量可在调试器中直接观察，不参与传输控制。 */
static volatile uint32_t fw_blocks_received = 0U;
static volatile uint32_t fw_blocks_completed = 0U;
static volatile uint32_t fw_blocks_failed = 0U;
static volatile uint32_t fw_credit_sent = 0U;
static volatile uint32_t fw_ack_sent = 0U;
static volatile uint32_t fw_queue_high_watermark = 0U;
static volatile uint32_t fw_queue_full_count = 0U;
static volatile uint32_t fw_invalid_block_count = 0U;
static volatile uint32_t fw_forward_fail_count = 0U;

/** 使用 CRC-16-CCITT 校验 AA59 FAMILY 至 payload 末尾的数据。 */
static uint16_t FirmwareFlow_Crc16(const uint8_t *data, uint16_t length)
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
      crc = ((crc & 0x8000U) != 0U) ?
            (uint16_t)((crc << 1U) ^ 0x1021U) :
            (uint16_t)(crc << 1U);
    }
  }
  return crc;
}

/** 从协议缓冲区读取小端序 32 位整数。 */
static uint32_t FirmwareFlow_ReadU32Le(const uint8_t *data)
{
  return ((uint32_t)data[0]) |
         ((uint32_t)data[1] << 8U) |
         ((uint32_t)data[2] << 16U) |
         ((uint32_t)data[3] << 24U);
}

/** 向协议缓冲区写入小端序 16 位整数。 */
static void FirmwareFlow_WriteU16Le(uint8_t *data, uint16_t value)
{
  data[0] = (uint8_t)(value & 0xFFU);
  data[1] = (uint8_t)(value >> 8U);
}

/** 向协议缓冲区写入小端序 32 位整数。 */
static void FirmwareFlow_WriteU32Le(uint8_t *data, uint32_t value)
{
  data[0] = (uint8_t)(value & 0xFFU);
  data[1] = (uint8_t)((value >> 8U) & 0xFFU);
  data[2] = (uint8_t)((value >> 16U) & 0xFFU);
  data[3] = (uint8_t)((value >> 24U) & 0xFFU);
}

/** 合并待发送 ACK 状态；错误和强制 ACK 优先保留。 */
static void FirmwareFlow_RequestAck(uint8_t status, uint8_t force)
{
  /* 错误状态优先，不能被后续正常完成事件覆盖。 */
  if ((fw_ack_pending == 0U) || (status != FW_FLOW_STATUS_OK))
  {
    fw_ack_status = status;
  }
  fw_ack_pending = 1U;
  if (force != 0U)
  {
    fw_ack_force = 1U;
  }
}

/** 清空会话进度和块队列，但不改变外部传输回调。 */
static void FirmwareFlow_ResetSession(void)
{
  fw_session_active = 0U;
  fw_last_block_received = 0U;
  fw_size = 0U;
  fw_next_block_index = 0U;
  fw_next_offset = 0U;
  fw_credit_available = 0U;
  fw_queue_head = 0U;
  fw_queue_tail = 0U;
  fw_queue_count = 0U;
}

/** 将逻辑数据长度向上取整到 CAN FD 合法 DLC 长度。 */
static uint8_t FirmwareFlow_DlcLength(uint8_t length)
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

/** 计算一个逻辑块需要的经典 CAN 分片数或 CAN FD 单帧数。 */
static uint8_t FirmwareFlow_GetFrameCount(const FirmwareFlowBlock_t *block)
{
  if ((block->can_flags & FW_FLOW_CAN_FLAG_FD) != 0U)
  {
    return 1U;
  }
  return (uint8_t)((block->valid_len + 7U) / 8U);
}

/** 根据当前分片游标生成一个待提交的 CAN/CAN FD 帧。 */
static CanGatewayIoResult_t FirmwareFlow_BuildFrame(
    const FirmwareFlowBlock_t *block,
    CanGatewayCanFrame_t *frame)
{
  uint16_t start;
  uint8_t length;
  uint8_t i;

  if ((block == NULL) || (frame == NULL) ||
      (block->frame_cursor >= block->frame_count))
  {
    return CAN_GATEWAY_IO_ERROR;
  }

  memset(frame, 0, sizeof(*frame));
  frame->id = block->can_id;
  frame->flags = block->can_flags;
  if ((block->can_flags & FW_FLOW_CAN_FLAG_FD) != 0U)
  {
    frame->len = FirmwareFlow_DlcLength(block->valid_len);
    for (i = 0U; i < frame->len; i++)
    {
      frame->data[i] = (i < block->valid_len) ? block->data[i] : 0xFFU;
    }
  }
  else
  {
    start = (uint16_t)block->frame_cursor * 8U;
    length = (uint8_t)(block->valid_len - start);
    if (length > 8U)
    {
      length = 8U;
    }
    frame->id += block->frame_cursor;
    frame->flags &= (uint8_t)~(FW_FLOW_CAN_FLAG_FD |
                               FW_FLOW_CAN_FLAG_BRS |
                               FW_FLOW_CAN_FLAG_REMOTE);
    frame->len = length;
    for (i = 0U; i < length; i++)
    {
      frame->data[i] = block->data[start + i];
    }
  }
  return CAN_GATEWAY_IO_OK;
}

/** 从队首移除当前逻辑块，并按成功/失败返还 credit 或生成错误 ACK。 */
static void FirmwareFlow_CompleteCurrentBlock(uint8_t success)
{
  FirmwareFlowBlock_t *block;

  if (fw_queue_count == 0U)
  {
    return;
  }

  block = &fw_queue[fw_queue_tail];
  if (success != 0U)
  {
    fw_blocks_completed++;
    fw_last_completed_block = block->block_index;
    fw_credit_available++;
    fw_ack_credit_return++;
    FirmwareFlow_RequestAck(FW_FLOW_STATUS_OK, block->is_last);
  }
  else
  {
    fw_blocks_failed++;
    fw_forward_fail_count++;
    FirmwareFlow_RequestAck(FW_FLOW_STATUS_FORWARD_FAILED, 1U);
  }

  fw_queue_tail = (uint16_t)((fw_queue_tail + 1U) % FW_FLOW_QUEUE_BLOCKS);
  fw_queue_count--;
}

/**
 * @brief 接收核心 CAN 发送队列的硬件提交结果。
 *
 * token 用来定位仍在可靠队列中的逻辑块。一个逻辑块可能拆成多个经典
 * CAN 帧，只有其所有分片都成功进入 FDCAN 硬件 TX FIFO，主循环才会把
 * 该块标记为完成并返还一个 credit。这里不把“节点应答”作为完成条件。
 */
static void FirmwareFlow_OnCanTxCompletion(void *context,
                                           uint32_t token,
                                           uint8_t success)
{
  uint16_t i;

  (void)context;
  for (i = 0U; i < fw_queue_count; i++)
  {
    uint16_t index = (uint16_t)((fw_queue_tail + i) % FW_FLOW_QUEUE_BLOCKS);
    FirmwareFlowBlock_t *block = &fw_queue[index];
    if (block->completion_token == token)
    {
      if (success != 0U)
      {
        if (block->completed_frames < block->frame_count)
        {
          block->completed_frames++;
        }
      }
      else
      {
        block->forward_failed = 1U;
      }
      return;
    }
  }
}

/** 在有限预算内推进队首块的 CAN 分帧发送。 */
static void FirmwareFlow_ProcessTx(void)
{
  uint8_t budget = 8U;

  while ((fw_queue_count != 0U) && (budget != 0U))
  {
    FirmwareFlowBlock_t *block = &fw_queue[fw_queue_tail];
    CanGatewayCanFrame_t frame;
    CanGatewayIoResult_t result;

    if (block->forward_failed != 0U)
    {
      FirmwareFlow_CompleteCurrentBlock(0U);
      continue;
    }

    if ((block->frame_cursor >= block->frame_count) &&
        (block->completed_frames >= block->frame_count))
    {
      FirmwareFlow_CompleteCurrentBlock(1U);
      continue;
    }

    /* 所有分片已经进入核心队列，等待 FDCAN 硬件提交回调，不能提前返还。 */
    if (block->frame_cursor >= block->frame_count)
    {
      return;
    }

    if (CanGateway_CanTxReady() == 0U)
    {
      return;
    }

    result = FirmwareFlow_BuildFrame(block, &frame);
    if (result != CAN_GATEWAY_IO_OK)
    {
      FirmwareFlow_CompleteCurrentBlock(0U);
      continue;
    }

    result = CanGateway_QueueTrackedCanFrame(&frame,
                                              block->completion_token);
    if (result == CAN_GATEWAY_IO_BUSY)
    {
      /* 当前分片没有入队，保留 frame_cursor，下一轮原样重试。 */
      return;
    }
    if (result != CAN_GATEWAY_IO_OK)
    {
      FirmwareFlow_CompleteCurrentBlock(0U);
      continue;
    }

    block->frame_cursor++;
    budget--;
  }

  /* 当底层测试桩或其他同步发送实现已经完成全部分片时，即使本轮预算
   * 刚好用完，也要在本次服务中释放逻辑块并生成 ACK；异步 FDCAN 路径则
   * 会在后续完成回调到达后再满足这个条件。 */
  if ((fw_queue_count != 0U) &&
      (fw_queue[fw_queue_tail].frame_cursor >=
       fw_queue[fw_queue_tail].frame_count) &&
      (fw_queue[fw_queue_tail].completed_frames >=
       fw_queue[fw_queue_tail].frame_count))
  {
    FirmwareFlow_CompleteCurrentBlock(1U);
  }
}

/** 按 AA59 FLOW_ACK 布局组装当前累计确认状态。 */
static void FirmwareFlow_BuildAck(uint8_t *packet)
{
  uint16_t crc;
  uint8_t *payload = &packet[FW_FLOW_FRAME_HEADER_SIZE];

  memset(packet, 0, FW_FLOW_FRAME_HEADER_SIZE + FW_FLOW_ACK_PAYLOAD_SIZE + 4U);
  packet[0] = 0xAAU;
  packet[1] = FW_FLOW_FAMILY;
  packet[2] = FW_FLOW_VERSION;
  packet[3] = FW_FLOW_CMD_FLOW_ACK;
  packet[4] = 0x02U; /* RESPONSE */
  packet[5] = fw_target;
  FirmwareFlow_WriteU32Le(&packet[6], fw_ack_sequence);
  FirmwareFlow_WriteU16Le(&packet[10], FW_FLOW_ACK_PAYLOAD_SIZE);
  FirmwareFlow_WriteU32Le(&packet[12], HAL_GetTick() * 1000U);

  FirmwareFlow_WriteU32Le(&payload[0], fw_last_completed_block);
  FirmwareFlow_WriteU16Le(&payload[4], fw_ack_credit_return);
  FirmwareFlow_WriteU16Le(&payload[6],
                          (uint16_t)(FW_FLOW_QUEUE_BLOCKS - fw_queue_count));
  payload[8] = fw_ack_status;
  /* payload[9..11] 保留为 0。 */

  crc = FirmwareFlow_Crc16(&packet[1], 15U + FW_FLOW_ACK_PAYLOAD_SIZE);
  FirmwareFlow_WriteU16Le(&packet[30], crc);
  packet[32] = FW_FLOW_FAMILY;
  packet[33] = 0xAAU;
}

/** 按累计数量、低水位和最小间隔规则尝试发送 FLOW_ACK。 */
static void FirmwareFlow_ProcessAck(void)
{
  uint32_t now;
  uint8_t packet[FW_FLOW_FRAME_HEADER_SIZE + FW_FLOW_ACK_PAYLOAD_SIZE + 4U];
  CanGatewayIoResult_t result;

  if ((fw_ack_pending == 0U) || (fw_transport.send_packet == NULL))
  {
    return;
  }

  now = HAL_GetTick();
  if ((fw_ack_force == 0U) &&
      (fw_ack_credit_return < FW_FLOW_ACK_INTERVAL) &&
      (fw_queue_count > FW_FLOW_LOW_WATERMARK) &&
      ((uint32_t)(now - fw_last_ack_tick) < FW_FLOW_ACK_INTERVAL_MS))
  {
    return;
  }

  FirmwareFlow_BuildAck(packet);
  result = fw_transport.send_packet(fw_transport.context,
                                    packet,
                                    sizeof(packet));
  if (result != CAN_GATEWAY_IO_OK)
  {
    return;
  }

  fw_ack_sequence++;
  fw_ack_sent++;
  fw_credit_sent += fw_ack_credit_return;
  fw_ack_credit_return = 0U;
  fw_ack_status = FW_FLOW_STATUS_OK;
  fw_ack_force = 0U;
  fw_ack_pending = 0U;
  fw_last_ack_tick = now;
}

/** 校验 BEGIN 并建立新的固件传输会话和初始 credit。 */
static void FirmwareFlow_HandleBegin(const uint8_t *payload,
                                     uint16_t payload_length,
                                     uint8_t target)
{
  uint32_t size;

  fw_target = target;
  if (payload_length != 4U)
  {
    fw_invalid_block_count++;
    FirmwareFlow_RequestAck(FW_FLOW_STATUS_INVALID_BLOCK, 1U);
    return;
  }

  size = FirmwareFlow_ReadU32Le(payload);
  if ((size == 0U) || (fw_session_active != 0U) ||
      (fw_queue_count != 0U))
  {
    FirmwareFlow_RequestAck(FW_FLOW_STATUS_BUSY, 1U);
    return;
  }

  FirmwareFlow_ResetSession();
  fw_session_active = 1U;
  fw_target = target;
  fw_size = size;
  fw_credit_available = FW_FLOW_INITIAL_CREDIT;
  fw_last_completed_block = 0xFFFFFFFFUL;
  fw_ack_credit_return = 0U;
  FirmwareFlow_RequestAck(FW_FLOW_STATUS_OK, 1U);
}

/** 校验 DATA_BLOCK 的顺序/边界并复制到可靠转发队列。 */
static void FirmwareFlow_HandleData(const uint8_t *payload,
                                    uint16_t payload_length,
                                    uint8_t target)
{
  FirmwareFlowBlock_t *block;
  uint32_t block_index;
  uint32_t offset;
  uint8_t can_flags;
  uint8_t valid_len;

  fw_target = target;
  if ((fw_session_active == 0U) ||
      (payload_length < FW_FLOW_DATA_HEADER_SIZE) ||
      (payload_length > FW_FLOW_MAX_PAYLOAD))
  {
    fw_invalid_block_count++;
    FirmwareFlow_RequestAck(FW_FLOW_STATUS_INVALID_BLOCK, 1U);
    return;
  }

  block_index = FirmwareFlow_ReadU32Le(&payload[5]);
  offset = FirmwareFlow_ReadU32Le(&payload[9]);
  can_flags = payload[4];
  valid_len = payload[13];

  if ((valid_len == 0U) || (valid_len > FW_FLOW_BLOCK_SIZE) ||
      (payload_length != (uint16_t)(FW_FLOW_DATA_HEADER_SIZE + valid_len)) ||
      (block_index != fw_next_block_index) ||
      (offset != fw_next_offset) ||
      (offset > fw_size) ||
      (valid_len > (uint32_t)(fw_size - offset)) ||
      ((can_flags & (uint8_t)~(FW_FLOW_CAN_FLAG_EXTENDED |
                               FW_FLOW_CAN_FLAG_FD |
                               FW_FLOW_CAN_FLAG_BRS |
                               FW_FLOW_CAN_FLAG_REMOTE)) != 0U) ||
      ((can_flags & FW_FLOW_CAN_FLAG_REMOTE) != 0U) ||
      (fw_last_block_received != 0U))
  {
    fw_invalid_block_count++;
    FirmwareFlow_RequestAck(FW_FLOW_STATUS_INVALID_BLOCK, 1U);
    return;
  }

  if ((fw_credit_available == 0U) ||
      (fw_queue_count >= FW_FLOW_HIGH_WATERMARK) ||
      (fw_queue_count >= FW_FLOW_QUEUE_BLOCKS))
  {
    fw_queue_full_count++;
    FirmwareFlow_RequestAck(FW_FLOW_STATUS_QUEUE_FULL, 1U);
    return;
  }

  block = &fw_queue[fw_queue_head];
  memset(block, 0, sizeof(*block));
  block->can_id = FirmwareFlow_ReadU32Le(&payload[0]);
  block->can_flags = can_flags;
  block->block_index = block_index;
  block->offset = offset;
  block->valid_len = valid_len;
  block->is_last = (offset + valid_len == fw_size) ? 1U : 0U;
  block->frame_count = FirmwareFlow_GetFrameCount(block);
  block->completion_token = fw_next_completion_token++;
  if (fw_next_completion_token == 0U)
  {
    /* 0 保留为“未跟踪”语义；32 位回绕时跳过它。 */
    fw_next_completion_token = 1U;
  }
  memcpy(block->data, &payload[FW_FLOW_DATA_HEADER_SIZE], valid_len);
  fw_queue_head = (uint16_t)((fw_queue_head + 1U) % FW_FLOW_QUEUE_BLOCKS);
  fw_queue_count++;
  fw_credit_available--;
  fw_blocks_received++;
  if (fw_queue_count > fw_queue_high_watermark)
  {
    fw_queue_high_watermark = fw_queue_count;
  }

  fw_next_block_index++;
  fw_next_offset += valid_len;
  if (fw_next_offset == fw_size)
  {
    fw_last_block_received = 1U;
  }
}

/** 仅在最后一块已完成转发后关闭会话。 */
static void FirmwareFlow_HandleEnd(uint8_t target)
{
  fw_target = target;
  if ((fw_session_active == 0U) ||
      (fw_last_block_received == 0U) ||
      (fw_queue_count != 0U))
  {
    FirmwareFlow_RequestAck(FW_FLOW_STATUS_BUSY, 1U);
    return;
  }

  fw_session_active = 0U;
  FirmwareFlow_RequestAck(FW_FLOW_STATUS_OK, 1U);
}

/** 校验完整 AA59 包并按命令分派到 BEGIN/DATA/END 处理器。 */
static void FirmwareFlow_CommitPacket(void)
{
  uint16_t payload_length;
  uint16_t crc_index;
  uint16_t expected_crc;

  payload_length = (uint16_t)fw_rx_packet[10] |
                   ((uint16_t)fw_rx_packet[11] << 8U);
  crc_index = (uint16_t)(FW_FLOW_FRAME_HEADER_SIZE + payload_length);
  expected_crc = FirmwareFlow_Crc16(&fw_rx_packet[1],
                                    (uint16_t)(15U + payload_length));
  if ((fw_rx_packet[crc_index + 2U] != FW_FLOW_FAMILY) ||
      (fw_rx_packet[crc_index + 3U] != 0xAAU) ||
      (expected_crc != ((uint16_t)fw_rx_packet[crc_index] |
                        ((uint16_t)fw_rx_packet[crc_index + 1U] << 8U))))
  {
    fw_invalid_block_count++;
    FirmwareFlow_RequestAck(FW_FLOW_STATUS_INVALID_BLOCK, 1U);
    return;
  }

  switch (fw_rx_packet[3])
  {
    case FW_FLOW_CMD_BEGIN:
      FirmwareFlow_HandleBegin(&fw_rx_packet[16], payload_length,
                               fw_rx_packet[5]);
      break;
    case FW_FLOW_CMD_DATA_BLOCK:
      FirmwareFlow_HandleData(&fw_rx_packet[16], payload_length,
                              fw_rx_packet[5]);
      break;
    case FW_FLOW_CMD_END:
      if (payload_length != 0U)
      {
        FirmwareFlow_RequestAck(FW_FLOW_STATUS_INVALID_BLOCK, 1U);
      }
      else
      {
        FirmwareFlow_HandleEnd(fw_rx_packet[5]);
      }
      break;
    default:
      FirmwareFlow_RequestAck(FW_FLOW_STATUS_INVALID_BLOCK, 1U);
      break;
  }
}

/** 丢弃当前半帧并把 AA59 解析器恢复到等待帧头状态。 */
static void FirmwareFlow_ResetParser(void)
{
  fw_parser_state = FW_PARSER_WAIT_START;
  fw_rx_index = 0U;
  fw_rx_expected_size = 0U;
  fw_parser_last_tick = HAL_GetTick();
}

/** 向 AA59 状态机输入一个字节，完成后触发整帧提交。 */
static void FirmwareFlow_PushByte(uint8_t byte)
{
  switch (fw_parser_state)
  {
    case FW_PARSER_WAIT_START:
      if (byte == 0xAAU)
      {
        fw_rx_packet[0] = byte;
        fw_rx_index = 1U;
        fw_parser_state = FW_PARSER_WAIT_FAMILY;
      }
      break;

    case FW_PARSER_WAIT_FAMILY:
      if (byte == FW_FLOW_FAMILY)
      {
        fw_rx_packet[1] = byte;
        fw_rx_index = 2U;
        fw_parser_state = FW_PARSER_READ_HEADER;
      }
      else if (byte == 0xAAU)
      {
        fw_rx_packet[0] = byte;
        fw_rx_index = 1U;
      }
      else
      {
        FirmwareFlow_ResetParser();
      }
      break;

    case FW_PARSER_READ_HEADER:
      fw_rx_packet[fw_rx_index++] = byte;
      if (fw_rx_index >= FW_FLOW_FRAME_HEADER_SIZE)
      {
        uint16_t payload_length = (uint16_t)fw_rx_packet[10] |
                                  ((uint16_t)fw_rx_packet[11] << 8U);
        if ((fw_rx_packet[2] != FW_FLOW_VERSION) ||
            (payload_length > FW_FLOW_MAX_PAYLOAD))
        {
          fw_invalid_block_count++;
          FirmwareFlow_RequestAck(FW_FLOW_STATUS_INVALID_BLOCK, 1U);
          FirmwareFlow_ResetParser();
        }
        else
        {
          fw_rx_expected_size = (uint16_t)(FW_FLOW_FRAME_HEADER_SIZE +
                                           payload_length + 4U);
          fw_parser_state = FW_PARSER_READ_REST;
        }
      }
      break;

    case FW_PARSER_READ_REST:
      if (fw_rx_index < sizeof(fw_rx_packet))
      {
        fw_rx_packet[fw_rx_index++] = byte;
      }
      if ((fw_rx_expected_size != 0U) &&
          (fw_rx_index >= fw_rx_expected_size))
      {
        FirmwareFlow_CommitPacket();
        FirmwareFlow_ResetParser();
      }
      break;

    default:
      FirmwareFlow_ResetParser();
      break;
  }
}

/** 注册 CAN 发送接口并清空解析器、会话、队列和确认状态。 */
HAL_StatusTypeDef FirmwareFlow_Init(const CanGatewayTransportOps_t *transport)
{
  if ((transport == NULL) || (transport->send_packet == NULL))
  {
    return HAL_ERROR;
  }

  fw_transport = *transport;
  if (CanGateway_SetTxCompletionCallback(FirmwareFlow_OnCanTxCompletion,
                                         NULL) != HAL_OK)
  {
    return HAL_ERROR;
  }
  FirmwareFlow_ResetSession();
  fw_parser_state = FW_PARSER_WAIT_START;
  fw_rx_index = 0U;
  fw_rx_expected_size = 0U;
  fw_ack_pending = 0U;
  fw_ack_force = 0U;
  fw_ack_status = FW_FLOW_STATUS_OK;
  fw_ack_credit_return = 0U;
  fw_ack_sequence = 0U;
  fw_last_ack_tick = HAL_GetTick();
  fw_last_completed_block = 0xFFFFFFFFUL;
  return HAL_OK;
}

/** 输入可分片 AA59 字节流；校验完整帧后更新会话或把数据块入队。 */
void FirmwareFlow_RxFeed(const uint8_t *data, uint16_t length)
{
  uint32_t now;
  uint16_t i;

  if ((data == NULL) || (length == 0U))
  {
    return;
  }
  now = HAL_GetTick();
  if ((fw_parser_state != FW_PARSER_WAIT_START) &&
      ((uint32_t)(now - fw_parser_last_tick) >= FW_FLOW_PARSER_TIMEOUT_MS))
  {
    /* 半帧超时后清空旧帧，避免错误长度永久占用 AA59 路由状态。 */
    FirmwareFlow_ResetParser();
  }

  for (i = 0U; i < length; i++)
  {
    FirmwareFlow_PushByte(data[i]);
    fw_parser_last_tick = now;
  }
}

/** 主循环服务固件块的 CAN 分帧发送以及待发送 FLOW_ACK。 */
void FirmwareFlow_Process(void)
{
  FirmwareFlow_ProcessTx();
  FirmwareFlow_ProcessAck();
}

/** 返回固件块队列占用百分比；读取状态，不改变队列。 */
uint8_t FirmwareFlow_GetQueueUsage(void)
{
  uint32_t percent = ((uint32_t)fw_queue_count * 100U) /
                     FW_FLOW_QUEUE_BLOCKS;
  return (percent > 100U) ? 100U : (uint8_t)percent;
}
